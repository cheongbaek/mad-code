// ============================================================
//  B보드 : 조향(PD 위치제어) + 제동 + 주행모드 스위치 (Arduino Mega 2560) - 0731 버전
//  kasa_0730_B.ino 기반. 이번 0731 변경점은 아래 3가지.
//
//  [0731-1] ★ 리니어 리미트 스위치 D2 + 인터럽트 즉시 차단 ★
//      배선 : D2 ── 리미트 스위치(NC) ── GND   (INPUT_PULLUP, 외부 풀업저항 불필요)
//        - 평상시(접점 닫힘, 통전)   -> D2 = LOW  -> 정상
//        - 밟힘(접점 열림, 회로 개방) -> D2 = HIGH -> ★리미트 도달★
//        (E-stop(13번)과 똑같은 NC 규약이다. 단선에도 '도달'로 읽혀 힘을 빼는 페일세이프)
//      ※ 예정 핀이 A1에서 D2로 변경되었다 (D2 = 외부 인터럽트 사용 가능 핀).
//      왜 인터럽트인가 : 리니어모터는 PWM만 0으로 두면 그 위치에 그대로 머문다. 즉 리미트에
//        닿았을 때 '늦게' 반응하면 기구를 계속 밀어붙인 상태가 된다. 그래서 폴링을 기다리지
//        않고 신호가 바뀌는 즉시 ISR에서 PWM을 0으로 떨어뜨린다.
//        - ISR은 CHANGE로 걸어 밟힘/해제 양쪽 엣지를 다 본다.
//        - 밟힘 : 디바운스 없이 즉시 차단 (늦는 것이 위험하다. 노이즈로 한 번 더 힘을
//          빼는 것은 무해하므로 안전 방향으로 몰아준다)
//        - 해제 : micros() 기준 LIMIT_RELEASE_CONFIRM_US 동안 유지되어야 확정
//          (접점 바운스로 깜빡이는 것만 걸러낸다. millis(1ms 해상도)보다 세밀하게 본다)
//      loop() 최상위에서도 매 회 확인해 리미트 중에는 어떤 브레이크 구동도 시작하지 않는다.
//      ★★ 리미트는 e-stop보다도 우선이다 ★★ e-stop 체결 구동 중에 리미트가 눌리면 ISR이
//        그 즉시 PWM을 끊고, updateLimit()이 진행 중이던 전이를 취소한다. e-stop 상태가
//        유지되는 동안에도 리미트가 걸려 있으면 리니어는 계속 무출력이다.
//      ※ 리미트가 걸린 동안에는 '방향을 가리지 않고' 리니어 출력을 0으로 둔다.
//        브레이크 페달은 사람이 주로 밟고 리니어는 보조 수단이라, 애매할 때 힘을 빼는 쪽이
//        항상 안전하다는 설계 판단에 따른 것이다.
//
//  [0731-2] ★ 브레이크 명령을 3단계(0/1/2)로 변경 + 전이별 열린루프 구동 ★
//      입력의 브레이크 필드 의미가 바뀌었다 (기존 -255~255 부호있는 PWM -> 단계값):
//        0 = 아무것도 안 함 / 1 = 약한 브레이킹 / 2 = 풀브레이킹
//      현재 단계에서 목표 단계로 갈 때만 정해진 방향·PWM·시간으로 한 번 구동한다
//      (리니어는 힘을 빼면 그 위치에 머무르므로, 도달 후에는 PWM 0으로 유지하면 된다):
//        0->1 정방향 100 / 0->2 정방향 255 / 1->2 정방향 100
//        2->1 역방향 100 / 1->0 역방향 255 / 2->0 역방향 255   (모두 0.1초)
//      값은 전부 아래 '브레이크 전이 파라미터' 절에서 수정할 수 있다.
//      방향 규약 : 정방향(체결) = DIR 0(LOW) / 역방향(해제) = DIR 1(HIGH)
//      ※ 0~2 범위를 벗어난 브레이크 값은 그 줄의 '브레이크 필드만' 무시한다(조향은 적용).
//        구 프로토콜(±255 PWM)로 보내는 송신측이 섞여 있어도 큰 값이 풀브레이킹으로
//        오해석되지 않게 하려는 것이다. 송신측(kasa_ws)은 0/1/2로 맞춰야 한다.
//
//  [0731-4] ★★ E-stop 즉시 발동 (500ms 디바운스 제거 + 핀체인지 인터럽트) ★★
//      [기존 문제] 13번 핀이 500ms 연속 개방(HIGH)이어야 발동해, 비상정지가 0.5초 늦었다.
//      [변경] 발동은 '즉시', 해제만 확인 시간을 둔다 (비대칭 설계):
//        - D13은 외부 인터럽트 핀이 아니지만 PB7 = PCINT7이라 **핀체인지 인터럽트**를
//          걸 수 있다. 포트B에서 이 보드가 쓰는 핀은 D13뿐이라 간섭이 없다.
//        - ISR은 개방을 본 즉시 조향·리니어 PWM을 모두 0으로 떨어뜨린다.
//          (리니어 '체결' 구동은 loop의 applyEstop이 맡는다 — 리미트 확인이 필요하고,
//           수 ms 늦어도 무해하다. 반면 진행 중이던 브레이크 해제 구동을 즉시 끊는 것은
//           안전에 도움이 된다)
//        - 짧은 개방 펄스도 놓치지 않도록 estop_edge_seen으로 래치하고 loop가 소비한다.
//        - 해제는 ESTOP_RELEASE_CONFIRM_MS(500ms) 연속 단락(LOW)이어야 인정한다.
//      ※ A보드(kasa_0731_A.ino)와 동일한 방식이다. 두 보드가 같은 13번 라인을 병렬로 본다.
//
//  [0731-3] ★ 조향 가변저항 하드 리밋 실측값 갱신 ★
//      좌 1001 -> 576, 우 751 -> 362 (2026-07-31 재실측)
//      가동 폭이 230 -> 214 카운트로 좁아졌다. PD 게인(KP_S)은 raw 카운트 기준이므로
//      체감 반응이 조금 달라질 수 있다 — 실차에서 확인하고 필요하면 KP_S를 재조정할 것.
//
//  --- 이하 구조는 0730과 동일 ---
//  [0730-1] 자율주행/수동조종 모드 스위치(D5) + 텔레메트리 3필드 "P,<각도>,<모드>"
//      통전(닫힘)=LOW=자율주행(보고 1) / 개방(열림)=HIGH=수동조종(보고 0), 50ms 디바운스
//  [0730-2] 조향 힘빼기 : 조향각도 자리에 'x'(또는 'X') -> 조향 DC모터 무동력 유지
//  [0727-1] 가변저항 필터 : 9샘플 중앙값(안전 판정용) + 지수평활(PD 입력 전용)
//  [0727-2] 하드 리밋 탈출 허용 : 리밋을 '더 파고드는 방향'만 차단
//
//  E-stop 스위치: 13번 핀, NC 방식, A보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//
//  입력 : "<조향각도>,<브레이크단계>"  (콤마 구분, 개행 종료)
//         - 예: "-10,1" → 조향각 -10, 약한 브레이킹
//         - 조향각도 자리에 'x' → 조향 힘빼기 (예: "x,0")
//         - 브레이크단계는 0/1/2  ★[0731-2]에서 변경★
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "P,<조향각환산값>,<모드>" (평상시) / "STOP" (e-stop 중)
//
//  조향(DC)모터 : A2 가변저항 피드백 PD 위치제어
//  리니어(브레이크)모터 : 단계 전이마다 열린루프 타이밍 구동
//  리니어 MB - 빨간색, MA - 검은색
//  E-stop 동작 : 조향 PWM 0, ★브레이크 단계 2(풀브레이킹)와 동일한 작용★, "STOP" 출력
//    - 전용 타이밍/출력을 두지 않고 브레이크 전이 파라미터를 그대로 쓴다(applyEstop 참고).
//      현재 단계에 따라 0->2(정방향 255) 또는 1->2(정방향 100)로 가고, 이미 2면 재가압 없음.
//    - 리미트가 걸려 있으면 e-stop 체결 구동도 하지 않는다 (리미트가 항상 우선)
//
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- DC 조향모터 (MD20A + 가변저항 A2) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A2;   // ★ 조향 가변저항 (이 보드 전용)

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- [0731-1] 리니어 리미트 스위치 (D2 ── NC 스위치 ── GND, 내부 풀업) ---
// 평상시 LOW(통전) / 밟히면 HIGH(개방). D2는 외부 인터럽트를 걸 수 있는 핀이다.
const uint8_t LIMIT_PIN = 2;
const bool LIMIT_ENABLED = true;   // false로 두면 리미트 감시 비활성(배선 전 테스트용)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
// D13 = PB7 = PCINT7 이라 핀체인지 인터럽트로 즉시 감지할 수 있다 ([0731-4]).
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// --- [0730-1] 주행모드 스위치 (단순 ON/OFF, D5 ── 스위치 ── GND) ---
const uint8_t MODE_PIN = 5;


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 조향 PD 게인 (여기서 조절) =================
float KP_S = 6.0f;
float KD_S = 0.1f;

// ================= 조향 PWM 상한/하한 =================
const int STEER_MIN_PWM = 110;
const int STEER_MAX_PWM = 255;

// ================= 조향 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;   // dc_0702_pd.ino와 동일 (PD게인 호환)

// ================= 조향 입력 각도 범위 =================
const int STEER_ANGLE_MAX =  40;
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

// ================= [0731-3] 실측 좌/우 하드 리밋 (raw, 0~1023) =================
// 2026-07-31 재실측값. 다른 가변저항/모터 개체로 교체 시 반드시 재측정 후 갱신할 것
const int RAW_LEFT_LIMIT  = 576;   // 왼쪽 끝 (하드 리밋)
const int RAW_RIGHT_LIMIT = 362;   // 오른쪽 끝 (하드 리밋)

// ================= 조향 안전 여유값 =================
const int SAFETY_MARGIN = 10;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// -40도/+40도에 대응하는 목표 raw값 (하드 리밋보다 SAFETY_MARGIN만큼 안쪽)
const int POT_AT_ANGLE_MIN = RAW_LEFT_LIMIT  - SAFETY_MARGIN;   // 각도 -40 -> 이 raw값
const int POT_AT_ANGLE_MAX = RAW_RIGHT_LIMIT + SAFETY_MARGIN;   // 각도 +40 -> 이 raw값

// [0727-2] 하드 리밋을 raw의 상/하한으로 정규화 (좌/우 어느 쪽이 큰 값이든 동일하게 동작)
// PD 부호 규약상 dcCW(출력>0) = raw 증가 방향, dcCCW(출력<0) = raw 감소 방향이다.
const int RAW_HI_LIMIT = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT) ? RAW_LEFT_LIMIT  : RAW_RIGHT_LIMIT;
const int RAW_LO_LIMIT = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT) ? RAW_RIGHT_LIMIT : RAW_LEFT_LIMIT;

// ================= [0727-1] 가변저항 필터 : 9샘플 중앙값 + 지수평활 =================
const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;   // -1 = 아직 초기화 안 됨
int   lastPotMedian = 512;     // 최근 중앙값 (하드리밋 판정 / 텔레메트리 공용)

// ================= 조향 도달 판정 (히스테리시스 분리) =================
const int STEER_TOLERANCE_ENTER = 3;   // 이 이하로 좁아지면 도달판정 타이머 시작/유지
const int STEER_TOLERANCE_EXIT  = 6;   // 이 이상으로 벌어져야 "도달 실패"로 재판정
const unsigned long SETTLE_MS = 500;   // 허용범위 유지 시간 -> 도달 판정

#define DIR_CW   HIGH   // 조향 왼쪽 (raw 증가 방향)
#define DIR_CCW  LOW    // 조향 오른쪽 (raw 감소 방향)


// ================= [0731-2] 브레이크 단계 정의 =================
// 송신측(kasa_ws)이 주는 값의 의미. 이 범위를 벗어난 값은 브레이크 필드만 무시한다.
const uint8_t BRAKE_NONE = 0;   // 아무것도 안 함
const uint8_t BRAKE_SOFT = 1;   // 약한 브레이킹
const uint8_t BRAKE_FULL = 2;   // 풀브레이킹
const uint8_t BRAKE_LEVEL_MAX = 2;

// ================= [0731-2] 리니어 방향 규약 =================
// 정방향(브레이크를 밟는 방향) = DIR 0(LOW) / 역방향(놓는 방향) = DIR 1(HIGH)
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH

// ================= [0731-2] 브레이크 전이 파라미터 (여기서 조절) =================
// 단계가 바뀔 때만 아래 방향·PWM으로 해당 시간 동안 한 번 구동한다.
// 구동이 끝나면 PWM 0으로 두며, 리니어는 그 위치에 그대로 머문다.
const int           BRAKE_SOFT_ENGAGE_PWM = 100;   // 0 -> 1 (약한 브레이킹 진입, 정방향)
const unsigned long BRAKE_SOFT_ENGAGE_MS  = 100;
const int           BRAKE_FULL_ENGAGE_PWM = 255;   // 0 -> 2 (풀브레이킹 진입, 정방향)
const unsigned long BRAKE_FULL_ENGAGE_MS  = 100;
const int           BRAKE_SOFT_TO_FULL_PWM = 100;  // 1 -> 2 (약 -> 풀, 정방향)
const unsigned long BRAKE_SOFT_TO_FULL_MS  = 100;
const int           BRAKE_FULL_TO_SOFT_PWM = 100;  // 2 -> 1 (풀 -> 약, 역방향)
const unsigned long BRAKE_FULL_TO_SOFT_MS  = 100;
const int           BRAKE_SOFT_RELEASE_PWM = 255;  // 1 -> 0 (약 해제, 역방향)
const unsigned long BRAKE_SOFT_RELEASE_MS  = 100;
const int           BRAKE_FULL_RELEASE_PWM = 255;  // 2 -> 0 (풀 해제, 역방향)
const unsigned long BRAKE_FULL_RELEASE_MS  = 100;

// ※ e-stop 전용 리니어 파라미터는 두지 않는다. e-stop은 '브레이크 단계 2(풀브레이킹)'와
//   정확히 같은 작용이므로 위 전이 파라미터를 그대로 쓴다 (applyEstop 참고).

// ================= [0731-1] 리미트 스위치 판정 파라미터 (여기서 조절) =================
// 밟힘은 디바운스 없이 즉시 차단한다(안전 방향). 해제만 이 시간 동안 유지되어야 인정한다.
// micros 단위로 두는 이유 : 반응이 늦으면 기구를 계속 밀어붙이므로, 접점 바운스만 겨우
// 걸러낼 만큼 짧게(수 ms 이하) 잡아야 한다.
const unsigned long LIMIT_RELEASE_CONFIRM_US = 3000;   // 3ms


// ================= 조향 PD 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int  steer_angle_cmd = 0;     // 마지막으로 수신한 명령 각도 (참고/디버그용)
int  target_pos = 512;        // PD 목표 raw값
int  prev_pos   = 0;          // 미분항 계산용 이전 raw값 (필터링된 값 기준)
unsigned long steer_win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;


// ================= [0731-2] 브레이크(리니어) 상태 =================
uint8_t brake_level     = BRAKE_NONE;   // 지금 물려 있다고 보는 단계 (구동 완료 기준)
uint8_t brake_cmd_level = BRAKE_NONE;   // 수신된 목표 단계
uint8_t linear_target_level = BRAKE_NONE;   // 진행 중인 구동이 끝나면 확정될 단계
int  brake_output   = 0;                // 지금 리니어에 주고 있는 PWM (디버그용)
bool linear_running = false;
unsigned long linear_start_t = 0;
unsigned long linear_run_ms  = 0;


// ================= [0731-1] 리미트 스위치 상태 =================
// ISR이 갱신하는 값은 volatile. 4바이트(unsigned long) 읽기는 원자적이지 않으므로
// loop에서 읽을 때 noInterrupts()로 감싼다.
volatile bool          limit_hit       = false;   // 지금 밟혀 있는가 (ISR 즉시 갱신)
volatile unsigned long limit_change_us = 0;       // 마지막 엣지 시각 (micros)
bool limit_release_pending = false;   // 해제 신호를 봤지만 아직 확정 전


// ================= [0730-1] 주행모드 상태 =================
bool auto_mode = false;               // setup()에서 실제 핀 상태로 프라이밍
uint8_t mode_pin_last = HIGH;         // 마지막으로 관측된 원시 레벨
unsigned long mode_change_t = 0;      // 레벨이 바뀐 시각 (0 = 확정됨/변화 없음)
const unsigned long MODE_CONFIRM_MS = 50;   // 접점 바운스 안정화 시간


// ================= [0731-4] E-stop 상태 =================
bool estop_active = false;

bool estop_latched = false;          // e-stop에 진입했다 (해제 엣지 처리용)
// ※ 체결 타이머/래치는 더 이상 없다. e-stop이 브레이크 단계 2와 같은 작용이 되면서
//   전이 로직(updateBrake)이 구동 시간과 완료 판정을 전부 관리하기 때문이다.

// ISR이 갱신하는 값들.
//   estop_pin_hit   : 지금 개방(HIGH)인가 — 인터럽트가 본 최신 레벨
//   estop_edge_seen : 개방 엣지를 한 번이라도 봤다 (loop가 읽고 지운다).
//                     아주 짧은 개방 펄스도 놓치지 않기 위한 래치.
volatile bool estop_pin_hit   = false;
volatile bool estop_edge_seen = false;

// ★ 발동은 즉시, 해제만 이 시간 동안 연속 단락(LOW)이어야 인정 ★
// (한 번 걸리면 확실히 멈춰 있도록 하는 비대칭 설계. 0730의 500ms '발동' 지연과는 다르다)
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 500;
unsigned long estop_low_t = 0;   // LOW가 처음 관측된 시각 (HIGH를 보면 0으로 리셋)


// ================= 출력용 =================
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearDrive(uint8_t dir, int p);
void limitISR();
bool limitBlocking();
void updateLimit();
void setupEstopPcint();
void updateEstop(unsigned long now);
void applyEstop(unsigned long now);
int  angleToPot(int angle);
int  potToAngle(int raw);
int  readPotMedian();
int  smoothPot(int med);
void updateSteer(unsigned long now);
void releaseSteer();
bool brakeTransition(uint8_t from, uint8_t to, uint8_t* dir, int* pwm, unsigned long* ms);
void updateBrake(unsigned long now);
bool isValidNumber(const char* s);
bool isReleaseToken(const char* s);
void handleLine(char* line);
void pollSerial();
void updateMode(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop() { analogWrite(LINEAR_PWM_PIN, 0); }

// [0731-2] 방향은 LINEAR_FWD(정, 체결) / LINEAR_REV(역, 해제) 둘 중 하나
void linearDrive(uint8_t dir, int p) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255));
}


// ================= [0731-1] 리미트 스위치 ISR =================
// D2 CHANGE 인터럽트. 밟힘을 본 '즉시' 리니어 PWM을 0으로 떨어뜨리는 것이 이 함수의 목적이다.
//   - analogWrite(pin, 0)은 내부적으로 digitalWrite(pin, LOW)라 ISR에서 호출해도 안전하다.
//   - micros()도 ISR에서 호출 가능하다(타이머0 카운터를 읽을 뿐).
//   - 해제(LOW) 쪽은 여기서 판정하지 않는다. 시각만 남기고 loop의 updateLimit()가
//     LIMIT_RELEASE_CONFIRM_US 경과를 확인해 확정한다(접점 바운스 대비).
void limitISR() {
  limit_change_us = micros();
  limit_hit = (digitalRead(LIMIT_PIN) == HIGH);   // NC 개방 = 밟힘
  if (limit_hit) {
    analogWrite(LINEAR_PWM_PIN, 0);              // ★ 즉시 힘 빼기 ★
  }
}


// ================= [0731-4] E-stop 핀체인지 ISR (D13 = PB7 = PCINT7) =================
// 개방(HIGH)을 본 '즉시' 조향·리니어 출력을 끊는 것이 이 함수의 목적이다.
//   - analogWrite(pin, 0)은 내부적으로 digitalWrite(pin, LOW)라 ISR에서 호출해도 안전하다.
//   - 리니어 '체결' 구동은 여기서 하지 않는다. 리미트 상태를 함께 봐야 하고 수 ms 늦어도
//     무해하므로 loop의 applyEstop()이 맡는다. 반면 진행 중이던 브레이크 '해제' 구동을
//     즉시 끊는 것은 안전에 도움이 되므로 여기서 0으로 만든다.
//   - 상태 정리(PD 리셋 등)도 loop 쪽 몫이다.
ISR(PCINT0_vect) {
  if (PINB & (1 << 7)) {            // PB7 == HIGH -> NC 개방 -> e-stop
    estop_pin_hit   = true;
    estop_edge_seen = true;         // 짧은 펄스도 loop가 반드시 보게 래치
    analogWrite(DC_PWM_PIN, 0);     // ★ 즉시 조향 힘빼기 ★
    analogWrite(LINEAR_PWM_PIN, 0); // ★ 즉시 리니어 출력 차단 ★
  } else {
    estop_pin_hit = false;
  }
}

// D13(PB7)만 핀체인지 인터럽트로 열어둔다
void setupEstopPcint() {
  PCMSK0 |= (1 << PCINT7);   // PB7만 감시 대상으로
  PCIFR  |= (1 << PCIF0);    // 설정 중 쌓인 잔여 플래그 클리어
  PCICR  |= (1 << PCIE0);    // 포트B 핀체인지 인터럽트 활성
}


// ================= [0731-4] E-stop 판정 (발동 즉시 / 해제만 확인 시간) =================
// ISR이 본 개방 레벨·엣지와 현재 핀 레벨을 함께 본다.
//   - 개방을 보면 그 자리에서 발동 (디바운스 없음. 늦는 것이 위험하다)
//   - 해제는 ESTOP_RELEASE_CONFIRM_MS 동안 계속 단락(LOW)이어야 인정
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_active = false;
    return;
  }

  bool hit, edge;
  noInterrupts();
  hit  = estop_pin_hit;
  edge = estop_edge_seen;
  estop_edge_seen = false;      // 엣지는 한 번만 소비
  interrupts();

  bool open_now = hit || edge || (digitalRead(ESTOP_PIN) == HIGH);

  if (open_now) {
    estop_active = true;        // ★ 즉시 발동 ★
    estop_low_t = 0;
  } else if (estop_active) {
    if (estop_low_t == 0) {
      estop_low_t = now;        // 단락 관측 시작 -> 해제 확인 타이머
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_active = false;
      estop_low_t = 0;
    }
  } else {
    estop_low_t = 0;
  }
}


// ================= [0731-1] 리미트가 리니어 구동을 막고 있는가 =================
// 밟혀 있는 동안, 그리고 해제 확정 전까지는 어떤 리니어 구동도 시작하지 않는다.
bool limitBlocking() {
  return LIMIT_ENABLED && (limit_hit || limit_release_pending);
}


// ================= [0731-1] 리미트 상태 갱신 (loop 최상위에서 호출) =================
// ISR이 이미 PWM을 떨어뜨렸지만, 여기서 한 번 더 확실히 0으로 두고 진행 중이던 구동을
// 취소한다(ISR은 최소한의 일만 하고 상태 정리는 여기서 한다).
void updateLimit() {
  if (!LIMIT_ENABLED) return;

  bool hit;
  unsigned long changed;
  noInterrupts();
  hit     = limit_hit;
  changed = limit_change_us;
  interrupts();

  if (hit) {
    linearStop();
    brake_output = 0;
    limit_release_pending = true;   // 이후 LOW로 바뀌어도 확정 대기를 거친다
    if (linear_running) {
      linear_running = false;
      // ★ 진행 중이던 전이를 '도달한 것으로 간주'한다 ★
      //   그 방향으로는 더 갈 수 없는 지점이므로, 목표를 그대로 두면 리미트가 살짝
      //   풀릴 때마다 같은 구동을 다시 시작해 접점을 두드리게 된다(채터링).
      brake_level = linear_target_level;
    }
    return;
  }

  // 해제 신호(LOW)를 본 상태 : micros 기준으로 짧게 유지되어야 확정
  if (limit_release_pending) {
    if (micros() - changed >= LIMIT_RELEASE_CONFIRM_US) {
      limit_release_pending = false;   // 해제 확정 -> 다음 주기부터 구동 허용
    } else {
      linearStop();                    // 확정 전에는 계속 힘을 빼둔다
      brake_output = 0;
    }
  }
}


// ================= e-stop 상태에서 매 루프 호출되는 안전 동작 =================
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;
  estop_latched = true;   // 진입 표시 (해제 엣지 처리용)

  // ★ e-stop은 '브레이크 단계 2(풀브레이킹)'와 정확히 같은 작용을 한다 ★
  //   그래서 전용 타이밍/출력을 따로 두지 않고 브레이크 전이 로직을 그대로 재사용한다.
  //   이렇게 하면 아래가 모두 공짜로 따라온다:
  //     - 리미트 최우선 : updateBrake()가 맨 앞에서 limitBlocking()을 보고, 리미트가
  //       걸려 있으면 아무 구동도 시작하지 않는다. 체결 도중 리미트가 눌리면 ISR이 그
  //       즉시 PWM을 끊고 updateLimit()이 전이를 취소한다.
  //     - 현재 단계에 맞는 올바른 전이 : 0->2는 정방향 255, 1->2는 정방향 100,
  //       이미 2면 구동 없음(불필요한 재가압을 하지 않는다).
  //     - 해제 후 단계 모델 일치 : brake_level이 실제로 2가 되므로, 이후 0 명령이 오면
  //       2->0 해제 구동(역방향 255)이 정상적으로 나간다.
  brake_cmd_level = BRAKE_FULL;
  updateBrake(now);
}


// ================= 각도 <-> raw 변환 =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX);
}

int potToAngle(int raw) {
  int angle = map(raw, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}


// ================= [0727-1] 1단 필터 : 9샘플 중앙값 =================
int readPotMedian() {
  int s[POT_MEDIAN_N];
  for (uint8_t i = 0; i < POT_MEDIAN_N; i++) {
    int v = analogRead(DC_POT_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[POT_MEDIAN_N / 2];
}


// ================= [0727-1] 2단 필터 : 지수평활 (PD 입력 전용) =================
int smoothPot(int med) {
  if (steerAdcFiltered < 0) {
    steerAdcFiltered = med;
  } else {
    steerAdcFiltered += STEER_ADC_SMOOTH_ALPHA * ((float)med - steerAdcFiltered);
  }
  return (int)steerAdcFiltered;
}


// ================= [0730-2] 조향 힘빼기 (릴리즈) =================
void releaseSteer() {
  dcStop();
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
  target_pos = lastPotMedian;
  steer_angle_cmd = potToAngle(lastPotMedian);   // 디버그용: 스테일 명령각 대신 현재각
}


// ================= 조향 PD 제어 (CONTROL_WINDOW_MS 주기) =================
void updateSteer(unsigned long now) {
  if (now - steer_win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_win_t = now;

  int med = readPotMedian();
  lastPotMedian = med;              // 텔레메트리도 이 값을 재사용
  int cur = smoothPot(med);

  // ── 대기 상태: 가변저항 변화 무시, 정지 유지 ──
  if (steer_state == ST_SETTLED) {
    dcStop();
    prev_pos = cur;
    return;
  }

  // ── PD 제어 ──
  int err = target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - prev_pos) / dt);
  float output = p + d;
  prev_pos = cur;

  int absErr = abs(err);

  if (absErr <= STEER_TOLERANCE_ENTER) {
    dcStop();
    if (!settleTimerRunning) {
      settleTimerRunning = true;
      settleStart = now;
    } else if (now - settleStart >= SETTLE_MS) {
      steer_state = ST_SETTLED;   // 0.5초 이상 허용범위 유지 -> 도달 판정, 대기로 전환
    }
  } else if (absErr > STEER_TOLERANCE_EXIT) {
    settleTimerRunning = false;

    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    bool wantRawUp = (output > 0);   // 출력>0 -> dcCW -> raw 증가 방향

    // [0727-2] 하드 리밋 게이팅 : 파고드는 방향만 차단, 벗어나는 방향은 허용
    if (wantRawUp && med >= RAW_HI_LIMIT) {
      dcStop();
    } else if (!wantRawUp && med <= RAW_LO_LIMIT) {
      dcStop();
    } else if (wantRawUp) {
      dcCW(spd);
    } else {
      dcCCW(spd);
    }
  } else {
    // ENTER < absErr <= EXIT : 죽은 영역. 모터 정지, 타이머는 유지
    dcStop();
  }
}


// ================= 가변저항 환산 현재 조향각 (텔레메트리용) =================
int readSteerAngle() {
  return potToAngle(lastPotMedian);
}


// ================= [0730-1] 주행모드 스위치 판정 (디바운스) =================
void updateMode(unsigned long now) {
  uint8_t lv = digitalRead(MODE_PIN);

  if (lv != mode_pin_last) {
    mode_pin_last = lv;
    mode_change_t = now;          // 변화 관측 -> 안정화 타이머 시작(재시작)
  } else if (mode_change_t != 0 && now - mode_change_t >= MODE_CONFIRM_MS) {
    auto_mode = (lv == LOW);      // 안정화 완료 -> 확정
    mode_change_t = 0;
  }
}


// ================= [0731-2] 단계 전이 파라미터 조회 =================
// from -> to 로 갈 때 쓸 방향/PWM/시간을 채워준다. 구동이 필요 없으면(같은 단계) false.
bool brakeTransition(uint8_t from, uint8_t to, uint8_t* dir, int* pwm, unsigned long* ms) {
  if (from == to) return false;

  if (from == BRAKE_NONE && to == BRAKE_SOFT) {
    *dir = LINEAR_FWD; *pwm = BRAKE_SOFT_ENGAGE_PWM;  *ms = BRAKE_SOFT_ENGAGE_MS;  return true;
  }
  if (from == BRAKE_NONE && to == BRAKE_FULL) {
    *dir = LINEAR_FWD; *pwm = BRAKE_FULL_ENGAGE_PWM;  *ms = BRAKE_FULL_ENGAGE_MS;  return true;
  }
  if (from == BRAKE_SOFT && to == BRAKE_FULL) {
    *dir = LINEAR_FWD; *pwm = BRAKE_SOFT_TO_FULL_PWM; *ms = BRAKE_SOFT_TO_FULL_MS; return true;
  }
  if (from == BRAKE_FULL && to == BRAKE_SOFT) {
    *dir = LINEAR_REV; *pwm = BRAKE_FULL_TO_SOFT_PWM; *ms = BRAKE_FULL_TO_SOFT_MS; return true;
  }
  if (from == BRAKE_SOFT && to == BRAKE_NONE) {
    *dir = LINEAR_REV; *pwm = BRAKE_SOFT_RELEASE_PWM; *ms = BRAKE_SOFT_RELEASE_MS; return true;
  }
  if (from == BRAKE_FULL && to == BRAKE_NONE) {
    *dir = LINEAR_REV; *pwm = BRAKE_FULL_RELEASE_PWM; *ms = BRAKE_FULL_RELEASE_MS; return true;
  }
  return false;   // 정의되지 않은 조합 (도달 불가) — 호출측이 단계만 맞춘다
}


// ================= [0731-2] 브레이크(리니어) 제어 =================
// 열린루프 타이밍 구동이라 '지금 어디까지 밀렸는지'를 알 수 없다. 그래서 구동을 끝까지
// 마친 뒤에만 그 단계로 확정한다. 구동 중에 목표가 또 바뀌면 현재 구동을 마치고 다음
// 주기에 이어서 전이한다(예: 0->1 구동 중 2 요청 -> 1 확정 후 1->2). 각 구동이 0.1초라
// 지연이 짧고, 위치 모델이 어긋나지 않는다.
void updateBrake(unsigned long now) {
  // ★ 리미트가 걸려 있으면 아무 구동도 시작하지 않는다 (취소는 updateLimit이 처리) ★
  if (limitBlocking()) return;

  if (linear_running) {
    if (now - linear_start_t >= linear_run_ms) {
      linearStop();
      linear_running = false;
      brake_output = 0;
      brake_level = linear_target_level;   // 구동 완료 -> 이 단계로 확정
    }
    return;
  }

  if (brake_level == brake_cmd_level) return;

  uint8_t dir;
  int pwm;
  unsigned long ms;
  if (!brakeTransition(brake_level, brake_cmd_level, &dir, &pwm, &ms)) {
    brake_level = brake_cmd_level;   // 정의되지 않은 조합이면 단계만 동기화
    return;
  }

  linearDrive(dir, pwm);
  linear_running      = true;
  linear_start_t      = now;
  linear_run_ms       = ms;
  linear_target_level = brake_cmd_level;
  brake_output        = pwm;
}


// ================= 입력 형식 검사 (정수, 부호 허용) =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}


// ================= [0730-2] 조향 힘빼기 토큰 검사 =================
bool isReleaseToken(const char* s) {
  return s && (s[0] == 'x' || s[0] == 'X') && s[1] == '\0';
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  // [0731-1] 리미트 스위치 : D2 ── NC ── GND, 내부 풀업. CHANGE 인터럽트로 양쪽 엣지 감시.
  //   부팅 시 실제 핀 상태를 그대로 채택한다(이미 밟힌 채로 켜져도 구동하지 않도록).
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  limit_hit = (digitalRead(LIMIT_PIN) == HIGH);
  limit_change_us = micros();
  limit_release_pending = limit_hit;
  if (LIMIT_ENABLED) {
    attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), limitISR, CHANGE);
  }

  // [0731-4] E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  //   부팅 시 실제 핀 상태를 그대로 채택한다(이미 개방된 채로 켜져도 즉시 정지 상태로).
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_pin_hit   = (digitalRead(ESTOP_PIN) == HIGH);
  estop_edge_seen = estop_pin_hit;
  estop_low_t     = 0;
  if (ESTOP_ENABLED) setupEstopPcint();

  // [0730-1] 주행모드 스위치 (단순 ON/OFF, 내부 풀업만 사용)
  pinMode(MODE_PIN, INPUT_PULLUP);
  mode_pin_last = digitalRead(MODE_PIN);
  auto_mode     = (mode_pin_last == LOW);
  mode_change_t = 0;

  unsigned long now = millis();
  tele_t = now;

  // 조향 PD: 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;
}


// ================= 입력 파서 =================
// "<조향각도>,<브레이크단계>" 콤마 구분 2개. 형식이 안 맞으면 무시.
//   조향각도 : 정수(-40~40) 또는 'x'/'X'(힘빼기)
//   브레이크단계 : 0(없음) / 1(약) / 2(풀)   ★[0731-2]★
//     범위를 벗어난 값은 브레이크 필드만 무시한다(조향은 정상 적용).
void handleLine(char* line) {
  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 || !isValidNumber(tok2)) return;

  // 조향 필드는 '정수' 또는 '힘빼기 토큰(x)' 둘 중 하나여야 한다
  bool release = isReleaseToken(tok1);
  if (!release && !isValidNumber(tok1)) return;

  int brake = atoi(tok2);

  // e-stop 중에는 구동 명령(조향/브레이크) 미적용 → 리니어 재구동 방지
  if (estop_active) return;

  if (release) {
    releaseSteer();
  } else {
    steer_angle_cmd = constrain(atoi(tok1), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    target_pos = angleToPot(steer_angle_cmd);
    settleTimerRunning = false;
    steer_state = ST_ACTIVE;                    // 새 각도 입력 -> PD 제어 재개
  }

  // [0731-2] 0/1/2만 인정. 그 외(구 프로토콜의 ±255 PWM 등)는 브레이크만 무시한다 —
  // 큰 값이 풀브레이킹으로 오해석되면 위험하기 때문이다.
  if (brake >= BRAKE_NONE && brake <= BRAKE_LEVEL_MAX) {
    brake_cmd_level = (uint8_t)brake;
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}


// ================= 출력 =================
// "P,<조향각환산값>,<모드>" / e-stop 중에는 "STOP"만
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("P,");
  Serial.print(readSteerAngle());
  Serial.print(',');
  Serial.println(auto_mode ? 1 : 0);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // ★ [0731-1] 최우선 : 리미트 스위치 ★
  //   ISR이 이미 PWM을 떨어뜨렸더라도, 여기서 상태를 정리하고 힘이 빠진 것을 확정한다.
  //   아래 어떤 로직(브레이크 전이·e-stop 체결)보다 먼저 와야 한다.
  updateLimit();

  // ★ [0731-4] E-stop 판정 : 개방을 보면 즉시 발동, 해제만 500ms 확인 ★
  //   실제 출력 차단은 이미 ISR이 했고, 여기서 상태를 확정한 뒤 applyEstop이 유지한다.
  //   ※ pollSerial보다 앞에 둔다 — handleLine이 e-stop 중 명령을 걸러내므로, 발동 직후
  //     들어온 줄이 한 번 적용되는 틈을 없애기 위함이다.
  updateEstop(now);

  pollSerial();

  // [0730-1] 주행모드 스위치는 e-stop 여부와 무관하게 항상 갱신 (보고 전용이므로 안전)
  updateMode(now);

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      estop_latched = false;

      // [0731-2] 브레이크는 보정하지 않는다 — e-stop 중에도 '단계 2'로 관리되었으므로
      //   brake_level이 이미 실제 상태와 일치한다. 진행 중인 전이가 있으면 아래
      //   updateBrake()가 그대로 마무리하고, 리니어를 여기서 강제로 끊지 않는다
      //   (끊으면 전이가 미완성으로 남아 단계 모델이 어긋난다).

      // 조향 PD 목표를 해제 시점의 현재 위치(필터 재초기화 포함)로 재동기화
      int cur = readPotMedian();
      lastPotMedian    = cur;
      steerAdcFiltered = cur;
      target_pos = cur;
      prev_pos = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
    }
    updateBrake(now);
    updateSteer(now);
  }

  sendOutput(now);
}
