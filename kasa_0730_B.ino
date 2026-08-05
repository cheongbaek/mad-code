// ============================================================
//  B보드 : 조향(PD 위치제어) + 제동 + 주행모드 스위치 (Arduino Mega 2560) - 0730 버전
//  kasa_0727_B.ino 기반. 이번 0730 변경점은 아래 2가지.
//
//  [0730-1] ★ 자율주행/수동조종 모드 스위치(D5) 추가 + 텔레메트리 3필드로 확장 ★
//      배선 : D5 ── ON/OFF 스위치 ── GND  (단순 온오프 스위치, 내부 풀업만 사용.
//             외부 풀업저항 불필요)
//        - 회로 통전(스위치 닫힘) -> D5 = LOW  -> ★자율주행 모드★ -> 보고값 1
//        - 회로 개방(스위치 열림) -> D5 = HIGH -> ★수동조종 모드★ -> 보고값 0
//        - 부팅 시엔 실제 핀 상태를 읽어 그대로 채택(첫 텔레메트리부터 정확)
//        - 토글 스위치 접점 바운스 대비 MODE_CONFIRM_MS(50ms) 안정화 후 확정
//      출력 : "P,<조향각환산값>"  ->  "P,<조향각환산값>,<모드>"
//        예) "P,-40"             ->  "P,-40,1"
//      ※ 필드가 하나 늘었으므로 kasa_ws의 B보드 파서를 반드시 같이 고쳐야 한다.
//      ※ 이 보드는 모드값으로 아무 동작도 바꾸지 않는다 (계측·보고 전용).
//        조향 힘빼기는 아래 [0730-2]의 'x' 명령으로 ROS2/master.py가 직접 지시한다.
//
//  [0730-2] ★ 조향 힘빼기(릴리즈) 명령 추가 : 조향각도 자리에 'x' ★
//      입력 "x,<브레이크출력>" (예: "x,0") 처럼 조향각도 필드에 알파벳 x가 오면
//      조향 DC모터에 힘을 주지 않는다(PWM 0 유지). 수동조종 시 사람이 핸들을 돌릴 때
//      PD가 직전 목표로 계속 구동해 운전자와 힘겨루기하는 것을 막는 용도.
//        - 내부적으로 ST_SETTLED(대기)로 전환 + target_pos를 현재 위치로 재동기화.
//          ST_SETTLED 동안 updateSteer()는 매 주기 dcStop()만 하므로 완전 무동력.
//        - 다시 숫자 각도가 들어오면 그 값으로 ST_ACTIVE 복귀(즉시 재구동).
//        - 대문자 'X'도 동일하게 인정. 브레이크 필드는 평소와 똑같이 처리된다.
//
//  --- 이하 구조는 0727과 동일 ---
//  [0727-1] 가변저항 필터 : '9샘플 중앙값 + 지수평활' 2단
//      조향 DC모터 가변저항은 A2 핀이다 (A1은 리니어 리미트 스위치용으로 예정).
//      중앙값(지연 ~1ms) -> 하드리밋 즉시정지 판정(안전)용
//      지수평활(지연 ~60ms) -> PD의 P/D항 입력 전용. 안전 판정에는 절대 쓰지 않는다.
//  [0727-2] 하드 리밋 탈출 허용 : 리밋을 '더 파고드는 방향'만 차단, 벗어나는 방향은 허용
//
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, A보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<조향각도>,<브레이크출력>"  (콤마 구분, 개행 종료)
//         - 예: "-10,-50" → 조향각 -10, 브레이크 -50
//         - 조향각도 자리에 'x' → 조향 힘빼기 (예: "x,0")  ★[0730-2]★
//         - 브레이크출력은 -255~255 부호있는 값
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "P,<조향각환산값>,<모드>" (평상시) / "STOP" (e-stop 중)
//         - 모드 : 1 = 자율주행(D5 통전), 0 = 수동조종(D5 개방)
//         - e-stop 중에는 기존 규약대로 접두어/필드 없이 "STOP"만 출력
//
//  조향(DC)모터 : A2 가변저항 피드백 PD 위치제어
//    - 목표각도 -40/+40 은 실측 좌/우 하드 리밋(raw)보다
//      SAFETY_MARGIN(10)만큼 안쪽으로 매핑됨
//    - |오차| <= STEER_TOLERANCE_ENTER(3) 상태가 SETTLE_MS(0.5초) 이상 지속되면
//      "도달"로 판정, 모터 정지 후 대기 상태로 전환 (대기 중엔 재구동 안 함)
//      단, 도달판정 타이머는 |오차| > STEER_TOLERANCE_EXIT(6)가 되어야 리셋됨
//    - 하드 리밋 raw값은 dc_0701_potential.ino 실측치 기반
//      (다른 개체로 교체 시 반드시 재측정할 것)
//  리니어(브레이크)모터 : 부호/크기로 열린루프 타이밍 구동 (기존과 동일)
//  리니어 MB - 빨간색, MA - 검은색
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 조향 PWM 0, 리니어 최고출력 체결 방향 2초간 구동 후 정지, "STOP" 출력
//    - e-stop 해제 시 조향 PD 목표를 그 시점의 현재 위치로 재동기화하여
//      해제 순간 과거 목표각도로 급조향하는 것을 방지
//
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- DC 조향모터 (MD20A + 가변저항 A2) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A2;   // ★ 조향 가변저항 (이 보드 전용)
// ※ 과거 A보드로 같은 신호선을 분기했으나, A보드가 nodiff 계열(kasa_0730_A.ino)로
//   바뀌면서 조향각 분기 입력은 제거됨. 좌우 차동은 ROS2(kasa_ws)가 이 보드의
//   텔레메트리 "P,<각도>,<모드>"를 보고 계산한다.

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;
// [계획] 리니어 리미트 스위치 A1 예정 (단순 on/off 스위치이므로 INPUT_PULLUP으로 처리,
//   외부 풀업저항 불필요). 아직 미구현 — 배선/코드 반영 시 이 주석 갱신할 것.

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// --- [0730-1] 주행모드 스위치 (단순 ON/OFF, D5 ── 스위치 ── GND) ---
// 내부 풀업만 사용하므로 외부 저항 불필요.
//   통전(닫힘) = LOW  = 자율주행 모드 -> 보고값 1
//   개방(열림) = HIGH = 수동조종 모드 -> 보고값 0
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

// ================= 실측 좌/우 하드 리밋 (raw, 0~1023) =================
// 다른 가변저항/모터 개체로 교체 시 반드시 재측정 후 갱신할 것
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
// 1단(중앙값) : 제어주기마다 analogRead를 POT_MEDIAN_N회 연속 수행해 중앙값만 채택.
//   ADC 스파이크(모터 노이즈 유입 등)를 '평균에 섞지 않고' 통째로 버리는 것이 목적.
//   Mega의 analogRead는 약 104us이므로 9회 ≈ 0.94ms (제어주기 20ms의 5% 수준).
//   -> 하드리밋 판정(안전)에 쓰는 값. 지연 ~1ms로 즉시값과 사실상 동등.
// 2단(지수평활) : 중앙값 출력에 남는 소진폭 잔떨림을 억제. PD의 P항/D항 입력으로만 사용.
//   -> 지연이 크므로(약 3주기=60ms) 안전 판정에는 절대 쓰지 않는다.
const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;   // -1 = 아직 초기화 안 됨
int   lastPotMedian = 512;     // 최근 중앙값 (하드리밋 판정 / 텔레메트리 공용)

// ================= 조향 도달 판정 (히스테리시스 분리) =================
const int STEER_TOLERANCE_ENTER = 3;   // 이 이하로 좁아지면 도달판정 타이머 시작/유지
const int STEER_TOLERANCE_EXIT  = 6;   // 이 이상으로 벌어져야 "도달 실패"로 재판정(타이머 리셋)
const unsigned long SETTLE_MS = 500;   // 허용범위 유지 시간 -> 도달 판정

#define DIR_CW   HIGH   // 왼쪽 (raw 증가 방향)
#define DIR_CCW  LOW    // 오른쪽 (raw 감소 방향)


// ================= 브레이크(리니어) 열린루프 타이밍 구동 =================
const int BRAKE_MAX = 255;                  // 브레이크출력 크기 상한
const unsigned long LINEAR_RUN_MS = 220;   // 입력받은 방향/크기로 구동하는 시간

// e-stop 시 리니어를 강제로 체결시키는 방향
const uint8_t LINEAR_ENGAGE_DIR = DIR_CW;


// ================= 조향 PD 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int  steer_angle_cmd = 0;     // 마지막으로 수신한 명령 각도 (참고/디버그용)
int  target_pos = 512;        // PD 목표 raw값
int  prev_pos   = 0;          // 미분항 계산용 이전 raw값 (필터링된 값 기준)
unsigned long steer_win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;


// ================= 브레이크(리니어) 상태 =================
int  brake_cmd     = 0;
int  brake_output  = 0;
bool linear_running = false;
unsigned long linear_start_t = 0;


// ================= [0730-1] 주행모드 상태 =================
// auto_mode = true  : 자율주행 (D5 통전/LOW) -> 텔레메트리 1
// auto_mode = false : 수동조종 (D5 개방/HIGH) -> 텔레메트리 0
// 이 보드는 이 값으로 구동 동작을 바꾸지 않는다 (보고 전용).
bool auto_mode = false;               // setup()에서 실제 핀 상태로 프라이밍
uint8_t mode_pin_last = HIGH;         // 마지막으로 관측된 원시 레벨
unsigned long mode_change_t = 0;      // 레벨이 바뀐 시각 (0 = 확정됨/변화 없음)
const unsigned long MODE_CONFIRM_MS = 50;   // 접점 바운스 안정화 시간


// ================= E-stop 상태 =================
bool estop_active = false;

// e-stop 시 리니어 체결 구동 시간 (이 시간 후 리니어 출력 정지)
const unsigned long ESTOP_BRAKE_MS = 2000;
bool estop_latched = false;          // e-stop 진입(엣지) 감지용
unsigned long estop_engage_t = 0;    // 체결 시작 시각

// e-stop 핀 판정: 이 시간 동안 '전부' 개방(HIGH)이어야 발동
// (loop 매회 폴링, 중간에 한 번이라도 단락(LOW)이 읽히면 타이머 리셋)
const unsigned long ESTOP_PIN_CONFIRM_MS = 500;
unsigned long estop_pin_high_t = 0;  // HIGH가 처음 관측된 시각 (LOW로 복귀하면 0)


// ================= 출력용 =================
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearCW(int p); void linearCCW(int p);
void applyEstop(unsigned long now);
int  angleToPot(int angle);
int  potToAngle(int raw);
int  readPotMedian();
int  smoothPot(int med);
void updateSteer(unsigned long now);
void releaseSteer();
void startLinear(int cmd);
bool isValidNumber(const char* s);
bool isReleaseToken(const char* s);
void handleLine(char* line);
void pollSerial();
void updateBrake(unsigned long now);
void updateMode(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;

  // 리니어(브레이크) 체결: 진입 시점부터 ESTOP_BRAKE_MS(2초) 동안만 최고출력 구동 후 정지
  if (!estop_latched) {
    estop_latched  = true;
    estop_engage_t = now;
    digitalWrite(LINEAR_DIR_PIN, LINEAR_ENGAGE_DIR);
    analogWrite(LINEAR_PWM_PIN, BRAKE_MAX);
    brake_output = BRAKE_MAX;
    linear_running = false;
  } else if (brake_output > 0 && now - estop_engage_t >= ESTOP_BRAKE_MS) {
    linearStop();
    brake_output = 0;
  }
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
// 읽으면서 바로 삽입정렬로 정렬해두고 가운데 값을 반환 (9개 기준 약 20회 비교, 부하 무시 가능).
// analogRead 9회 ≈ 0.94ms 소요. 제어주기(20ms) 안에서만 호출할 것.
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
// 'x' 명령을 받았을 때 호출. 조향 DC모터에 힘을 주지 않는 상태로 만든다.
//   - 즉시 PWM 0
//   - ST_SETTLED(대기)로 전환 -> updateSteer()가 매 주기 dcStop()만 수행하므로 무동력 유지
//   - 목표를 현재 위치로 재동기화 -> 잔여 목표가 남아 나중에 급조향하는 것을 방지
// 다시 숫자 각도가 들어오면 handleLine()이 ST_ACTIVE로 되돌려 즉시 재구동한다.
// (lastPotMedian은 updateSteer()가 20ms마다 갱신하므로 여기서 ADC를 또 돌 필요 없음)
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

  // [0727-1] 중앙값 = 하드리밋 판정용(스파이크 제거, 지연 ~1ms)
  //          지수평활 = PD 제어용(잔떨림 제거)
  int med = readPotMedian();
  lastPotMedian = med;              // 텔레메트리도 이 값을 재사용 (추가 ADC 버스트 방지)
  int cur = smoothPot(med);

  // ── 대기 상태: 가변저항 변화 무시, 정지 유지 ──
  //   도달 판정으로 들어온 경우와 [0730-2] 'x' 힘빼기로 들어온 경우가 모두 여기로 온다.
  //   어느 쪽이든 "조향 모터에 힘을 주지 않는다"는 동작은 동일하다.
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
    // ENTER~EXIT 사이(3~6)에서는 판정 타이머를 유지(히스테리시스),
    // EXIT(6)를 넘어서야 "확실히 벗어났다"고 보고 타이머 리셋 + 재구동
    settleTimerRunning = false;

    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    bool wantRawUp = (output > 0);   // 출력>0 -> dcCW -> raw 증가 방향

    // [0727-2] 하드 리밋 게이팅 (페일세이프)
    // 리밋을 '더 파고드는 방향'만 차단하고, 벗어나는 방향은 구동을 허용한다.
    // (기존처럼 무조건 정지시키면 리밋을 한 번 넘어갔을 때 스스로 복귀하지 못함)
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
    // ENTER < absErr <= EXIT : 죽은 영역(dead zone). 모터 정지, 타이머는 건드리지 않음
    // (경계에서 노이즈로 오차가 3~6 사이를 오갈 때 모터가 다시 튀어나가는 것 방지)
    dcStop();
  }
}


// ================= 가변저항 환산 현재 조향각 (텔레메트리용) =================
// updateSteer()가 갱신해둔 중앙값을 재사용 (50ms마다 ADC 9회 버스트를 또 도는 것을 방지)
int readSteerAngle() {
  return potToAngle(lastPotMedian);
}


// ================= [0730-1] 주행모드 스위치 판정 (디바운스) =================
// D5는 INPUT_PULLUP. 스위치가 D5와 GND 사이에 있으므로:
//   닫힘(통전) -> LOW  -> 자율주행 (auto_mode = true,  보고값 1)
//   열림       -> HIGH -> 수동조종 (auto_mode = false, 보고값 0)
// 레벨이 바뀐 뒤 MODE_CONFIRM_MS 동안 같은 레벨이 유지되어야 확정한다
// (토글 스위치 접점 바운스로 모드 플래그가 깜빡이는 것 방지).
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


// ================= 브레이크출력 → 리니어 열린루프 구동 시작 =================
void startLinear(int cmd) {
  if (cmd > 0) {
    linearCW(cmd);
    linear_running = true;
    linear_start_t = millis();
  } else if (cmd < 0) {
    linearCCW(-cmd);
    linear_running = true;
    linear_start_t = millis();
  } else {
    linearStop();
    linear_running = false;
  }
  brake_output = abs(cmd);
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
// 조향각도 자리의 "x" 또는 "X" 한 글자만 인정 (그 외 문자열은 형식 오류로 무시됨)
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
  digitalWrite(LINEAR_DIR_PIN, DIR_CW);
  linearStop();

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // [0730-1] 주행모드 스위치 (단순 ON/OFF, 내부 풀업만 사용)
  // 부팅 시 실제 핀 상태를 그대로 채택해 첫 텔레메트리부터 정확한 모드를 보고한다.
  pinMode(MODE_PIN, INPUT_PULLUP);
  mode_pin_last = digitalRead(MODE_PIN);
  auto_mode     = (mode_pin_last == LOW);
  mode_change_t = 0;

  unsigned long now = millis();
  tele_t = now;

  // 조향 PD: 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  // [0727-1] 중앙값으로 프라이밍해서 부팅 직후 첫 샘플부터 안정된 값으로 시작
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;
}


// ================= 입력 파서 =================
// "<조향각도>,<브레이크출력>" 콤마 구분 2개. 형식이 안 맞으면 무시.
// [0730-2] 조향각도 자리에는 정수 대신 'x'(또는 'X')를 넣어 조향 힘빼기를 지시할 수 있다.
//   예) "x,0" -> 조향 무동력 + 브레이크 0
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
    // [0730-2] 조향 무동력 (사람이 핸들을 돌리는 동안 PD와 힘겨루기 방지)
    releaseSteer();
  } else {
    steer_angle_cmd = constrain(atoi(tok1), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    target_pos = angleToPot(steer_angle_cmd);
    settleTimerRunning = false;
    steer_state = ST_ACTIVE;                    // 새 각도 입력 -> PD 제어 재개
  }

  brake_cmd = constrain(brake, -BRAKE_MAX, BRAKE_MAX);
  startLinear(brake_cmd);
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


// ================= 브레이크(리니어) 제어 (열린루프 타이밍, MD20A) =================
void updateBrake(unsigned long now) {
  if (!linear_running) return;
  if (now - linear_start_t >= LINEAR_RUN_MS) {
    linearStop();
    linear_running = false;
    brake_output = 0;
  }
}


// ================= 출력 =================
// [0730-1] 3필드로 확장: "P,<조향각환산값>,<모드>"
//   조향각환산값 : 가변저항 실측 기반 현재 조향각 (-40~40)
//   모드         : 1 = 자율주행(D5 통전), 0 = 수동조종(D5 개방)
//   e-stop 중에는 기존 규약대로 "STOP"만 출력 (접두어/필드 없음)
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
  pollSerial();

  // [0730-1] 주행모드 스위치는 e-stop 여부와 무관하게 항상 갱신 (보고 전용이므로 안전)
  updateMode(now);

  // E-stop NC: 평상시 LOW(단락), 개방(버튼/단선) 시 HIGH → 발동
  // 디바운스: ESTOP_PIN_CONFIRM_MS 이상 연속 HIGH일 때만 인정 (노이즈 오발동 방지)
  bool pinEstop = false;
  if (digitalRead(ESTOP_PIN) == HIGH) {
    if (estop_pin_high_t == 0) estop_pin_high_t = now;
    pinEstop = (now - estop_pin_high_t >= ESTOP_PIN_CONFIRM_MS);
  } else {
    estop_pin_high_t = 0;
  }
  estop_active = ESTOP_ENABLED && pinEstop;

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      // e-stop 해제 엣지: 체결 구동 중이던 리니어를 반드시 정지
      // (2초 내 해제 시 updateBrake가 못 꺼주는 상태로 남는 것 방지)
      linearStop();
      brake_output = 0;
      estop_latched = false;

      // 조향 PD 목표를 해제 시점의 현재 위치(필터 재초기화 포함)로 재동기화
      // (e-stop 중 가변저항이 밀렸어도 과거 목표로 급조향하지 않도록)
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
