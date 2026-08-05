// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID + 쓰로틀 페달 계측 (Arduino Mega 2560) - 0731 버전
//  kasa_0730_A.ino 기반. 0731 변경점은 아래 1가지뿐이며 제어 로직은 손대지 않았다.
//
//  [0731-1] ★★ E-stop 즉시 발동 (500ms 디바운스 제거 + 핀체인지 인터럽트) ★★
//      [기존 문제] 13번 핀이 500ms 연속 개방(HIGH)이어야 발동하는 디바운스가 있었다.
//        노이즈 오발동을 막으려던 것이지만, 비상정지가 0.5초 늦게 걸린다는 뜻이기도 하다.
//        게다가 판정이 loop 폴링이라 그 사이 제어주기(20ms)마다 PWM이 계속 나갔다.
//      [변경] 발동은 '즉시', 해제만 확인 시간을 둔다 (비대칭 설계):
//        - D13은 외부 인터럽트 핀이 아니지만 PB7 = PCINT7이라 **핀체인지 인터럽트**를
//          걸 수 있다. 포트B에서 이 보드가 쓰는 핀은 D13뿐이므로 PCMSK0을 PB7만 열어
//          다른 핀과 간섭하지 않는다.
//        - ISR은 개방(HIGH)을 본 즉시 좌우 인휠 PWM을 0으로 떨어뜨린다. loop를 기다리지
//          않으므로 최대 지연이 인터럽트 응답 시간(수 us) 수준이다.
//        - 아주 짧은 개방 펄스도 놓치지 않도록 estop_edge_seen으로 래치하고 loop가 소비한다.
//        - 해제는 ESTOP_RELEASE_CONFIRM_MS(500ms) 동안 계속 단락(LOW)이어야 인정한다.
//          발동만 즉시로 만들면 노이즈 한 번에 발동/해제가 깜빡일 수 있는데, 해제를 늦추면
//          '한 번 걸리면 확실히 멈춰 있는' 안전한 쪽으로 몰린다.
//      ※ NC 배선(평상시 LOW, 개방/단선 시 HIGH)과 판정 규약 자체는 그대로다.
//
//  --- 이하 구조는 0730과 동일 ---
//  [0730-1] 쓰로틀 페달 입력(A0) + 텔레메트리 4필드 "S,<좌펄스>,<우펄스>,<쓰로틀raw>"
//      가속페달 가변저항을 A0으로 읽어 0~1023 raw로 보고만 한다(이 보드는 제어에 쓰지 않음).
//      9샘플 중앙값만 적용(지연 ~1ms). 지수평활은 넣지 않았다.
//
//  [0730-2] ★★ 기동 노이즈 대책 : 기동 블랭킹(LAUNCH) + 3점 중앙값 필터 ★★
//      (iw_0730_pid.ino에서 검증한 로직을 그대로 이식. 상세 배경은 그 파일 헤더 참고)
//
//      [문제] "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간에서 홀신호에 수백~1500
//      단위 노이즈가 쏟아진다. 이 모터는 무부하 최대 1320RPM x 64펄스/회전(직결구동)이라
//      20ms 창당 최대 ≈28.2펄스가 물리적 한계이므로 그 이상은 전부 허수다.
//      기존 PID는 이 허수를 오차로 먹어 pwm을 0으로 클램프했고, 램프업이 통째로 리셋 →
//      재램프 → 또 노이즈의 무한루프에 빠졌다.
//
//      [대책 1 : 기동 블랭킹] 정지 상태에서 목표가 0 → 양수로 바뀌면 LAUNCH 상태로
//      들어가 그 구간 동안 피드백을 아예 보지 않는다(개루프).
//        - pwm = FF(target)까지 PWM_SLEW_MAX(+4/cycle)로 램프. PID/코스트/폭주감지 정지
//        - 종료 조건 : (a) 램프 완료 && (b) 정상 펄스 LAUNCH_SETTLE_CYCLES 연속,
//          또는 LAUNCH_MAX_MS 타임아웃.  ★(a)가 없으면 노이즈가 오기도 전에 조기 종료★
//      [대책 2 : 3점 중앙값] 주행 중 남는 단발성 스파이크 제거. 허수 판정된 값은 버퍼에
//      아예 넣지 않아, 노이즈 주기에 '직전 정상값이 자동 대입'되는 효과가 난다.
//
//      ※ 텔레메트리 펄스 필드는 '필터 후 값'(PID가 실제로 본 값)을 보낸다.
//
//  --- 이하 구조는 0727 nodiff와 동일 ---
//  좌우 차동은 이 보드에 없다. ROS2(kasa_ws)가 담당한다:
//     B보드 텔레메트리 "P,<조향각>,<모드>"  ->  kasa_ws가 좌/우 펄스 계산
//     ->  A보드로 "<좌펄스>,<우펄스>" 콤마 2값 전송  ->  A보드는 그대로 추종만
//
//  ★ 배선 (2026-07-30 실차 확인 완료) ★
//     왼쪽  모터 : 펄스 2  -> PID -> PWM 8
//     오른쪽 모터 : 펄스 21 -> PID -> PWM 9
//   좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//
//  입력 : "<값>" 또는 "<왼쪽값>,<오른쪽값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값: 펄스 전용(0~15), 범위 밖/숫자 아님은 무시 (좌우 동일값)
//         - 콤마 2값: 좌/우 독립 — 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지
//  출력 : "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" (평상시) / "STOP" (e-stop 중)
//  제어주기 : 20ms, 출력주기 : 50ms
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) -> e-stop
//    - ★[0731-1] 개방을 본 즉시 발동(PCINT). 해제는 500ms 연속 단락 확인 후★
//    - e-stop 발동 시 직접 PWM 모드도 즉시 해제(펄스 0으로 복귀)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 2;    // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 21;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (펄스 2와 같은 컨트롤러)
const uint8_t PWM_PIN_R = 9;     // 오른쪽 모터 PWM (펄스 21과 같은 컨트롤러)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
// D13 = PB7 = PCINT7 이라 핀체인지 인터럽트로 즉시 감지할 수 있다 ([0731-1]).
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성(배선 전 테스트용)

// --- [0730-1] 쓰로틀(가속) 페달 가변저항 ---
// 이 보드는 이 값으로 제어하지 않는다. 0~1023 raw를 텔레메트리로 보고만 한다.
const uint8_t THROTTLE_PIN = A1;


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= [0730-1] 쓰로틀 필터 : 9샘플 중앙값 =================
const uint8_t THROTTLE_MEDIAN_N = 9;   // 반드시 홀수
int throttleRaw = 0;                   // 최근 중앙값 (0~1023, 텔레메트리용)


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(펄스2-PWM8), 오른쪽(펄스21-PWM9)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ================= ★ 값 해석 경계 ★ =================
const int TARGET_MAX = 15;      // 0~15 = 펄스 목표. 단일 값 입력은 이 범위만 유효
const int PWM_DIRECT_MAX = 255; // 16~255 = 직접 PWM (콤마 2값 형식에서만). 256 이상 = 정지

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= ★ [0730-2] 허수 펄스 판정 경계 (튜닝 지점) ★ =================
const int PULSE_SANITY_MAX = 40;
const int NOISE_HOLD_MAX_CYCLES = 50;   // 50주기 = 1초

// ================= ★ [0730-2] 기동 블랭킹 LAUNCH (튜닝 지점) ★ =================
const int LAUNCH_ENTRY_SPEED_MAX = 1;      // 이 펄스 이하일 때만 '정지 상태'로 보고 진입
const int LAUNCH_SETTLE_CYCLES   = 5;      // 정상 펄스가 이만큼 연속되면 노이즈 종료로 판정
const unsigned long LAUNCH_MAX_MS = 3000;  // 이 시간이 지나면 조건 불충족이어도 PID로 인계

// ================= ★ [0730-2] 주행 중 3점 중앙값 필터 ★ =================
const bool SPEED_MEDIAN_ON = true;
const uint8_t SPEED_MEDIAN_N = 3;   // 3 고정 (median3 함수가 3점 전용)


// ================= 좌/우 PID 상태 (완전 분리) =================
const uint8_t LEFT  = 0;   // 왼쪽  : 펄스 2  피드백 -> PWM 8
const uint8_t RIGHT = 1;   // 오른쪽 : 펄스 21 피드백 -> PWM 9

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터
int   noiseCnt[2]    = {0, 0};   // [0730-2] 연속 허수 펄스 주기 (블랭킹 중엔 세지 않음)

// [0730-2] 기동 블랭킹 상태
bool          launching[2] = {false, false};
unsigned long launch_t[2]  = {0, 0};   // 블랭킹 진입 시각 (타임아웃용)
int           settleCnt[2] = {0, 0};   // 정상 펄스 연속 주기 카운터

// [0730-2] 중앙값 필터용 링버퍼 (정상 판정된 값만 들어간다)
int     spdBuf[2][SPEED_MEDIAN_N] = {{0, 0, 0}, {0, 0, 0}};
uint8_t spdPos[2] = {0, 0};


// ================= 구동 모드 (좌/우 독립) =================
const uint8_t DRIVE_PULSE = 0;   // 펄스 목표 PID 주행 (0~15)
const uint8_t DRIVE_PWM   = 1;   // 직접 PWM 출력 (16~255, 무보호)

uint8_t driveMode[2] = {DRIVE_PULSE, DRIVE_PULSE};
int     pwmDirect[2] = {0, 0};   // 직접 PWM 모드에서 출력할 값


// ================= 인휠 상태 =================
volatile long encCountL = 0;    // 왼쪽 펄스 (2번 핀)
volatile long encCountR = 0;    // 오른쪽 펄스 (21번 핀)
int target[2] = {0, 0};         // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)
int rawSpeed[2] = {0, 0};       // [0730-2] 필터 전 원시 펄스 (진단용)
int useSpeed[2] = {0, 0};       // [0730-2] PID가 실제로 본 값 = 텔레메트리로 보내는 값
unsigned long wheel_t = 0;


// ================= [0731-1] E-stop 상태 =================
bool estop_active = false;

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


// ================= ISR (2번=왼쪽 펄스, 21번=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


// ================= [0731-1] E-stop 핀체인지 ISR (D13 = PB7 = PCINT7) =================
// 개방(HIGH)을 본 '즉시' 좌우 인휠 PWM을 0으로 떨어뜨리는 것이 이 함수의 목적이다.
//   - analogWrite(pin, 0)은 내부적으로 digitalWrite(pin, LOW)라 ISR에서 호출해도 안전하다.
//   - 상태 정리(PID 리셋 등)는 여기서 하지 않는다. loop의 applyEstop()이 맡는다.
//   - 포트B에서 이 보드가 쓰는 핀은 D13뿐이라 다른 핀 변화로 이 ISR이 불릴 일은 없다.
ISR(PCINT0_vect) {
  if (PINB & (1 << 7)) {          // PB7 == HIGH -> NC 개방 -> e-stop
    estop_pin_hit   = true;
    estop_edge_seen = true;       // 짧은 펄스도 loop가 반드시 보게 래치
    analogWrite(PWM_PIN_L, 0);    // ★ 즉시 인휠 출력 차단 ★
    analogWrite(PWM_PIN_R, 0);
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


// ================= [0730-1] 쓰로틀 중앙값 필터 =================
int readThrottleMedian() {
  int s[THROTTLE_MEDIAN_N];
  for (uint8_t i = 0; i < THROTTLE_MEDIAN_N; i++) {
    int v = analogRead(THROTTLE_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[THROTTLE_MEDIAN_N / 2];
}


// ================= [0730-2] 주행펄스 3점 중앙값 (정렬 없이 비교 3회) =================
int median3(int a, int b, int c) {
  int lo = min(a, b);
  int hi = max(a, b);
  return max(lo, min(hi, c));
}

void pushSpeed(uint8_t idx, int v) {
  spdBuf[idx][spdPos[idx]] = v;
  spdPos[idx] = (spdPos[idx] + 1) % SPEED_MEDIAN_N;
}

void fillSpeed(uint8_t idx, int v) {
  for (uint8_t k = 0; k < SPEED_MEDIAN_N; k++) spdBuf[idx][k] = v;
  spdPos[idx] = 0;
}


// ================= 인휠 FF 보간 (목표펄스 -> PWM, 3점 라그랑주 2차보간) =================
float interpFF(float x) {
  if (x <= 0) return 0.0;
  if (x <= ffPulseTable[0]) {
    return ffPwmTable[0] * (x / ffPulseTable[0]);
  }
  if (x >= ffPulseTable[FF_TABLE_N - 1]) {
    return ffPwmTable[FF_TABLE_N - 1];
  }

  int i = 0;
  while (i < FF_TABLE_N - 2 && x > ffPulseTable[i + 1]) i++;

  int i0, i1, i2;
  if (i == 0) { i0 = 0; i1 = 1; i2 = 2; }
  else        { i0 = i - 1; i1 = i; i2 = i + 1; }

  float x0 = ffPulseTable[i0], x1 = ffPulseTable[i1], x2 = ffPulseTable[i2];
  float y0 = ffPwmTable[i0],   y1 = ffPwmTable[i1],   y2 = ffPwmTable[i2];

  float L0 = (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2));
  float L1 = (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2));
  float L2 = (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));

  return y0 * L0 + y1 * L1 + y2 * L2;
}


// ================= [0730-2] 기동 블랭킹 진입/종료 =================
void enterLaunch(uint8_t idx, unsigned long now) {
  launching[idx] = true;
  launch_t[idx] = now;
  settleCnt[idx] = 0;
  noiseCnt[idx] = 0;
  pidI[idx] = 0;
  pidLastErr[idx] = 0;
  pidCoasting[idx] = false;   // 블랭킹 중엔 코스트/캐치 판정을 아예 하지 않는다
  runawayCnt[idx] = 0;
}

void exitLaunch(uint8_t idx, int speed) {
  launching[idx] = false;
  settleCnt[idx] = 0;
  noiseCnt[idx] = 0;
  pidI[idx] = 0;
  pidLastErr[idx] = target[idx] - speed;
}

// ================= [0730-2] 기동 블랭킹 중 개루프 출력 =================
int launchPwm(uint8_t idx) {
  int ffPwm = (int)interpFF((float)target[idx]);
  if (ffPwm > PWM_MAX) ffPwm = PWM_MAX;

  int pwm = pidLastPwm[idx] + PWM_SLEW_MAX;
  if (pwm > ffPwm) pwm = ffPwm;
  if (pwm < 0) pwm = 0;

  pidLastPwm[idx] = pwm;
  return pwm;
}


// ================= 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =================
int updatePid(uint8_t idx, int tgt, int speed) {
  int err = tgt - speed;
  int d = err - pidLastErr[idx];
  pidLastErr[idx] = err;

  float ff = interpFF((float)tgt);

  // 폭주 감지: 지속 과속이면 코스트 진입(PWM 0)
  if (err <= -RUNAWAY_ERR_OVER) {
    runawayCnt[idx]++;
    if (runawayCnt[idx] >= RUNAWAY_CONFIRM_CYCLES) {
      pidCoasting[idx] = true;
      pidI[idx] = 0;
      runawayCnt[idx] = 0;
    }
  } else {
    runawayCnt[idx] = 0;
  }

  // 코스트-캐치: 목표+마진까지 내려오면 PID 재개
  if (pidCoasting[idx] && speed <= tgt + CATCH_MARGIN) {
    pidCoasting[idx] = false;
  }

  int pwm;
  if (pidCoasting[idx]) {
    pwm = 0;
    pidI[idx] = 0;
  } else {
    float iTerm = ki[idx] * pidI[idx];
    pwm = ff + kp[idx] * err + iTerm + kd[idx] * d;

    // anti-windup: 출력 포화 시, 그리고 오차가 클 때(가속 중)는 적분 누적 중단
    if (pwm > 0 && pwm < PWM_MAX && abs(err) < I_ACCUM_ERR_MAX) {
      pidI[idx] += err;
    }

    // 적분 기여분을 ki 값과 무관하게 ±40 pwm로 고정 제한
    iTerm = constrain(ki[idx] * pidI[idx], -40, 40);
    pwm = ff + kp[idx] * err + iTerm + kd[idx] * d;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < 0) pwm = 0;
  }

  pwm = constrain(pwm, 0, PWM_MAX);

  // 슬루레이트 제한: pwm 급상승만 제한(관성 오버슈트 방지), 하강은 즉시 반영
  if (pwm > pidLastPwm[idx] + PWM_SLEW_MAX) pwm = pidLastPwm[idx] + PWM_SLEW_MAX;
  pidLastPwm[idx] = pwm;

  return pwm;
}


// ================= [0730-2] 한쪽 바퀴 갱신: 모드 분기 + 허수 필터 + 블랭킹/PID =================
int updateWheelSide(uint8_t idx, int raw, unsigned long now) {
  rawSpeed[idx] = raw;

  // 직접 PWM 모드는 무보호 경로: 받은 값을 그대로 출력 (피드백을 아예 쓰지 않음)
  if (driveMode[idx] == DRIVE_PWM) {
    if (raw <= PULSE_SANITY_MAX) pushSpeed(idx, raw);
    useSpeed[idx] = SPEED_MEDIAN_ON
                      ? median3(spdBuf[idx][0], spdBuf[idx][1], spdBuf[idx][2])
                      : raw;
    return pwmDirect[idx];
  }

  // ── 허수 판정 : 물리적으로 불가능한 값은 중앙값 버퍼에 넣지 않는다 ──
  bool sane = (raw <= PULSE_SANITY_MAX);
  if (sane) {
    pushSpeed(idx, raw);
    noiseCnt[idx] = 0;
    settleCnt[idx]++;
  } else {
    settleCnt[idx] = 0;
    if (!launching[idx]) noiseCnt[idx]++;   // 블랭킹 중엔 노이즈가 정상이라 세지 않음
  }

  useSpeed[idx] = SPEED_MEDIAN_ON
                    ? median3(spdBuf[idx][0], spdBuf[idx][1], spdBuf[idx][2])
                    : (sane ? raw : useSpeed[idx]);
  int speed = useSpeed[idx];

  // ── 기동 블랭킹 : 피드백 무시하고 FF까지 개루프 램프 ──
  if (launching[idx]) {
    int pwm = launchPwm(idx);

    int ffPwm = (int)interpFF((float)target[idx]);
    if (ffPwm > PWM_MAX) ffPwm = PWM_MAX;
    bool rampDone = (pwm >= ffPwm);
    bool settled  = (settleCnt[idx] >= LAUNCH_SETTLE_CYCLES);
    bool timeout  = (now - launch_t[idx] >= LAUNCH_MAX_MS);

    if ((rampDone && settled) || timeout) {
      exitLaunch(idx, speed);
    }
    return pwm;
  }

  // ── 센서 이상 실패안전 : 허수가 계속 들어오면 코스트로 내린다 ──
  if (NOISE_HOLD_MAX_CYCLES > 0 && noiseCnt[idx] >= NOISE_HOLD_MAX_CYCLES) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
    pidLastPwm[idx] = 0;
    noiseCnt[idx] = 0;
    return 0;
  }

  return updatePid(idx, target[idx], speed);
}


// ================= E-stop 안전 동작 (e-stop 상태에서 매 루프 호출) =================
// ※ ISR이 이미 PWM을 0으로 떨어뜨렸지만, 여기서 다시 0을 써서 확실히 유지하고
//   제어 상태를 정리한다 (ISR은 출력 차단만 하고 상태 정리는 하지 않는다).
void applyEstop() {
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // 양쪽 구동 상태 초기화 (직접 PWM 모드도 강제 해제, 해제 후 재개 시 잔재 방지)
  for (uint8_t s = 0; s < 2; s++) {
    target[s] = 0;
    driveMode[s] = DRIVE_PULSE;
    pwmDirect[s] = 0;
    pidI[s] = 0;
    pidLastErr[s] = 0;
    pidCoasting[s] = false;
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 0부터 다시 램프업)
    runawayCnt[s] = 0;
    launching[s] = false;
    settleCnt[s] = 0;
    noiseCnt[s] = 0;
    // ※ 중앙값 버퍼(spdBuf)는 일부러 건드리지 않는다. e-stop 중에는 계측이 멈추므로
    //   0으로 채워버리면 '바퀴가 멈춤'으로 오인되어, 아직 관성으로 굴러가는 상태에서
    //   해제 직후 블랭킹(개루프 가속)에 진입할 수 있다.
  }
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 입력 형식 검사 (부호 없는 정수만) =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), encISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), encISR_R, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // [0731-1] E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  //   부팅 시 실제 핀 상태를 그대로 채택한다(이미 개방된 채로 켜져도 즉시 정지 상태로).
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_pin_hit   = (digitalRead(ESTOP_PIN) == HIGH);
  estop_edge_seen = estop_pin_hit;
  estop_low_t     = 0;
  if (ESTOP_ENABLED) setupEstopPcint();

  // [0730-1] 쓰로틀 페달 (analogRead는 pinMode 없어도 되지만 명시)
  pinMode(THROTTLE_PIN, INPUT);
  throttleRaw = readThrottleMedian();   // 첫 텔레메트리부터 실제값이 나가도록 프라이밍

  // [0730-2] 부팅 시엔 바퀴가 멈춰 있으므로 중앙값 버퍼를 0으로 채워둔다
  fillSpeed(LEFT, 0);
  fillSpeed(RIGHT, 0);

  unsigned long now = millis();
  wheel_t = tele_t = now;
}


// ================= 값 파싱 =================
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}


// ================= 펄스 모드 목표 적용 =================
void setPulseTarget(uint8_t idx, int newTarget, unsigned long now) {
  if (driveMode[idx] == DRIVE_PWM) {
    driveMode[idx] = DRIVE_PULSE;
    pwmDirect[idx] = 0;
    pidI[idx] = 0;
    pidLastErr[idx] = 0;
    pidLastPwm[idx] = 0;      // 슬루 기준점 리셋: 캐치 후 0부터 다시 램프업
    runawayCnt[idx] = 0;
    noiseCnt[idx] = 0;
    target[idx] = newTarget;

    if (newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
      enterLaunch(idx, now);
    } else {
      launching[idx] = false;
      pidCoasting[idx] = true;
    }
    return;
  }

  // ★ [0730-2] 기동 블랭킹 진입 판정 : 멈춘 상태(0)에서 처음 양수 목표를 받을 때만 ★
  if (target[idx] == 0 && newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
    target[idx] = newTarget;
    enterLaunch(idx, now);
    return;
  }

  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제 (좌우 개별)
  if (newTarget < target[idx]) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
    if (newTarget == 0) launching[idx] = false;   // 정지 명령은 블랭킹보다 우선
  } else if (newTarget > target[idx]) {
    pidCoasting[idx] = false;
  }
  target[idx] = newTarget;
}


// ================= 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =================
void applySide(uint8_t idx, long v, unsigned long now) {
  if (v <= TARGET_MAX) {
    setPulseTarget(idx, (int)v, now);
  } else if (v <= PWM_DIRECT_MAX) {
    driveMode[idx] = DRIVE_PWM;
    pwmDirect[idx] = (int)v;
    launching[idx] = false;   // 직접 PWM은 무보호 경로 — 블랭킹도 적용하지 않는다
  } else {
    setPulseTarget(idx, 0, now);
  }
}


// ================= 입력 파서 =================
void handleLine(char* line, unsigned long now) {
  if (strchr(line, ' ')) return;   // 공백 포함 줄은 형식 오류 (기존과 동일하게 무시)

  char* comma = strchr(line, ',');
  if (comma) {
    *comma = '\0';
    char* tokR = comma + 1;
    if (strchr(tokR, ',')) return;   // 콤마 2개 이상 → 무시

    long vL = parseValue(line);
    long vR = parseValue(tokR);
    if (vL < 0 || vR < 0) return;

    // e-stop 중에는 구동 명령 미적용
    if (estop_active) return;
    applySide(LEFT, vL, now);
    applySide(RIGHT, vR, now);
  } else {
    // 단일 값은 무조건 펄스 모드: 0~15만 받고 그 외는 무시
    long v = parseValue(line);
    if (v < 0 || v > TARGET_MAX) return;

    if (estop_active) return;
    setPulseTarget(LEFT, (int)v, now);
    setPulseTarget(RIGHT, (int)v, now);
  }
}

void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf, now);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}


// ================= 인휠 제어 (좌우 독립: 펄스 PID 또는 직접 PWM) =================
void updateWheel(unsigned long now) {
  if (now - wheel_t < CONTROL_WINDOW_MS) return;
  wheel_t += CONTROL_WINDOW_MS;

  noInterrupts();
  long cL = encCountL;
  long cR = encCountR;
  encCountL = 0;
  encCountR = 0;
  interrupts();

  // [0730-1] 쓰로틀 페달 계측 (제어에는 쓰지 않고 텔레메트리로만 보고)
  throttleRaw = readThrottleMedian();

  int pwmL = updateWheelSide(LEFT,  (int)cL, now);   // 왼쪽 : 펄스 2 -> PWM 8
  int pwmR = updateWheelSide(RIGHT, (int)cR, now);   // 오른쪽 : 펄스 21 -> PWM 9

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= [0731-1] E-stop 판정 (발동 즉시 / 해제만 확인 시간) =================
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


// ================= 출력 =================
// "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" / e-stop 중에는 "STOP"만
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("S,");
  Serial.print(useSpeed[LEFT]);
  Serial.print(',');
  Serial.print(useSpeed[RIGHT]);
  Serial.print(',');
  Serial.println(throttleRaw);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // ★ [0731-1] 최우선 : E-stop 판정 (개방을 보면 즉시 발동) ★
  //   실제 출력 차단은 이미 ISR이 했고, 여기서 상태를 확정한 뒤 applyEstop이 유지한다.
  updateEstop(now);

  pollSerial(now);

  if (estop_active) {
    applyEstop();
  } else {
    updateWheel(now);
  }

  sendOutput(now);
}
