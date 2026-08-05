// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID (순수 주행 전용, Arduino Mega 2560) - 0727 nodiff 버전
//
//  kasa_0727_A.ino에서 ★조향 연동 좌우 차동 로직을 통째로 제거★한 버전.
//  차동은 이제 ROS2(nxde 패키지 walker_k)가 담당한다:
//     B보드 텔레메트리 "P,<조향각>"  ->  walker_k가 좌/우 펄스 계산
//     ->  A보드로 "<좌펄스>,<우펄스>" 콤마 2값 전송  ->  A보드는 그대로 추종만
//  따라서 이 보드는 A1 가변저항 분기 입력이 필요 없다 (배선 제거 가능).
//
//  [제거된 것] DC_POT_PIN(A1) / potToAngle / readPotMedian / readSteerAngleFiltered /
//    ENABLE_STEER_DIFF / STEER_DIFF_SIGN / diffGainPulsePerDeg / STEER_DEADBAND_DEG /
//    STEER_ZERO_OFFSET_DEG / RAW_LEFT_LIMIT / RAW_RIGHT_LIMIT / effTargetL,R / steerAngleDeg
//    -> updateWheel()이 target[LEFT]/target[RIGHT]를 그대로 PID에 넘긴다.
//
//  [유지된 것] 좌/우 핀 배정, 좌우 독립 PID, FF보간, 코스트-캐치, 폭주감지, 직접 PWM 모드,
//    E-stop, 텔레메트리 형식 — 전부 kasa_0727_A.ino와 동일.
//
//  ★ 배선 (실측 확정) ★
//     왼쪽  모터 : 펄스 D3 -> PID -> PWM D9
//     오른쪽 모터 : 펄스 D2 -> PID -> PWM D8
//   좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//
//  입력 : "<값>" 또는 "<왼쪽값>,<오른쪽값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값: 펄스 전용(0~15), 범위 밖/숫자 아님은 무시 (좌우 동일값)
//         - 콤마 2값: 좌/우 독립 — 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지
//           ★ walker_k가 차동 주행 시 쓰는 경로가 이것. 0~15를 넘기면 직접 PWM으로
//             오해석되므로 walker_k 쪽(setting.PULSE_MAX)에서 반드시 클램프할 것.
//  출력 : "S,<왼쪽펄스>,<오른쪽펄스>" (평상시) / "STOP" (e-stop 중)
//  제어주기 : 20ms, 출력주기 : 50ms
//    - FF 보간 테이블(펄스->PWM, 라그랑주 2차보간) + PID(0.4/0.03/0.2, 좌우 게인 별도 관리)
//    - PWM 슬루레이트 제한(+4/cycle) + 조건부 적분(|오차|<4, 기여 ±40 클램프)
//    - 하강 코스트-캐치, 폭주 감지(목표+2펄스 과속 1초 연속 시 코스트)
//      ※ 위 안전장치들은 '펄스 모드'에만 적용. 직접 PWM 모드는 무보호(테스트 전용).
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) -> e-stop
//    - e-stop 발동 시 직접 PWM 모드도 즉시 해제(펄스 0으로 복귀)
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// ★ 실측 확정 배선 (0727) ★
//     왼쪽  모터 : 펄스 D3 -> PID -> PWM D9
//     오른쪽 모터 : 펄스 D2 -> PID -> PWM D8
//   즉 좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//   ※ 좌우가 뒤바뀐 것으로 보이면 이 4줄만 고치면 된다.
//     아래 파서/PID/출력은 전부 LEFT/RIGHT 인덱스로만 동작하므로 자동으로 따라간다.

// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 2;   // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 3;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (D3 펄스와 같은 컨트롤러, 0727 재확인)
const uint8_t PWM_PIN_R = 9;     // 오른쪽 모터 PWM (D2 펄스와 같은 컨트롤러, 0727 재확인)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// ※ 조향각 가변저항(A1) 분기 입력은 이 버전에서 제거됨.
//   좌우 차동은 ROS2(walker_k)가 B보드 텔레메트리 "P,<각도>"를 보고 계산해
//   "<좌펄스>,<우펄스>" 콤마 2값으로 내려보낸다. A보드는 그 값을 그대로 PID로 추종만 한다.


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(D3-D9), 오른쪽(D2-D8)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ================= ★ 값 해석 경계 ★ =================
const int TARGET_MAX = 15;      // 0~15 = 펄스 목표. 단일 값 입력은 이 범위만 유효
const int PWM_DIRECT_MAX = 255; // 16~255 = 직접 PWM (콤마 2값 형식에서만). 256 이상 = 정지

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
// ※ 펄스(PID) 모드에만 적용. 직접 PWM 모드(16~255)는 이 상한을 무시하고 그대로 출력.
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영). 직접 PWM 모드에는 미적용.
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
// 한쪽 컨트롤러의 과속 특성 대비 안전망: 목표보다 RUNAWAY_ERR_OVER 펄스 이상 과속이
// RUNAWAY_CONFIRM_CYCLES 주기(20ms) 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= 좌/우 PID 상태 (완전 분리) =================
// 주의: Arduino IDE는 함수 프로토타입을 파일 맨 위(커스텀 타입 정의보다 앞)에 자동 삽입한다.
// struct로 상태를 묶으면 그 프로토타입이 struct 정의보다 앞에 삽입되어 컴파일 에러가 남.
// 그래서 기본 타입(int/float/bool) 배열 + 좌(0)/우(1) 인덱스로 상태를 분리한다.
const uint8_t LEFT  = 0;   // 왼쪽  : 펄스 D3 피드백 -> PWM D9
const uint8_t RIGHT = 1;   // 오른쪽 : 펄스 D2 피드백 -> PWM D8

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터


// ================= 구동 모드 (좌/우 독립) =================
const uint8_t DRIVE_PULSE = 0;   // 펄스 목표 PID 주행 (0~15)
const uint8_t DRIVE_PWM   = 1;   // 직접 PWM 출력 (16~255, 무보호)

uint8_t driveMode[2] = {DRIVE_PULSE, DRIVE_PULSE};
int     pwmDirect[2] = {0, 0};   // 직접 PWM 모드에서 출력할 값


// ================= 인휠 상태 =================
volatile long encCountL = 0;    // 왼쪽 펄스 (D3)
volatile long encCountR = 0;    // 오른쪽 펄스 (D2)
int target[2] = {0, 0};         // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)
int wheelSpeedL = 0;
int wheelSpeedR = 0;
unsigned long wheel_t = 0;

// ================= E-stop 상태 =================
bool estop_active = false;

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


// ================= ISR (D3=왼쪽 펄스, D2=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


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


// ================= 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =================
// idx(LEFT/RIGHT)로 자기 상태/게인 배열만 참조해 완전히 독립적으로 동작.
int updatePid(uint8_t idx, int tgt, int speed) {
  int err = tgt - speed;
  int d = err - pidLastErr[idx];
  pidLastErr[idx] = err;

  float ff = interpFF((float)tgt);

  // 폭주 감지: 지속 과속이면 코스트 진입(PWM 0) — 순간 오버슈트는 CONFIRM 주기로 걸러냄
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

  // 코스트-캐치: 목표+마진까지 내려오면 PID 재개 (PWM은 FF값에서 시작)
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


// ================= E-stop 안전 동작 (e-stop 상태에서 매 루프 호출) =================
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
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 재개 시 0부터 다시 램프업)
    runawayCnt[s] = 0;
  }
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 입력 형식 검사 (부호 없는 정수만) =================
// 음수/소수/그 외 문자는 여기서 걸러짐
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

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  wheel_t = tele_t = now;
}


// ================= 값 파싱 =================
// 숫자 토큰 -> 값. 숫자가 아니면 -1(줄 전체 무시용).
// 4자리 이상은 어차피 255 초과 = 정지이므로 atoi 오버플로 없이 256으로 통일.
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}


// ================= 펄스 모드 목표 적용 =================
// 직접 PWM 모드에서 복귀할 때는 PID 상태를 리셋하고 코스트로 진입
// (과속 상태면 무동력 감속, 이미 느리면 다음 주기에 바로 캐치되어 PID 재개)
void setPulseTarget(uint8_t idx, int newTarget) {
  if (driveMode[idx] == DRIVE_PWM) {
    driveMode[idx] = DRIVE_PULSE;
    pwmDirect[idx] = 0;
    pidI[idx] = 0;
    pidLastErr[idx] = 0;
    pidLastPwm[idx] = 0;      // 슬루 기준점 리셋: 캐치 후 0부터 다시 램프업
    runawayCnt[idx] = 0;
    pidCoasting[idx] = true;
    target[idx] = newTarget;
    return;
  }
  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제 (좌우 개별)
  if (newTarget < target[idx]) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
  } else if (newTarget > target[idx]) {
    pidCoasting[idx] = false;
  }
  target[idx] = newTarget;
}


// ================= 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =================
// 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지(펄스 0)
void applySide(uint8_t idx, long v) {
  if (v <= TARGET_MAX) {
    setPulseTarget(idx, (int)v);
  } else if (v <= PWM_DIRECT_MAX) {
    driveMode[idx] = DRIVE_PWM;
    pwmDirect[idx] = (int)v;
  } else {
    setPulseTarget(idx, 0);
  }
}


// ================= 입력 파서 =================
// "<값>" 단일 = 펄스 전용(0~15, 좌우 공통), "<좌값>,<우값>" = 좌/우 독립(펄스/PWM/정지).
// 형식이 안 맞으면 무시.
void handleLine(char* line) {
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
    applySide(LEFT, vL);
    applySide(RIGHT, vR);
  } else {
    // 단일 값은 무조건 펄스 모드: 0~15만 받고 그 외는 무시
    // (직접 PWM은 콤마 2값 형식으로만 가능 — 일반 주행 경로에서의 오발동 방지)
    long v = parseValue(line);
    if (v < 0 || v > TARGET_MAX) return;

    if (estop_active) return;
    setPulseTarget(LEFT, (int)v);
    setPulseTarget(RIGHT, (int)v);
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
  wheelSpeedL = (int)cL;
  wheelSpeedR = (int)cR;

  // ※ 좌우 차동 계산은 이 보드에서 제거됨 (ROS2 walker_k가 담당).
  //   walker_k가 B보드 조향각을 보고 이미 좌/우로 나눈 값을 "<좌>,<우>"로 내려주므로,
  //   여기서는 target[LEFT]/target[RIGHT]를 그대로 추종하기만 하면 된다.

  // 직접 PWM 모드면 받은 값을 그대로 출력 (상한/슬루/PID 미적용),
  // 펄스 모드면 기존 PID. 펄스 계측은 두 모드 공통으로 계속된다.
  int pwmL = (driveMode[LEFT] == DRIVE_PWM)
               ? pwmDirect[LEFT]
               : updatePid(LEFT,  target[LEFT],  wheelSpeedL);   // 왼쪽 : 펄스 D3 -> PWM D9
  int pwmR = (driveMode[RIGHT] == DRIVE_PWM)
               ? pwmDirect[RIGHT]
               : updatePid(RIGHT, target[RIGHT], wheelSpeedR);   // 오른쪽 : 펄스 D2 -> PWM D8

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= 출력 =================
// [0727-4/5] 기존 2필드 규약 유지: "S,<왼쪽펄스>,<오른쪽펄스>"
//   입력 "<왼쪽>,<오른쪽>"과 순서가 동일하다 (왼쪽이 앞, 오른쪽이 뒤).
//   직접 PWM 모드에서도 펄스 계측/출력은 계속됨 (PWM-펄스 특성 계측용)
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("S,");
  Serial.print(wheelSpeedL);
  Serial.print(',');
  Serial.println(wheelSpeedR);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();

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
    applyEstop();
  } else {
    updateWheel(now);
  }

  sendOutput(now);
}
