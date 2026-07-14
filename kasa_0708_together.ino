// ============================================================
//  통합 차량 제어 펌웨어 (Arduino Mega 2560)
//  입력 : "<주행목표값> <조향각도> <브레이크강도>"  (스페이스 구분, 개행 종료)
//         예) 32 -20 5
//  출력 : "<2번펄스> <3번펄스>"
//         2번 = 왼쪽 모터 (PID 피드백에 사용), 3번 = 오른쪽 모터 (모니터링)
//
//  인휠 PID : iw_0708_pidtuning.ino 에서 검증된 구조 반영
//    - 왼쪽(2번 핀) 펄스로 루프를 닫음 (공통 PWM 분배, 폭주 억제 확인)
//    - 피드포워드(FF) + PID 보정, 포화 방향 적분 중단(anti-windup)
//    - 목표값 변경 시 적분 리셋, 측정값 기준 미분
//  제어주기 : 인휠/조향/브레이크 모두 20ms 공유 (CONTROL_WINDOW_MS)
//             게인/FF는 20ms 창 기준 실측 튜닝값
//  차후 계획 : 좌/우 모터 PWM 특성이 달라 배선 분리 후 모터별 개별 PID 예정
//
//  E-stop 조건 (아래 중 하나라도 충족 시 발동)
//    ① 40번 핀 단락 (LOW)
//    ② 입력 형식 오류 (숫자 3개 형식이 아님)
//    ③ 5초 이상 입력 없음
//
//  E-stop 동작
//    - 조향(DC)모터 PWM = 0, 인휠모터 PWM = 0
//    - 리니어(브레이크) 모터 최고출력 (미장착 상태이므로 자리만)
//    - 출력 펄스값을 999 999 로 강제 표시
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (2, 3번 인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN2 = 2;    // 왼쪽 모터 (PID 피드백에 사용)
const uint8_t HALL_PIN3 = 3;    // 오른쪽 모터 (모니터링 전용)

// --- 인휠 주행 PWM ---
const uint8_t INWHEEL_PWM_PIN = 10;

// --- DC 조향모터 ---
const uint8_t DC_DIR_PIN = 22;
const uint8_t DC_PWM_PIN = 6;
const uint8_t DC_POT_PIN = A1;         // 조향 위치 가변저항

// --- 리니어(브레이크) : 자리만 (장비 준비되면 주석 해제) ---
const uint8_t LINEAR_PWM_PIN = 7;      // 미사용 (자리만)

// --- E-stop ---
const uint8_t ESTOP_PIN = 40;          // INPUT_PULLUP, 단락(LOW) 시 e-stop


// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long INPUT_TIMEOUT_MS = 5000;   // 5초 이상 무입력 → e-stop


// ================= 공통 제어주기 =================
// 인휠/조향/브레이크 제어 루프가 모두 공유하는 주기 (인휠 게인/FF가 20ms 창 기준 튜닝값)
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= 인휠 PID (iw_0708_pidtuning.ino 실측 튜닝값) =================
float kp = 20.0;
float ki = 2.0;
float kd = 2.0;

const float I_MAX = 100.0;     // 적분 클램프 값

// 피드포워드 (실측 정적 맵: 데드존 ≈60 PWM, 25펄스 ≈ 220 PWM)
// ff = FF_DEADZONE + FF_GAIN * wheelREF  (wheelREF=0이면 0)
const float FF_DEADZONE = 60.0;
const float FF_GAIN     = 6.4;

// 인휠 루프 주기는 공통 제어주기(CONTROL_WINDOW_MS, 20ms) 사용 (주기 변경 시 게인/FF 재튜닝 필요)


// ================= 조향 PD =================
#define KP_S   6.0f
#define KD_S   0.1f
const int  STEER_TOLERANCE = 5;
const int  STEER_MIN_PWM   = 30;
const int  STEER_MAX_PWM   = 255;
// 조향 루프 주기는 공통 제어주기(CONTROL_WINDOW_MS, 20ms) 사용

#define DIR_CW   HIGH
#define DIR_CCW  LOW

// --- 조향각도(도) ↔ 포텐셔미터(raw) 변환 ---
// 0701_dc_potential.ino 로 측정한 좌/우 끝값 (example_control.py DC 보정값 기준, 재측정 시 갱신)
const int STEER_POT_LEFT   = 944;    // angle = -STEER_ANGLE_MAX 에 대응하는 raw값
const int STEER_POT_RIGHT  = 645;    // angle = +STEER_ANGLE_MAX 에 대응하는 raw값
const int STEER_ANGLE_MAX  =  30;    // -30 ~ +30도
const int STEER_ANGLE_MIN  = -STEER_ANGLE_MAX;


// --- 브레이크(리니어) ---
const int BRAKE_MAX = 255;   // 입력 브레이크강도의 상한 (0~255)


// ================= 인휠 상태 =================
volatile long encCount2 = 0;   // 홀센서 2번 펄스 카운트 (왼쪽, PID 피드백)
volatile long encCount3 = 0;   // 홀센서 3번 펄스 카운트 (오른쪽, 모니터링)
long   wheelREF   = 0;         // 목표 속도 (펄스/주기)
float  i_term      = 0;
int    lastSpeed2   = 0;       // 측정값 기준 미분용 (2번 핀 직전 속도)
int    wheel_pwm    = 0;
int    wheel_speed2 = 0;       // 2번 핀 측정 펄스 (주기당, PID 사용)
int    wheel_speed3 = 0;       // 3번 핀 측정 펄스 (주기당, 모니터링)
unsigned long wheel_t = 0;


// ================= 조향 상태 =================
int  steer_angle_cmd  = 0;
int  steer_target_pos = 512;
int  steer_prev_pos   = 0;
unsigned long steer_t = 0;


// ================= 브레이크(리니어) 상태 : 자리만 =================
int  brake_cmd    = 0;   // 입력받은 브레이크강도
int  brake_output = 0;   // 실제(예정) 출력값
unsigned long brake_t = 0;


// ================= E-stop 상태 =================
bool estop_active = false;
bool badFormat    = false;
unsigned long lastInputTime = 0;


// ================= 출력용 =================
int pulse2_out = 0;
int pulse3_out = 0;
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void encISR2();
void encISR3();
void inwheelWrite(int pwm);
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearWrite(int pwm);
void allMotorsStop();
void applyEstop();
int  angleToPot(int angle);
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateWheel(unsigned long now);
void updateSteer(unsigned long now);
void updateBrake(unsigned long now);
void sendOutput(unsigned long now);


// ================= ISR (홀센서 2, 3번) =================
void encISR2() { encCount2++; }
void encISR3() { encCount3++; }


// ================= 모터 출력 =================
void inwheelWrite(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(INWHEEL_PWM_PIN, pwm);
  wheel_pwm = pwm;
}
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

// 리니어(브레이크) : 장비 준비되면 analogWrite 주석 해제
void linearWrite(int pwm) {
  pwm = constrain(pwm, 0, 255);
  brake_output = pwm;
  // analogWrite(LINEAR_PWM_PIN, pwm);   // TODO: 리니어 액추에이터 연결 후 활성화
}

void allMotorsStop() { inwheelWrite(0); dcStop(); }

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop() {
  allMotorsStop();
  linearWrite(BRAKE_MAX);   // 리니어(브레이크) 최고출력

  // 인휠 PID 상태 초기화 (해제 후 재개 시 적분 잔재/펄스 누적 방지)
  i_term = 0;
  lastSpeed2 = 0;
  noInterrupts();
  encCount2 = 0;
  encCount3 = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 조향각도 → 포텐셔미터 raw값 =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, STEER_POT_LEFT, STEER_POT_RIGHT);
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


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  // 홀센서 (2번 = 왼쪽 PID 피드백, 3번 = 오른쪽 모니터링)
  pinMode(HALL_PIN2, INPUT_PULLUP);
  pinMode(HALL_PIN3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN2), encISR2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN3), encISR3, CHANGE);

  // 인휠
  pinMode(INWHEEL_PWM_PIN, OUTPUT);
  inwheelWrite(0);

  // 조향
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  // 리니어(브레이크) : 자리만
  // pinMode(LINEAR_PWM_PIN, OUTPUT);

  // E-stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  steer_target_pos = analogRead(DC_POT_PIN);   // 시작 시 현재 위치 유지
  steer_prev_pos   = steer_target_pos;

  unsigned long now = millis();
  wheel_t = steer_t = brake_t = tele_t = now;
  lastInputTime = now;   // 부팅 직후 5초의 유예시간 부여
}


// ================= 입력 파서 =================
// "<target> <angle> <brake>" 형식(공백 구분, 정수 3개)이 아니면 형식 오류로 처리
void handleLine(char* line) {
  char* tok1 = strtok(line, " ");
  char* tok2 = tok1 ? strtok(NULL, " ") : NULL;
  char* tok3 = tok2 ? strtok(NULL, " ") : NULL;
  char* tok4 = tok3 ? strtok(NULL, " ") : NULL;   // 토큰이 더 있으면 형식 오류

  if (!tok1 || !tok2 || !tok3 || tok4 ||
      !isValidNumber(tok1) || !isValidNumber(tok2) || !isValidNumber(tok3)) {
    badFormat = true;
    return;
  }

  badFormat = false;
  lastInputTime = millis();

  long driveTarget = atol(tok1);
  int  angle        = atoi(tok2);
  int  brake         = atoi(tok3);

  long newREF = max(0L, driveTarget);
  if (newREF != wheelREF) {
    i_term = 0;   // 목표값 변경 시 적분 리셋 (이전 목표의 적분 잔재 제거)
  }
  wheelREF        = newREF;
  steer_angle_cmd = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  steer_target_pos = angleToPot(steer_angle_cmd);
  brake_cmd        = constrain(brake, 0, BRAKE_MAX);
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
      badFormat = true;   // 버퍼 초과 = 비정상 입력
    }
  }
}


// ================= 인휠 제어 (FF + PID, iw_0708_pidtuning.ino 검증 구조) =================
void updateWheel(unsigned long now) {
  if (now - wheel_t < CONTROL_WINDOW_MS) return;
  wheel_t += CONTROL_WINDOW_MS;

  // 홀센서 2번(왼쪽, PID) / 3번(오른쪽, 모니터링) 펄스로 속도 측정
  noInterrupts();
  long c2 = encCount2;
  long c3 = encCount3;
  encCount2 = 0;
  encCount3 = 0;
  interrupts();
  wheel_speed2 = (int)c2;
  wheel_speed3 = (int)c3;

  int err = (int)wheelREF - wheel_speed2;

  // 측정값 기준 미분 (derivative on measurement, 목표 변경 시 미분 킥 없음)
  int d = -(wheel_speed2 - lastSpeed2);
  lastSpeed2 = wheel_speed2;

  // 피드포워드: 목표값에 대응하는 기본 PWM (PID는 보정만 담당)
  float ff = (wheelREF > 0) ? (FF_DEADZONE + FF_GAIN * wheelREF) : 0.0;

  // 현재 적분값으로 출력 후보 계산 (포화 판정용)
  float pwm_raw = ff + kp * err + ki * i_term + kd * d;

  // anti-windup: 출력이 포화된 방향으로는 적분 누적 중단
  bool sat_low  = (pwm_raw <= 0);
  bool sat_high = (pwm_raw >= 255);
  if (!(sat_low && err < 0) && !(sat_high && err > 0)) {
    i_term += err;
    if (i_term > I_MAX) i_term = I_MAX;
    if (i_term < -I_MAX) i_term = -I_MAX;
  }

  // 누적 반영하여 최종 출력 계산
  int pwm = ff + kp * err + ki * i_term + kd * d;
  if (pwm > 255) pwm = 255;
  if (pwm < 0) pwm = 0;

  if (wheelREF == 0) {
    pwm = 0;
    i_term = 0;
  }

  inwheelWrite(pwm);
}


// ================= 조향 제어 (목표 위치 PD) =================
void updateSteer(unsigned long now) {
  if (now - steer_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_t = now;

  int cur = analogRead(DC_POT_PIN);

  int err = steer_target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - steer_prev_pos) / dt);
  float output = p + d;
  steer_prev_pos = cur;

  if (abs(err) > STEER_TOLERANCE) {
    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    if (output > 0) dcCW(spd); else dcCCW(spd);
  } else {
    dcStop();
  }
}


// ================= 브레이크(리니어) : 자리만 =================
void updateBrake(unsigned long now) {
  if (now - brake_t < CONTROL_WINDOW_MS) return;   // 공통 제어주기 20ms
  brake_t = now;
  linearWrite(brake_cmd);   // 장비 연결 전까지는 brake_output 값만 갱신됨
}


// ================= 출력 =================
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    pulse2_out = 999;
    pulse3_out = 999;
  } else {
    pulse2_out = wheel_speed2;  // 왼쪽 (PID 피드백)
    pulse3_out = wheel_speed3;  // 오른쪽 (모니터링)
  }

  Serial.print(pulse2_out);
  Serial.print(' ');
  Serial.println(pulse3_out);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();

  bool pinEstop     = (digitalRead(ESTOP_PIN) == LOW);
  bool timeoutEstop = (now - lastInputTime > INPUT_TIMEOUT_MS);
  estop_active = pinEstop || timeoutEstop || badFormat;

  if (estop_active) {
    applyEstop();
  } else {
    updateWheel(now);
    updateSteer(now);
    updateBrake(now);
  }

  sendOutput(now);
}
