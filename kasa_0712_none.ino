// ============================================================
//  통합 차량 제어 펌웨어 (Arduino Mega 2560) - 0712 버전
//  kasa_0709_none.ino 기반 + 인휠모터 PID 로직을 iw_0712_pid.ino로 완전 대체
//    - FF 보간 테이블(펄스->PWM, 라그랑주 2차보간) + PID(1.0/0.1/0.2)
//    - 하강 코스트-캐치: 목표 하강 시 PWM 0 + 적분 리셋, 목표+마진 도달 시 PID 재개
//    - 인휠 PWM 상한 155
//  E-stop 스위치: 22번 핀, NC(Normally Closed) 방식
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<주행목표펄스> <조향각도> <브레이크출력> <모드>"  (스페이스 구분, 개행 종료)
//         - 3번째 값(브레이크출력)은 -255~255 부호있는 값
//         - 4번째 값(모드)이 1이면 e-stop 발동 (없으면 0, 평상시)
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "<20번펄스> <21번펄스> <조향각도>" (평상시) / "STOP" (e-stop 중)
//
//  조향(DC)모터 : 열린루프, PWM = |각도| * 4, 음수=왼쪽/양수=오른쪽/0=정지
//  리니어(브레이크)모터 : 부호/크기로 열린루프 타이밍 구동 (2초)
//  제어주기 : 20ms 공유
//
//  E-stop 조건 (매 루프 재평가) : ① 22번 핀 개방(HIGH) ② 모드값 1 ③ 5초 무입력
//  E-stop 동작 : 인휠/조향 PWM 0, 리니어 최고출력 체결 방향 2초간 구동 후 정지, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN20 = 20;   // 모니터링 전용
const uint8_t HALL_PIN21 = 21;   // 왼쪽 모터컨트롤러 (PID 피드백에 사용)

// --- 인휠 주행 PWM ---
const uint8_t INWHEEL_PWM_PIN = 10;

// --- DC 조향모터 (MD20A, 가변저항 미사용) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀/모드/타임아웃 e-stop 전부 비활성


// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long INPUT_TIMEOUT_MS = 5000;   // 5초 이상 무입력 → e-stop


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1,  2,  3,  4,  6,  7,  8, 11, 13, 18, 23, 25};
const float ffPwmTable[FF_TABLE_N]   = {60, 70, 80, 90,100,110,120,130,140,150,160,170};

// ================= ★ 인휠 PID 게인 (튜닝 지점) ★ =================
float kp = 1.0;
float ki = 0.1;
float kd = 0.2;

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
const int PWM_MAX = 155;


// ================= 조향 (열린루프, 각도 → PWM 직결) =================
const int STEER_PWM_PER_DEG = 4;     // PWM = |각도| * 4
const int STEER_ANGLE_MAX   =  40;   // 입력 각도 클램프 범위
const int STEER_ANGLE_MIN   = -STEER_ANGLE_MAX;

#define DIR_CW   HIGH   // 왼쪽
#define DIR_CCW  LOW    // 오른쪽


// ================= 브레이크(리니어) 열린루프 타이밍 구동 =================
const int BRAKE_MAX = 255;                  // 브레이크출력 크기 상한
const unsigned long LINEAR_RUN_MS = 2000;   // 입력받은 방향/크기로 구동하는 시간

// e-stop 시 리니어를 강제로 체결시키는 방향
const uint8_t LINEAR_ENGAGE_DIR = DIR_CW;


// ================= 인휠 상태 =================
volatile long encCount20 = 0;
volatile long encCount21 = 0;
long   wheelREF     = 0;        // 목표 속도 (펄스/주기)
float  i_term       = 0;
int    lastErr      = 0;
bool   coasting     = false;    // 목표 하강 전이 중(무동력 감속) 여부
int    wheel_pwm    = 0;
int    wheel_speed20 = 0;
int    wheel_speed21 = 0;
unsigned long wheel_t = 0;


// ================= 조향 상태 =================
int  steer_angle_cmd = 0;


// ================= 브레이크(리니어) 상태 =================
int  brake_cmd     = 0;
int  brake_output  = 0;
bool linear_running = false;
unsigned long linear_start_t = 0;


// ================= 모드 (4번째 입력값) =================
int mode_cmd = 0;   // 1 = e-stop 발동


// ================= E-stop 상태 =================
bool estop_active = false;
unsigned long lastInputTime = 0;

// e-stop 시 리니어 체결 구동 시간 (이 시간 후 리니어 출력 정지)
const unsigned long ESTOP_BRAKE_MS = 2000;
bool estop_latched = false;          // e-stop 진입(엣지) 감지용
unsigned long estop_engage_t = 0;    // 체결 시작 시각

// e-stop 핀 판정: 이 시간 동안 '전부' 개방(HIGH)이어야 발동
// (loop 매회 폴링, 중간에 한 번이라도 단락(LOW)이 읽히면 타이머 리셋)
const unsigned long ESTOP_PIN_CONFIRM_MS = 500;
unsigned long estop_pin_high_t = 0;  // HIGH가 처음 관측된 시각 (LOW로 복귀하면 0)


// ================= 출력용 =================
int pulse20_out = 0;
int pulse21_out = 0;
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void encISR20();
void encISR21();
float interpFF(float x);
void inwheelWrite(int pwm);
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearCW(int p); void linearCCW(int p);
void allMotorsStop();
void applyEstop(unsigned long now);
void applySteer(int angle);
void startLinear(int cmd);
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateWheel(unsigned long now);
void updateBrake(unsigned long now);
void sendOutput(unsigned long now);


// ================= ISR (홀센서 20, 21번) =================
void encISR20() { encCount20++; }
void encISR21() { encCount21++; }


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


// ================= 모터 출력 =================
void inwheelWrite(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(INWHEEL_PWM_PIN, pwm);
  wheel_pwm = pwm;
}
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }

void allMotorsStop() { inwheelWrite(0); dcStop(); }

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop(unsigned long now) {
  allMotorsStop();

  wheelREF        = 0;
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

  // 인휠 PID 상태 초기화 (해제 후 재개 시 적분 잔재/펄스 누적 방지)
  i_term   = 0;
  lastErr  = 0;
  coasting = false;
  noInterrupts();
  encCount20 = 0;
  encCount21 = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 조향각도 → 열린루프 PWM 구동 =================
void applySteer(int angle) {
  int pwm = abs(angle) * STEER_PWM_PER_DEG;
  if (angle < 0)      dcCW(pwm);    // 왼쪽
  else if (angle > 0) dcCCW(pwm);   // 오른쪽
  else                dcStop();
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


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(HALL_PIN20, INPUT_PULLUP);
  pinMode(HALL_PIN21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN20), encISR20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN21), encISR21, CHANGE);

  pinMode(INWHEEL_PWM_PIN, OUTPUT);
  inwheelWrite(0);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, DIR_CW);
  linearStop();

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  wheel_t = tele_t = now;
  lastInputTime = now;   // 부팅 직후 5초의 유예시간 부여
}


// ================= 입력 파서 =================
// "<target> <angle> <brake> <mode>" 형식(공백 구분). 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, " ");
  char* tok2 = tok1 ? strtok(NULL, " ") : NULL;
  char* tok3 = tok2 ? strtok(NULL, " ") : NULL;
  char* tok4 = tok3 ? strtok(NULL, " ") : NULL;
  char* tok5 = tok4 ? strtok(NULL, " ") : NULL;   // 토큰이 5개 이상이면 형식 오류

  if (!tok1 || !tok2 || !tok3 || tok5 ||
      !isValidNumber(tok1) || !isValidNumber(tok2) || !isValidNumber(tok3)) {
    return;
  }

  lastInputTime = millis();

  long driveTarget = atol(tok1);
  int  angle        = atoi(tok2);
  int  brake         = atoi(tok3);

  mode_cmd = (tok4 && isValidNumber(tok4)) ? atoi(tok4) : 0;

  // e-stop 중에는 구동 명령(주행/조향/브레이크) 미적용 → 리니어 재구동 방지
  //  - 수신 자체는 유효 처리(lastInputTime/mode_cmd 갱신)하므로 해제 판정은 정상 동작
  if (estop_active) return;

  long newREF = max(0L, driveTarget);
  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제
  if (newREF < wheelREF) {
    coasting = true;
    i_term = 0;
  } else if (newREF > wheelREF) {
    coasting = false;
  }
  wheelREF        = newREF;
  steer_angle_cmd = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  applySteer(steer_angle_cmd);
  brake_cmd       = constrain(brake, -BRAKE_MAX, BRAKE_MAX);
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


// ================= 인휠 제어 (iw_0712_pid.ino: FF 보간 + PID + 코스트-캐치) =================
void updateWheel(unsigned long now) {
  if (now - wheel_t < CONTROL_WINDOW_MS) return;
  wheel_t += CONTROL_WINDOW_MS;

  noInterrupts();
  long c20 = encCount20;
  long c21 = encCount21;
  encCount20 = 0;
  encCount21 = 0;
  interrupts();
  wheel_speed20 = (int)c20;
  wheel_speed21 = (int)c21;

  int err = (int)wheelREF - wheel_speed21;
  int d = err - lastErr;
  lastErr = err;

  float ff = interpFF((float)wheelREF);

  // 코스트-캐치: 목표+마진까지 내려오면 PID 재개 (PWM은 FF값에서 시작)
  if (coasting && wheel_speed21 <= (int)wheelREF + CATCH_MARGIN) {
    coasting = false;
  }

  int pwm;
  if (coasting) {
    pwm = 0;
    i_term = 0;
  } else {
    pwm = ff + kp * err + ki * i_term + kd * d;

    // anti-windup: 출력 포화 시 적분 누적 중단
    if (pwm > 0 && pwm < PWM_MAX) {
      i_term += err;
    }

    pwm = ff + kp * err + ki * i_term + kd * d;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < 0) pwm = 0;
  }

  pwm = constrain(pwm, 0, PWM_MAX);

  inwheelWrite(pwm);
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
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  pulse20_out = wheel_speed20;
  pulse21_out = wheel_speed21;
  int angle_out = steer_angle_cmd;

  Serial.print(pulse20_out);
  Serial.print(' ');
  Serial.print(pulse21_out);
  Serial.print(' ');
  Serial.println(angle_out);
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
  bool modeEstop    = (mode_cmd == 1);
  bool timeoutEstop = (now - lastInputTime > INPUT_TIMEOUT_MS);
  estop_active = ESTOP_ENABLED && (pinEstop || modeEstop || timeoutEstop);

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      // e-stop 해제 엣지: 체결 구동 중이던 리니어를 반드시 정지
      // (2초 내 해제 시 updateBrake가 못 꺼주는 상태로 남는 것 방지)
      linearStop();
      brake_output = 0;
      estop_latched = false;
    }
    updateWheel(now);
    updateBrake(now);
  }

  sendOutput(now);
}
