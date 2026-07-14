// ============================================================
//  A보드 : 인휠모터 PID + 주행 펄스 계측 (Arduino Mega 2560) - 0713 버전
//  kasa_0712_none.ino에서 조향/브레이크 분리(→ B보드), 입출력 규약 변경
//    - FF 보간 테이블(펄스->PWM, 실측 재보정, 라그랑주 2차보간) + PID(0.4/0.03/0.2, iw_0712_pid.ino 튜닝 이식)
//    - PWM 슬루레이트 제한(+4/cycle) + 조건부 적분(|오차|<4)로 오버슈트 억제, 적분 기여 ±40 클램프
//    - 하강 코스트-캐치: 목표 하강 시 PWM 0 + 적분 리셋, 목표+마진 도달 시 PID 재개
//    - 인휠 PWM 상한 170
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<주행목표펄스>"  (정수 1개, 개행 종료)
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "S,<21번펄스>,<20번펄스>" (평상시) / "STOP" (e-stop 중)
//         - 21번(PID 피드백)이 앞, 20번(모니터링)이 뒤
//  제어주기 : 20ms
//
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 인휠 PWM 0, PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN20 = 20;   // 모니터링 전용
const uint8_t HALL_PIN21 = 21;   // 왼쪽 모터컨트롤러 (PID 피드백에 사용)

// --- 인휠 주행 PWM ---
const uint8_t INWHEEL_PWM_PIN = 10;

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점) ★ =================
float kp = 0.4;
float ki = 0.03;
float kd = 0.2;

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영).
const int PWM_SLEW_MAX = 4;
int lastPwm = 0;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;


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


// ================= 함수 선언 =================
void encISR20();
void encISR21();
float interpFF(float x);
void inwheelWrite(int pwm);
void applyEstop();
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateWheel(unsigned long now);
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

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop() {
  inwheelWrite(0);
  wheelREF = 0;

  // 인휠 PID 상태 초기화 (해제 후 재개 시 적분 잔재/펄스 누적 방지)
  i_term   = 0;
  lastErr  = 0;
  coasting = false;
  noInterrupts();
  encCount20 = 0;
  encCount21 = 0;
  interrupts();
  wheel_t = millis();
  lastPwm = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 재개 시 0부터 다시 램프업)
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

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  wheel_t = tele_t = now;
}


// ================= 입력 파서 =================
// "<주행목표펄스>" 정수 1개. 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, " ");
  char* tok2 = tok1 ? strtok(NULL, " ") : NULL;   // 토큰이 2개 이상이면 형식 오류

  if (!tok1 || tok2 || !isValidNumber(tok1)) {
    return;
  }

  // e-stop 중에는 구동 명령 미적용
  if (estop_active) return;

  long newREF = max(0L, atol(tok1));
  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제
  if (newREF < wheelREF) {
    coasting = true;
    i_term = 0;
  } else if (newREF > wheelREF) {
    coasting = false;
  }
  wheelREF = newREF;
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


// ================= 인휠 제어 (FF 보간 + PID + 코스트-캐치) =================
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
    float iTerm = ki * i_term;
    pwm = ff + kp * err + iTerm + kd * d;

    // anti-windup: 출력 포화 시, 그리고 오차가 클 때(가속 중)는 적분 누적 중단
    if (pwm > 0 && pwm < PWM_MAX && abs(err) < I_ACCUM_ERR_MAX) {
      i_term += err;
    }

    // 적분 기여분을 ki 값과 무관하게 ±40 pwm로 고정 제한
    iTerm = constrain(ki * i_term, -40, 40);
    pwm = ff + kp * err + iTerm + kd * d;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < 0) pwm = 0;
  }

  pwm = constrain(pwm, 0, PWM_MAX);

  // 슬루레이트 제한: pwm 급상승만 제한(관성 오버슈트 방지), 하강은 즉시 반영
  if (pwm > lastPwm + PWM_SLEW_MAX) pwm = lastPwm + PWM_SLEW_MAX;
  lastPwm = pwm;

  inwheelWrite(pwm);
}


// ================= 출력 =================
// "S,<21번펄스>,<20번펄스>" — 21번(PID 피드백)이 앞
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("S,");
  Serial.print(wheel_speed21);
  Serial.print(',');
  Serial.println(wheel_speed20);
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
