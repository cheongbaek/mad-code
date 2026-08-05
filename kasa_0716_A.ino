// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID + 주행 펄스 계측 (Arduino Mega 2560) - 0716 버전
//  kasa_0713_A.ino의 입출력 규약 유지 + iw_0716_pid.ino의 좌우 독립 PID 이식
//    - 기존(0713): 21번 펄스 하나로 PID → 단일 PWM(10번) 출력
//    - 변경(0716): 21번(왼쪽 펄스) → PID → 8번(왼쪽 PWM)
//                  20번(오른쪽 펄스) → PID → 9번(오른쪽 PWM)
//      목표펄스는 좌우 동일하게 주되 PID(오차/적분/미분/코스팅)는 완전 분리 —
//      두 모터컨트롤러의 특성 차이 보정.
//    - FF 보간 테이블(펄스->PWM, 라그랑주 2차보간) + PID(0.4/0.03/0.2, 좌우 게인 별도 관리)
//    - PWM 슬루레이트 제한(+4/cycle) + 조건부 적분(|오차|<4, 기여 ±40 클램프)
//    - 하강 코스트-캐치: 목표 하강 시 PWM 0 + 적분 리셋, 목표+마진 도달 시 PID 재개
//    - 폭주 감지: 목표+2펄스 이상 과속 1초 연속 시 해당 바퀴만 PWM 0(코스트) → 캐치로 재개
//      (좌측 컨트롤러가 PWM ~150 초과 지속 시 과속 모드로 폭주하는 특성 대비 안전망, 0716 실측)
//    - 목표펄스는 0~15 정수만 유효 (음수/16 이상/소수/형식 오류 무시)
//      16펄스 이상은 좌측 과속 모드 영역(PWM 150↑)이라 운용하지 않음
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<주행목표펄스>"  (정수 1개, 0~15, 개행 종료)
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "S,<21번펄스>,<20번펄스>" (평상시) / "STOP" (e-stop 중)
//         - 21번(왼쪽)이 앞, 20번(오른쪽)이 뒤
//  제어주기 : 20ms, 출력주기 : 50ms
//
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN20 = 20;   // 오른쪽 모터컨트롤러 (오른쪽 PID 피드백)
const uint8_t HALL_PIN21 = 21;   // 왼쪽 모터컨트롤러 (왼쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (21번 펄스 피드백)
const uint8_t PWM_PIN_R = 9;     // 오른쪽 모터 PWM (20번 펄스 피드백)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(21-8), 오른쪽(20-9)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ================= ★ 목표펄스 유효 범위 ★ =================
const int TARGET_MAX = 15;    // 0~15만 유효. 16↑는 좌측 과속 모드 영역이라 사용 안 함

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영).
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
// 좌측 컨트롤러 과속 특성 대비 안전망: 목표보다 RUNAWAY_ERR_OVER 펄스 이상 과속이
// RUNAWAY_CONFIRM_CYCLES 주기(20ms) 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= 좌/우 PID 상태 (완전 분리) =================
// 주의: Arduino IDE는 함수 프로토타입을 파일 맨 위(커스텀 타입 정의보다 앞)에 자동 삽입한다.
// struct로 상태를 묶으면 그 프로토타입이 struct 정의보다 앞에 삽입되어 컴파일 에러가 남.
// 그래서 기본 타입(int/float/bool) 배열 + 좌(0)/우(1) 인덱스로 상태를 분리한다.
const uint8_t LEFT  = 0;   // 21번 펄스 피드백 -> 8번 PWM (왼쪽)
const uint8_t RIGHT = 1;   // 20번 펄스 피드백 -> 9번 PWM (오른쪽)

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터


// ================= 인휠 상태 =================
volatile long encCount20 = 0;   // 오른쪽 펄스
volatile long encCount21 = 0;   // 왼쪽 펄스
int target = 0;                 // 좌우 공통 목표펄스 (0~TARGET_MAX)
int wheel_speed20 = 0;
int wheel_speed21 = 0;
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


// ================= 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =================
// iw_0716_pid.ino와 동일. idx(LEFT/RIGHT)로 자기 상태/게인 배열만 참조해
// 완전히 독립적으로 동작 — 공유하는 것은 target뿐.
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
  target = 0;

  // 양쪽 PID 상태 초기화 (해제 후 재개 시 적분 잔재/펄스 누적 방지)
  for (uint8_t s = 0; s < 2; s++) {
    pidI[s] = 0;
    pidLastErr[s] = 0;
    pidCoasting[s] = false;
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 재개 시 0부터 다시 램프업)
    runawayCnt[s] = 0;
  }
  noInterrupts();
  encCount20 = 0;
  encCount21 = 0;
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

  pinMode(HALL_PIN20, INPUT_PULLUP);
  pinMode(HALL_PIN21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN20), encISR20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN21), encISR21, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  wheel_t = tele_t = now;
}


// ================= 입력 파서 =================
// "<주행목표펄스>" 정수 1개 (0~TARGET_MAX). 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, " ");
  char* tok2 = tok1 ? strtok(NULL, " ") : NULL;   // 토큰이 2개 이상이면 형식 오류

  if (!tok1 || tok2 || !isValidNumber(tok1)) {
    return;
  }
  // 자릿수 2 제한: 0~15가 최대 2자리이므로, 긴 숫자열의 atoi 오버플로도 함께 차단
  if (strlen(tok1) > 2) return;

  // e-stop 중에는 구동 명령 미적용
  if (estop_active) return;

  int newTarget = atoi(tok1);
  if (newTarget > TARGET_MAX) return;

  // 목표 하강 → 좌우 모두 코스트 진입(무동력 감속), 상승 → 좌우 모두 코스트 해제
  if (newTarget < target) {
    pidCoasting[LEFT] = true;
    pidCoasting[RIGHT] = true;
    pidI[LEFT] = 0;
    pidI[RIGHT] = 0;
  } else if (newTarget > target) {
    pidCoasting[LEFT] = false;
    pidCoasting[RIGHT] = false;
  }
  target = newTarget;
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


// ================= 인휠 제어 (좌우 독립 PID) =================
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

  int pwmL = updatePid(LEFT,  target, wheel_speed21);   // 21번 펄스 -> 8번 PWM
  int pwmR = updatePid(RIGHT, target, wheel_speed20);   // 20번 펄스 -> 9번 PWM

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= 출력 =================
// "S,<21번펄스>,<20번펄스>" — 21번(왼쪽)이 앞
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
