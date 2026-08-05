// ============================================================
//  인휠모터 좌우 독립 PID 테스트 (Arduino Mega 2560)
//  iw_0712_pid.ino / kasa_0713_A.ino 대비 변경점
//    - 기존: 21번 펄스 하나로만 PID → 좌우 동일 PWM 출력
//    - 변경: 21번(왼쪽 펄스) → PID → 8번(왼쪽 PWM)
//            20번(오른쪽 펄스) → PID → 9번(오른쪽 PWM)
//      좌우 모터컨트롤러 특성 차이를 보정하기 위해 목표펄스는 동일하게 주되
//      PID(오차/적분/미분/코스팅 상태)는 완전히 분리해서 따로 돈다.
//  FF 보간 테이블 + PID + 하강 코스트-캐치 로직은 iw_0712_pid.ino와 동일 (검증됨).
//  + PID 게인 kp/ki/kd 좌우 분리([0]=왼쪽, [1]=오른쪽) — 튜닝 별도 관리.
//  + 폭주 감지: 목표+2펄스 이상 과속이 1초 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
//  + 목표펄스는 0~15 정수만 유효 (음수/16 이상/소수/형식 오류는 무시).
//    왼쪽 컨트롤러가 PWM ~150 초과 지속 시 과속 모드로 폭주하는 특성이 있어(0716 실측),
//    그 영역이 필요한 16펄스 이상은 운용하지 않는다.
//  입력 : 정수(좌우 공통 목표펄스, 0~15) 한 줄
//  출력 : "<pwmL> <pwmR> <target> <speed21> <speed20> 0 25"  (20ms마다)
// ============================================================

// ===== ★ 피드포워드 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =====
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// 목표펄스 -> PWM 보간 (3점 라그랑주 2차보간)
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

// ===== ★ PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =====
float kp[2] = {0.4,  0.4};    // {왼쪽(21-8), 오른쪽(20-9)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ===== ★ 목표펄스 유효 범위 ★ =====
const int TARGET_MAX = 15;    // 0~15만 유효. 16↑는 좌측 과속 모드 영역이라 사용 안 함

// ===== ★ 코스트-캐치 (튜닝 지점) ★ =====
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ===== ★ PWM 상한 (튜닝 지점) ★ =====
const int PWM_MAX = 170;

// ===== ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =====
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영).
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ===== ★ 폭주 감지 (튜닝 지점) ★ =====
// 좌측 컨트롤러 과속 특성 대비 안전망: 목표보다 RUNAWAY_ERR_OVER 펄스 이상 과속이
// RUNAWAY_CONFIRM_CYCLES 주기(20ms) 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초

// ===== 좌/우 PID 상태 (완전 분리 — 적분값/이전오차/코스팅여부/이전PWM 각자 보관) =====
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

// ===== 핀 =====
const uint8_t PWM_PIN_L  = 8;   // 왼쪽 모터 PWM
const uint8_t PWM_PIN_R  = 9;   // 오른쪽 모터 PWM
const uint8_t HALL_PIN_L = 21;  // 왼쪽 펄스 (PID 피드백)
const uint8_t HALL_PIN_R = 20;  // 오른쪽 펄스 (PID 피드백)

// ===== 변수선언 =====
volatile long encCount21 = 0;   // 왼쪽 펄스
volatile long encCount20 = 0;   // 오른쪽 펄스
unsigned long lastTime = 0;
int target = 0;   // 좌우 공통 목표펄스 (0~TARGET_MAX)

// ===== 시리얼 입력 버퍼 =====
char rxBuf[16];
uint8_t rxLen = 0;

// 부호 없는 정수(숫자만)인지 검사 — 음수/소수/그 외 문자는 여기서 걸러짐
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}

// 목표 펄스값 수신 - 줄 단위. 0~TARGET_MAX 정수만 유효, 그 외는 무시.
void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      // 자릿수 2 제한: 0~15가 최대 2자리이므로, 긴 숫자열의 atoi 오버플로도 함께 차단
      if (rxLen > 0 && rxLen <= 2 && isValidNumber(rxBuf)) {
        int newTarget = atoi(rxBuf);
        if (newTarget <= TARGET_MAX) {
          // 목표 하강 → 좌우 모두 코스트 진입, 상승 → 좌우 모두 코스트 해제
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
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}

// ===== 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =====
// iw_0712_pid.ino의 PID 로직을 그대로 이식. idx(LEFT/RIGHT)로 자기 상태/게인 배열만 참조해
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

void enc21() {
  encCount21++;
}

void enc20() {
  encCount20++;
}

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), enc21, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), enc20, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);

  lastTime = millis();
}

void loop() {
  pollSerial();

  // ===== 제어주기 20ms =====
  if (millis() - lastTime >= 20) {
    lastTime = millis();

    noInterrupts();
    long speed21 = encCount21;
    long speed20 = encCount20;
    encCount21 = 0;
    encCount20 = 0;
    interrupts();

    int pwmL = updatePid(LEFT,  target, (int)speed21);   // 21번 펄스 -> 8번 PWM
    int pwmR = updatePid(RIGHT, target, (int)speed20);   // 20번 펄스 -> 9번 PWM

    analogWrite(PWM_PIN_L, pwmL);
    analogWrite(PWM_PIN_R, pwmR);

    Serial.print(pwmL);
    Serial.print(" ");
    Serial.print(pwmR);
    Serial.print(" ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print(speed21);
    Serial.print(" ");
    Serial.print(speed20);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(25);
  }
}
