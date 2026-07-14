// ============================================================
//  인휠모터 PID 펌웨어 (Arduino Mega 2560)
//  FF 보간 테이블 + PID + 하강 코스트-캐치
//  입력 : 정수(목표펄스) 한 줄
//  출력 : "<pwm> <target> <speed21> <speed20> 0 25"  (20ms마다)
// ============================================================

// ===== ★ 피드포워드 테이블 (펄스 -> PWM, 실측으로 조절) ★ =====
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

// ===== 변수선언 =====
volatile long encCount20 = 0;   // 모니터링 전용
volatile long encCount21 = 0;   // 왼쪽 모터컨트롤러 (PID 피드백)
unsigned long lastTime = 0;
int target = 0;
float i = 0;
int lastErr = 0;

// ===== ★ PID 게인 (튜닝 지점) ★ =====
float kp = 0.4;
float ki = 0.03;
float kd = 0.2;

// ===== ★ 코스트-캐치 (튜닝 지점) ★ =====
bool coasting = false;
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ===== ★ PWM 상한 (튜닝 지점) ★ =====
const int PWM_MAX = 170;

// ===== ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =====
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영).
const int PWM_SLEW_MAX = 4;
int lastPwm = 0;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ===== 시리얼 입력 버퍼 =====
char rxBuf[16];
uint8_t rxLen = 0;

bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t k = 0;
  if (s[0] == '-' || s[0] == '+') k = 1;
  if (s[k] == '\0') return false;
  for (; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}

// 목표 펄스값 수신 - 줄 단위, 숫자 아니면 무시
void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0 && isValidNumber(rxBuf)) {
        int newTarget = atoi(rxBuf);
        // 목표 하강 → 코스트 진입, 상승 → 코스트 해제
        if (abs(newTarget) < abs(target)) {
          coasting = true;
          i = 0;
        } else if (abs(newTarget) > abs(target)) {
          coasting = false;
        }
        target = newTarget;
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), enc20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), enc21, CHANGE);

  pinMode(10, OUTPUT);

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

    int err = abs(target) - (int)speed21;
    int d = err - lastErr;
    lastErr = err;

    float ff = interpFF((float)abs(target));

    // 코스트-캐치: 목표+마진까지 내려오면 PID 재개 (PWM은 FF값에서 시작)
    if (coasting && speed21 <= abs(target) + CATCH_MARGIN) {
      coasting = false;
    }

    int pwm;
    if (coasting) {
      pwm = 0;
      i = 0;
    } else {
      float iTerm = ki * i;
      pwm = ff + kp * err + iTerm + kd * d;

      // anti-windup: 출력 포화 시, 그리고 오차가 클 때(가속 중)는 적분 누적 중단
      if (pwm > 0 && pwm < PWM_MAX && abs(err) < I_ACCUM_ERR_MAX) {
        i += err;
      }

      // 적분 기여분을 ki 값과 무관하게 ±40 pwm로 고정 제한
      iTerm = constrain(ki * i, -40, 40);
      pwm = ff + kp * err + iTerm + kd * d;
      if (pwm > PWM_MAX) pwm = PWM_MAX;
      if (pwm < 0) pwm = 0;
    }

    pwm = constrain(pwm, 0, PWM_MAX);

    // 슬루레이트 제한: pwm 급상승만 제한(관성 오버슈트 방지), 하강은 즉시 반영
    if (pwm > lastPwm + PWM_SLEW_MAX) pwm = lastPwm + PWM_SLEW_MAX;
    lastPwm = pwm;

    analogWrite(10, pwm);

    Serial.print(pwm);
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

void enc20() {
  encCount20++;
}

void enc21() {
  encCount21++;
}
