// 변수선언
volatile long encCountL = 0;   // 왼쪽 모터 펄스 카운트 (핀 2)
volatile long encCountR = 0;   // 오른쪽 모터 펄스 카운트 (핀 3) - 지금은 튜닝에 미사용, 추후 확장용
long lastCountL = 0;
long lastCountR = 0;
long lastSpeed = 0;            // 미분항 계산용 (측정값 기준 미분)
unsigned long lastTime = 0;
int target = 0;                // 목표 펄스값 (Reference, 시리얼로 입력)
float i = 0;

// PID 게인 (왼쪽 모터)
float kp = 20.0;
float ki = 2.0;
float kd = 2.0;

const float I_MAX = 100.0;     // 적분 클램프 값 (실측하며 조정)

// 피드포워드 (실측 정적 맵 기반: 데드존 ≈60 PWM, 25펄스 ≈ 220 PWM)
// ff = FF_DEADZONE + FF_GAIN * target  (target=0이면 0)
const float FF_DEADZONE = 60.0;
const float FF_GAIN     = 6.4;

// 시리얼 입력 버퍼 (parseInt의 CRLF/타임아웃 문제 방지용 줄 단위 파싱)
char rxBuf[16];
uint8_t rxLen = 0;

// 정수 형식 검사 (부호 허용)
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

// 시리얼에서 목표 펄스값(Reference) 수신 - 줄 단위, 숫자 아니면 무시
void pollSerial() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0 && isValidNumber(rxBuf)) {
        int newTarget = atoi(rxBuf);
        if (newTarget != target) {
          i = 0;   // 목표값 변경 시 적분 리셋 (이전 목표의 적분 잔재 제거)
        }
        target = newTarget;
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;   // 버퍼 초과 -> 폐기
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), encR, CHANGE);
  pinMode(10, OUTPUT);
  lastTime = millis();
}

void loop() {
  pollSerial();

  // 밑부분에 제어주기 설정 (20ms)
  if (millis() - lastTime >= 20) {
    lastTime = millis();

    long l_speed = encCountL - lastCountL;  // 왼쪽 모터 속도 (PID 제어에 사용)
    lastCountL = encCountL;

    long r_speed = encCountR - lastCountR;  // 오른쪽 모터 속도 (확인용, 제어에는 미사용)
    lastCountR = encCountR;

    int err = abs(target) - (int)l_speed;

    long d = -(l_speed - lastSpeed);  // 측정값 기준 미분 (derivative on measurement)
    lastSpeed = l_speed;

    // 피드포워드: 목표값에 대응하는 기본 PWM (PID는 보정만 담당)
    float ff = (target > 0) ? (FF_DEADZONE + FF_GAIN * target) : 0.0;

    // 현재 적분값으로 출력 후보 계산 (포화 판정용)
    float pwm_raw = ff + kp * err + ki * i + kd * d;

    // anti-windup: 출력이 포화된 방향으로는 적분 누적 중단
    bool sat_low  = (pwm_raw <= 0);
    bool sat_high = (pwm_raw >= 255);
    if (!(sat_low && err < 0) && !(sat_high && err > 0)) {
      i += err;
      if (i > I_MAX) i = I_MAX;
      if (i < -I_MAX) i = -I_MAX;
    }

    // 누적 반영하여 최종 출력 계산
    int pwm = ff + kp * err + ki * i + kd * d;
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    analogWrite(10, pwm);

    Serial.print(pwm);
    Serial.print(" ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print(l_speed);
    Serial.print(" ");
    Serial.print(r_speed);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(25);
  }
}

void encL() {
  encCountL++;
}

void encR() {
  encCountR++;
}
