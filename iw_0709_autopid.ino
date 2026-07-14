// ============================================================
//  인휠모터 PID 튜닝 펌웨어 - 기초형으로 재시작 (Arduino Mega 2560)
//  iw_0707_pidtuning.ino 수준의 가장 기초적인 PID만 남김. FF/과속컷/적분게이팅
//  등은 전부 제거. 여기서부터 다시 차근차근 클램프/보정을 추가해나갈 것.
//
//  유지한 것 :
//   - 21번 핀만으로 제어 (20번은 모니터링 전용, 제어에는 미사용)
//   - 출력 구조 6필드 "<pwm> <target> <speed21> <speed20> 0 25" (기존과 동일)
//
//  뺀 것 (iw_0709 이전 버전 대비) :
//   - 피드포워드(FF_DEADZONE/FF_GAIN) 전부 제거 → pwm = kp*err + ki*i + kd*d 뿐
//   - 과속 컷 제거
//   - 적분 게이팅(rolling) 제거 → 안티와인드업은 "출력이 0~255 사이일 때만
//     적분 누적"이라는 가장 단순한 새추레이션 체크로 대체 (iw_0707과 동일)
//   - 미분을 측정값 기준이 아닌 오차 기준으로 되돌림 (d = err - lastErr)
//
//  입력 : 정수(목표펄스, 펄스/20ms) 한 줄. 0 = 정지. 그 외 형식은 무시.
//  출력 : "<pwm> <target> <speed21> <speed20> 0 25"  (20ms마다, 공백 구분)
// ============================================================

// 변수선언
volatile long encCount20 = 0;   // 모니터링 전용
volatile long encCount21 = 0;   // 왼쪽 모터컨트롤러 (PID 피드백)
unsigned long lastTime = 0;
int target = 0;                 // 목표 펄스값 (Reference, 시리얼로 입력)
float i = 0;
int lastErr = 0;

// PID 게인 (여기서부터 다시 튜닝 시작)
float kp = 1.0;
float ki = 0.0;
float kd = 0.0;

// PWM 상한 (무슨 일이 있어도 이 값을 넘기지 않음)
const int PWM_MAX = 170;

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
        target = atoi(rxBuf);
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

  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(20), enc20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), enc21, CHANGE);

  pinMode(10, OUTPUT);

  lastTime = millis();
}

void loop() {
  pollSerial();

  // 밑부분에 제어주기 설정
  if (millis() - lastTime >= 20) {
    lastTime = millis();

    noInterrupts();
    long speed21 = encCount21;  // 왼쪽 모터 속도 (PID 제어에 사용)
    long speed20 = encCount20;  // 모니터링용 속도 (제어에는 미사용)
    encCount21 = 0;
    encCount20 = 0;
    interrupts();

    int err = abs(target) - (int)speed21;
    int d = err - lastErr;
    lastErr = err;

    int pwm = kp * err + ki * i + kd * d;

    // 출력이 포화(0 또는 PWM_MAX)면 적분 누적 중단 → windup 방지
    if (pwm > 0 && pwm < PWM_MAX) {
      i += err;
    }

    // 계산 다시 (누적 반영)
    pwm = kp * err + ki * i + kd * d;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < 0) pwm = 0;

    // 하드 클램프: 위에서 이미 걸렸어도 한 번 더 강제 (170을 절대 넘기지 않음)
    pwm = constrain(pwm, 0, PWM_MAX);

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
