// ============================================================
//  인휠모터 PID 튜닝 펌웨어 - FF 보간 테이블 적용판 (Arduino Mega 2560)
//  iw_0709_autopid.ino(기초형) 기반 + iw_0709_pwm2pulse.ino 실측 데이터로
//  만든 PWM-펄스 보간 테이블을 피드포워드로 사용. PID는 1, 0, 0부터 재시작.
//
//  피드포워드 테이블 출처 (iw_0709_pwm2pulse.ino 실측, 21번 채널, 반올림) :
//    PWM  60  70  80  90 100 110 120 130 140 150 160 170
//    펄스   1   2   3   4   6   7   8  11  13  17  22  24
//  테이블 자체는 "목표펄스가 정해졌을 때 필요한 PWM"이 필요하므로 위 표를
//  펄스 기준 오름차순으로 뒤집어 저장(펄스 -> PWM), 구간마다 가장 가까운
//  3점을 골라 라그랑주 2차보간(Simpson 1/3 법칙이 적분에 쓰는 것과 동일한
//  포물선)으로 매끄럽게 잇는다. 표 범위(펄스 1~24) 밖은 외삽하지 않고
//  양끝값으로 고정(테이블 시작 전은 0->첫값까지 직선, 끝은 170 고정).
//
//  PWM 상한 : 무슨 일이 있어도 170을 넘기지 않음 (PWM_MAX, 이중 클램프).
//
//  홀펄스 : 20/21번 핀 모두 수신, PID 피드백은 21번만 사용, 20번은 모니터링.
//  입력 : 정수(목표펄스) 한 줄. 그 외 형식은 무시.
//  출력 : "<pwm> <target> <speed21> <speed20> 0 25"  (20ms마다, 공백 구분)
// ============================================================

// ================= 피드포워드 보간 테이블 (펄스 -> PWM, 오름차순) =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1,  2,  3,  4,  6,  7,  8, 11, 13, 17, 22, 24 };
const float ffPwmTable[FF_TABLE_N]   = {60, 70, 80, 90,100,110,120,130,140,150,160,170};

// Simpson 1/3 법칙과 동일한 3점 포물선(라그랑주 2차보간)으로 목표펄스 -> PWM 보간
float interpFF(float x) {
  if (x <= 0) return 0.0;
  if (x <= ffPulseTable[0]) {
    return ffPwmTable[0] * (x / ffPulseTable[0]);   // 0~첫값 구간은 직선 연결
  }
  if (x >= ffPulseTable[FF_TABLE_N - 1]) {
    return ffPwmTable[FF_TABLE_N - 1];               // 특성화 범위 밖은 외삽 안 하고 고정
  }

  // x가 속한 구간 [i, i+1] 탐색
  int i = 0;
  while (i < FF_TABLE_N - 2 && x > ffPulseTable[i + 1]) i++;

  // 해당 구간을 감싸는 가장 가까운 3점 선택 (Simpson 1/3의 포물선과 동일한 방식)
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

// ================= 변수선언 =================
volatile long encCount20 = 0;   // 모니터링 전용
volatile long encCount21 = 0;   // 왼쪽 모터컨트롤러 (PID 피드백)
unsigned long lastTime = 0;
int target = 0;                 // 목표 펄스값 (Reference, 시리얼로 입력)
float i = 0;
int lastErr = 0;

// PID 게인 (여기서부터 다시 튜닝 시작)
float kp = 1.0;
float ki_up   = 0.0;   // 가속(err>=0, 목표>실제) 구간 I 게인 - 응답 좋아서 약하게
float ki_down = 0.0;   // 감속(err<0, 목표<실제) 구간 I 게인 - 관성 때문에 세게 (1:10 비율로 시작)
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

    float ff = interpFF((float)abs(target));   // 목표펄스에 대응하는 보간 피드포워드

    float ki = (err >= 0) ? ki_up : ki_down;   // 가속/감속 방향에 따라 I 게인 전환

    int pwm = ff + kp * err + ki * i + kd * d;

    // 출력이 포화(0 또는 PWM_MAX)면 적분 누적 중단 → windup 방지
    if (pwm > 0 && pwm < PWM_MAX) {
      i += err;
    }

    // 계산 다시 (누적 반영)
    pwm = ff + kp * err + ki * i + kd * d;
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
