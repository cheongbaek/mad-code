// ============================================================
//  인휠 PWM -> 펄스 배선 검증용 코드 (Arduino Mega 2560)
//  kasa_0730_A.ino 기반. REF값 계산 / PID / FF / 코스트-캐치 / 폭주감지 / E-stop 전부 제거.
//  받은 PWM을 좌/우에 그대로 출력하고, 후보 홀센서 핀 6개(2,3,18,19,20,21)를
//  전부 인터럽트로 걸어 실제로 어느 핀에서 펄스가 들어오는지만 확인한다.
//  ※ 순수 검증용: PWM 입력 -> 펄스 읽기 기능만 수행하며 안전장치(E-stop 등)는 없다.
//
//  입력 : "<PWM>" 또는 "<왼쪽PWM>,<오른쪽PWM>"  (0~255, 개행 종료)
//         - 단일 값 : 양쪽에 동일 PWM 적용
//         - 콤마 2값 : 좌/우 독립 PWM 적용 (예: "50,50")
//         - 범위 밖 값 / 숫자 아닌 토큰 / 형식 오류(공백, 콤마 2개 이상 등)는 무시(직전 값 유지)
//  출력 : "(2번펄스),(3번펄스),(18번펄스),(19번펄스),(20번펄스),(21번펄스)" - 50ms 주기
//         각 값은 직전 20ms 창(CONTROL_WINDOW_MS) 동안 카운트된 펄스 수
// ============================================================


// ================= 핀 정의 =================
const uint8_t PWM_PIN_L = 8;
const uint8_t PWM_PIN_R = 9;

// 후보 홀센서 핀 6개 전부 인터럽트로 계측 (실배선 확인용, 전부 CHANGE 모드)
const uint8_t HALL_PIN_2  = 2;
const uint8_t HALL_PIN_3  = 3;
const uint8_t HALL_PIN_18 = 18;
const uint8_t HALL_PIN_19 = 19;
const uint8_t HALL_PIN_20 = 20;
const uint8_t HALL_PIN_21 = 21;


// ================= 통신 / 주기 =================
const unsigned long BAUD = 115200;
const unsigned long CONTROL_WINDOW_MS = 20;   // 펄스 계측 창(리셋 주기)
const unsigned long TELE_MS = 50;             // 출력 주기


// ================= 인터럽트 카운터 =================
volatile long encCount2  = 0;
volatile long encCount3  = 0;
volatile long encCount18 = 0;
volatile long encCount19 = 0;
volatile long encCount20 = 0;
volatile long encCount21 = 0;

void isr2()  { encCount2++; }
void isr3()  { encCount3++; }
void isr18() { encCount18++; }
void isr19() { encCount19++; }
void isr20() { encCount20++; }
void isr21() { encCount21++; }

// 직전 20ms 창의 스냅샷 (텔레메트리는 이 값을 그대로 출력)
long pulse2 = 0, pulse3 = 0, pulse18 = 0, pulse19 = 0, pulse20 = 0, pulse21 = 0;


// ================= PWM 출력값 (직접 출력, PID 없음) =================
int pwmL = 0;
int pwmR = 0;


// ================= 시간 =================
unsigned long pulse_t = 0;
unsigned long tele_t = 0;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 입력 형식 검사 (부호 없는 정수만) =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}

// 4자리 이상은 atoi 오버플로 방지용으로 256(=범위 밖)으로 통일
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}


// ================= PWM 즉시 적용 (0~255 클램프) =================
void applyPwm(uint8_t idx, long v) {
  int p = (int)constrain(v, 0, 255);
  if (idx == 0) {
    pwmL = p;
    analogWrite(PWM_PIN_L, pwmL);
  } else {
    pwmR = p;
    analogWrite(PWM_PIN_R, pwmR);
  }
}


// ================= 입력 파서 =================
// "<값>" 단일 = 좌/우 동일 PWM, "<좌값>,<우값>" = 좌/우 독립 PWM. 형식 오류는 무시.
void handleLine(char* line) {
  if (strchr(line, ' ')) return;   // 공백 포함 줄은 형식 오류

  char* comma = strchr(line, ',');
  if (comma) {
    *comma = '\0';
    char* tokR = comma + 1;
    if (strchr(tokR, ',')) return;   // 콤마 2개 이상 → 무시

    long vL = parseValue(line);
    long vR = parseValue(tokR);
    if (vL < 0 || vR < 0) return;

    applyPwm(0, vL);
    applyPwm(1, vR);
  } else {
    long v = parseValue(line);
    if (v < 0) return;

    applyPwm(0, v);
    applyPwm(1, v);
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


// ================= 펄스 계측 (20ms 창마다 스냅샷 후 리셋) =================
void updatePulses(unsigned long now) {
  if (now - pulse_t < CONTROL_WINDOW_MS) return;
  pulse_t += CONTROL_WINDOW_MS;

  noInterrupts();
  pulse2  = encCount2;  encCount2  = 0;
  pulse3  = encCount3;  encCount3  = 0;
  pulse18 = encCount18; encCount18 = 0;
  pulse19 = encCount19; encCount19 = 0;
  pulse20 = encCount20; encCount20 = 0;
  pulse21 = encCount21; encCount21 = 0;
  interrupts();
}


// ================= 출력 =================
// "(2번펄스),(3번펄스),(18번펄스),(19번펄스),(20번펄스),(21번펄스)" - 50ms 주기
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  Serial.print(pulse2);  Serial.print(',');
  Serial.print(pulse3);  Serial.print(',');
  Serial.print(pulse18); Serial.print(',');
  Serial.print(pulse19); Serial.print(',');
  Serial.print(pulse20); Serial.print(',');
  Serial.println(pulse21);
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(HALL_PIN_2,  INPUT_PULLUP);
  pinMode(HALL_PIN_3,  INPUT_PULLUP);
  pinMode(HALL_PIN_18, INPUT_PULLUP);
  pinMode(HALL_PIN_19, INPUT_PULLUP);
  pinMode(HALL_PIN_20, INPUT_PULLUP);
  pinMode(HALL_PIN_21, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN_2),  isr2,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_3),  isr3,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_18), isr18, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_19), isr19, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_20), isr20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_21), isr21, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  unsigned long now = millis();
  pulse_t = tele_t = now;
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();
  updatePulses(now);
  sendOutput(now);
}
