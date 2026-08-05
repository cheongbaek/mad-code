// ============================================================
//  리니어(브레이크)모터 단독 테스트 (Arduino Mega 2560) - 0718 버전
//  kasa_0714_B.ino의 리니어 구동부(핀/방향)만 그대로 가져온 간단 테스트 코드.
//  리니어모터 단자: MB-RED, MA-BLACK
//
//  입력 : "<구동시간(초)>,<PWM>"  (콤마 구분, 개행 종료)
//         - 구동시간은 0보다 큰 실수 (예: 3, 1.1)
//         - PWM은 부호있는 정수 -255~255 (부호 = 방향, 절댓값 = 세기)
//           예) "3,150"   -> 3초 동안 PWM 150 방향(양수)으로 구동
//               "1.1,-200" -> 1.1초 동안 PWM 200 반대 방향(음수)으로 구동
//         - 형식이 안 맞는 줄, 시간<=0, 숫자 아닌 토큰은 그냥 무시
//         - 구동 중 새 명령이 들어오면 기존 구동을 즉시 중단하고 새 명령으로 갱신
//  동작 : delay() 미사용, millis() 기준으로 구동시간을 관리 (논블로킹)
//  출력 : 명령 수신/구동 종료 시점에만 상태를 1줄 출력 (평상시엔 조용히 구동만 함)
// ============================================================


// ================= 핀 정의 (kasa_0714_B.ino와 동일) =================
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

#define DIR_CW   HIGH   // 브레이크출력 양수 방향 (kasa_0714_B와 동일 매핑)
#define DIR_CCW  LOW    // 브레이크출력 음수 방향


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= PWM 상한 =================
const int PWM_MAX = 255;


// ================= 구동 상태 =================
bool running = false;
unsigned long run_start_t = 0;
unsigned long run_dur_ms = 0;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 모터 출력 =================
void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, PWM_MAX)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, PWM_MAX)); }


// ================= 입력 형식 검사 =================
// 시간 토큰: 0~9와 '.' 하나만 허용(부호 없음, 최소 숫자 1개)
bool isValidSeconds(const char* s) {
  if (!s || *s == '\0') return false;
  bool sawDigit = false, sawDot = false;
  for (uint8_t i = 0; s[i] != '\0'; i++) {
    if (s[i] == '.') {
      if (sawDot) return false;
      sawDot = true;
    } else if (isdigit((unsigned char)s[i])) {
      sawDigit = true;
    } else {
      return false;
    }
  }
  return sawDigit;
}

// PWM 토큰: 부호(+/-) 허용 정수
bool isValidSignedInt(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}


// ================= 구동 시작 =================
void startRun(float seconds, int pwmVal) {
  pwmVal = constrain(pwmVal, -PWM_MAX, PWM_MAX);

  if (pwmVal == 0) {
    linearStop();
    running = false;
    Serial.println("STOP,0");
    return;
  }

  if (pwmVal > 0) linearCW(pwmVal);
  else            linearCCW(-pwmVal);

  run_start_t = millis();
  run_dur_ms  = (unsigned long)(seconds * 1000.0f);
  running = true;

  Serial.print("RUN,");
  Serial.print(seconds, 2);
  Serial.print(',');
  Serial.println(pwmVal);
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, DIR_CW);
  linearStop();
}


// ================= 입력 파서 =================
// "<구동시간(초)>,<PWM>" 콤마 구분 2개. 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 ||
      !isValidSeconds(tok1) || !isValidSignedInt(tok2)) {
    return;
  }

  float seconds = atof(tok1);
  if (seconds <= 0) return;

  int pwmVal = atoi(tok2);
  startRun(seconds, pwmVal);
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


// ================= 구동시간 관리 (millis 기준, 논블로킹) =================
void updateRun(unsigned long now) {
  if (!running) return;
  if (now - run_start_t >= run_dur_ms) {
    linearStop();
    running = false;
    Serial.println("DONE");
  }
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();
  updateRun(now);
}
