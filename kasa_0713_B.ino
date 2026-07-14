// ============================================================
//  B보드 : 조향 + 제동 (Arduino Mega 2560) - 0713 버전
//  kasa_0712_none.ino에서 인휠/펄스 분리(→ A보드), 입출력 규약 변경
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, A보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<조향각도>,<브레이크출력>"  (콤마 구분, 개행 종료)
//         - 예: "-10,-50" → 조향각 -10, 브레이크 -50
//         - 브레이크출력은 -255~255 부호있는 값
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "P,<조향각환산값>" (평상시) / "STOP" (e-stop 중)
//         - 가변저항 미장착 상태 → 현재는 항상 "P,0" 고정 출력 (추후 ADC 장착 시 교체)
//
//  조향(DC)모터 : 열린루프, PWM = |각도| * 4, 음수=왼쪽/양수=오른쪽/0=정지
//  리니어(브레이크)모터 : 부호/크기로 열린루프 타이밍 구동 (2초)
//
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 조향 PWM 0, 리니어 최고출력 체결 방향 2초간 구동 후 정지, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- DC 조향모터 (MD20A, 가변저항 미사용) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성


// ================= 통신 =================
const unsigned long BAUD = 115200;


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


// ================= 조향 상태 =================
int  steer_angle_cmd = 0;


// ================= 브레이크(리니어) 상태 =================
int  brake_cmd     = 0;
int  brake_output  = 0;
bool linear_running = false;
unsigned long linear_start_t = 0;


// ================= E-stop 상태 =================
bool estop_active = false;

// e-stop 시 리니어 체결 구동 시간 (이 시간 후 리니어 출력 정지)
const unsigned long ESTOP_BRAKE_MS = 2000;
bool estop_latched = false;          // e-stop 진입(엣지) 감지용
unsigned long estop_engage_t = 0;    // 체결 시작 시각

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
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearCW(int p); void linearCCW(int p);
void applyEstop(unsigned long now);
void applySteer(int angle);
void startLinear(int cmd);
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateBrake(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop(unsigned long now) {
  dcStop();
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


// ================= 가변저항 환산 현재 조향각 =================
// 가변저항 미장착 상태 → 항상 0 반환 (추후 ADC 장착 시 이 함수만 교체)
int readSteerAngle() {
  return 0;
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
  tele_t = now;
}


// ================= 입력 파서 =================
// "<조향각도>,<브레이크출력>" 콤마 구분 정수 2개. 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 ||
      !isValidNumber(tok1) || !isValidNumber(tok2)) {
    return;
  }

  int angle = atoi(tok1);
  int brake = atoi(tok2);

  // e-stop 중에는 구동 명령(조향/브레이크) 미적용 → 리니어 재구동 방지
  if (estop_active) return;

  steer_angle_cmd = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  applySteer(steer_angle_cmd);
  brake_cmd = constrain(brake, -BRAKE_MAX, BRAKE_MAX);
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
// "P,<조향각환산값>" — 가변저항 미장착 상태라 현재는 항상 "P,0"
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("P,");
  Serial.println(readSteerAngle());
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
    applyEstop(now);
  } else {
    if (estop_latched) {
      // e-stop 해제 엣지: 체결 구동 중이던 리니어를 반드시 정지
      // (2초 내 해제 시 updateBrake가 못 꺼주는 상태로 남는 것 방지)
      linearStop();
      brake_output = 0;
      estop_latched = false;
    }
    updateBrake(now);
  }

  sendOutput(now);
}
