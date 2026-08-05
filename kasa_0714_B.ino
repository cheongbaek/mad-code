// ============================================================
//  B보드 : 조향(PD 위치제어) + 제동 (Arduino Mega 2560) - 0714 버전
//  kasa_0713_B.ino의 조향부(열린루프)를 dc_0702_pd.ino의 PD 위치제어로 교체
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, A보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - 단선(와이어 끊김)에도 정지되는 페일세이프
//
//  입력 : "<조향각도>,<브레이크출력>"  (콤마 구분, 개행 종료)
//         - 예: "-10,-50" → 조향각 -10, 브레이크 -50
//         - 브레이크출력은 -255~255 부호있는 값
//         - 형식이 안 맞는 줄은 그냥 무시
//  출력 : "P,<조향각환산값>" (평상시, 가변저항 실측 기반) / "STOP" (e-stop 중)
//
//  조향(DC)모터 : A0 가변저항 피드백 PD 위치제어
//    - 목표각도 -40/+40 은 실측 좌/우 하드 리밋(raw)보다
//      SAFETY_MARGIN(10)만큼 안쪽으로 매핑됨
//    - 가변저항 현재값이 실측 하드 리밋(RAW_LEFT_LIMIT/RAW_RIGHT_LIMIT)에
//      도달하면 안전마진과 무관하게 즉시 PWM 0 (페일세이프)
//    - |오차| <= STEER_TOLERANCE(3) 상태가 SETTLE_MS(0.5초) 이상 지속되면
//      "도달"로 판정, 모터 정지 후 대기 상태로 전환 (대기 중엔 재구동 안 함)
//    - 하드 리밋 raw값(933/751)은 dc_0701_potential.ino 실측치를 재사용
//      (다른 개체로 교체 시 반드시 재측정할 것)
//  리니어(브레이크)모터 : 부호/크기로 열린루프 타이밍 구동 (2초, 기존과 동일)
//  리니어 MB - 빨간색, MA - 검은색
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 조향 PWM 0, 리니어 최고출력 체결 방향 2초간 구동 후 정지, "STOP" 출력
//    - e-stop 해제 시 조향 PD 목표를 그 시점의 현재 위치로 재동기화하여
//      해제 순간 과거 목표각도로 급조향하는 것을 방지
//
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- DC 조향모터 (MD20A + 가변저항 A1) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A1;

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 조향 PD 게인 (여기서 조절) =================
float KP_S = 6.0f;
float KD_S = 0.1f;

// ================= 조향 PWM 상한/하한 =================
const int STEER_MIN_PWM = 110;
const int STEER_MAX_PWM = 255;

// ================= 조향 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;   // dc_0702_pd.ino와 동일 (PD게인 호환)

// ================= 조향 입력 각도 범위 =================
const int STEER_ANGLE_MAX =  40;
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

// ================= 실측 좌/우 하드 리밋 (raw, 0~1023) =================
// dc_0701_potential.ino 로 측정한 값 재사용 (왼쪽 933 / 오른쪽 751 / 중앙 886)
// 다른 가변저항/모터 개체로 교체 시 반드시 재측정 후 갱신할 것
const int RAW_LEFT_LIMIT  = 1001;   // 왼쪽 끝 (하드 리밋)
const int RAW_RIGHT_LIMIT = 751;   // 오른쪽 끝 (하드 리밋)

// ================= 조향 안전 여유값 =================
const int SAFETY_MARGIN = 10;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// -40도/+40도에 대응하는 목표 raw값 (하드 리밋보다 SAFETY_MARGIN만큼 안쪽)
const int POT_AT_ANGLE_MIN = RAW_LEFT_LIMIT  - SAFETY_MARGIN;   // 각도 -40 -> 이 raw값
const int POT_AT_ANGLE_MAX = RAW_RIGHT_LIMIT + SAFETY_MARGIN;   // 각도 +40 -> 이 raw값

// ================= 조향 도달 판정 =================
const int STEER_TOLERANCE = 3;
const unsigned long SETTLE_MS = 500;   // 허용범위 유지 시간 -> 도달 판정

#define DIR_CW   HIGH   // 왼쪽
#define DIR_CCW  LOW    // 오른쪽


// ================= 브레이크(리니어) 열린루프 타이밍 구동 =================
const int BRAKE_MAX = 255;                  // 브레이크출력 크기 상한
const unsigned long LINEAR_RUN_MS = 220;   // 입력받은 방향/크기로 구동하는 시간

// e-stop 시 리니어를 강제로 체결시키는 방향
const uint8_t LINEAR_ENGAGE_DIR = DIR_CW;


// ================= 조향 PD 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int  steer_angle_cmd = 0;     // 마지막으로 수신한 명령 각도 (참고/디버그용)
int  target_pos = 512;        // PD 목표 raw값
int  prev_pos   = 0;          // 미분항 계산용 이전 raw값
unsigned long steer_win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;


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
int  angleToPot(int angle);
int  potToAngle(int raw);
void updateSteer(unsigned long now);
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


// ================= 각도 <-> raw 변환 =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX);
}

int potToAngle(int raw) {
  int angle = map(raw, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}


// ================= 조향 PD 제어 (CONTROL_WINDOW_MS 주기) =================
void updateSteer(unsigned long now) {
  if (now - steer_win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_win_t = now;

  int cur = analogRead(DC_POT_PIN);

  // ── 하드 리밋 도달 시 즉시 정지 (안전마진 무관, 페일세이프) ──
  bool atHardLimit = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT)
                        ? (cur >= RAW_LEFT_LIMIT || cur <= RAW_RIGHT_LIMIT)
                        : (cur <= RAW_LEFT_LIMIT || cur >= RAW_RIGHT_LIMIT);
  if (atHardLimit) {
    dcStop();
    prev_pos = cur;
    return;
  }

  // ── 대기 상태: 가변저항 변화 무시, 정지 유지 ──
  if (steer_state == ST_SETTLED) {
    dcStop();
    prev_pos = cur;
    return;
  }

  // ── PD 제어 ──
  int err = target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - prev_pos) / dt);
  float output = p + d;
  prev_pos = cur;

  if (abs(err) <= STEER_TOLERANCE) {
    dcStop();
    if (!settleTimerRunning) {
      settleTimerRunning = true;
      settleStart = now;
    } else if (now - settleStart >= SETTLE_MS) {
      steer_state = ST_SETTLED;   // 0.5초 이상 허용범위 유지 -> 도달 판정, 대기로 전환
    }
  } else {
    settleTimerRunning = false;   // 허용범위를 벗어나면 도달 판정 타이머 리셋
    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    if (output > 0) dcCW(spd); else dcCCW(spd);
  }
}


// ================= 가변저항 환산 현재 조향각 (텔레메트리용) =================
int readSteerAngle() {
  return potToAngle(analogRead(DC_POT_PIN));
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
  pinMode(DC_POT_PIN, INPUT);
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

  // 조향 PD: 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  target_pos = analogRead(DC_POT_PIN);
  prev_pos   = target_pos;
  steer_win_t = now;
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
  target_pos = angleToPot(steer_angle_cmd);
  settleTimerRunning = false;
  steer_state = ST_ACTIVE;                      // 새 각도 입력 -> PD 제어 재개

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
// "P,<조향각환산값>" — 가변저항 실측 기반 현재 조향각
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

      // 조향 PD 목표를 해제 시점의 현재 위치로 재동기화
      // (e-stop 중 가변저항이 밀렸어도 과거 목표로 급조향하지 않도록)
      int cur = analogRead(DC_POT_PIN);
      target_pos = cur;
      prev_pos = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
    }
    updateBrake(now);
    updateSteer(now);
  }

  sendOutput(now);
}
