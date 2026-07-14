// ============================================================
//  통합 차량 제어 펌웨어 (Arduino Mega 2560) - 0709 가변저항 미사용 버전
//  kasa_0708_test.ino 기반, 조향 가변저항(포텐셔미터) 사용 완전 배제
//
//  입력 : "<주행목표펄스> <조향각도> <브레이크출력> <모드>"  (스페이스 구분, 개행 종료)
//         예) 0 -30 0   → 왼쪽으로 PWM 120 (= |-30| * 4)
//             0 5 0     → 오른쪽으로 PWM 20 (= 5 * 4)
//             0 0 0     → 조향 정지
//         - 3번째 값(브레이크출력)은 -255~255 부호있는 값
//         - 4번째 값(모드)이 1이면 e-stop 발동 (없으면 0, 평상시)
//         - 형식이 안 맞는 줄은 그냥 무시 (e-stop 발동 없음)
//  출력 : "<20번펄스> <21번펄스> <조향각도>" (평상시) / "STOP" (e-stop 중)
//         21번 = 왼쪽 모터컨트롤러 (PID 피드백에 사용), 20번 = 모니터링 전용
//         조향각도 = 가변저항 환산 없이 입력받은 각도값을 그대로 대입
//
//  조향(DC)모터 : 피드백/PD 없이 열린루프 구동
//    - 각도 음수 → 왼쪽(DIR_CW), 각도 양수 → 오른쪽(DIR_CCW)
//    - PWM = |각도| * STEER_PWM_PER_DEG(4), 최대 255 클램프
//    - 새 명령이 올 때까지 해당 출력 유지, 각도 0이면 정지
//    - ※ 피드백이 없으므로 하드리밋에 밀어붙인 채 유지될 수 있음 → 0으로 정지 필수
//
//  인휠 모터 : iw_0708_pidtuning.ino의 FF + PID (출력 PWM 10번, 21번 펄스 피드백)
//  리니어(브레이크) 모터 : 부호/크기로 열린루프 타이밍 구동 (2초)
//    - 양수 → DIR HIGH(나옴/신장), 음수 → DIR LOW(들어감/수축), 0 → 즉시 정지
//  제어주기 : 20ms 공유 (CONTROL_WINDOW_MS)
//
//  ※ 현재 ESTOP_ENABLED = false 로 임시 비활성화 상태 (true/false 스위치는 유지)
//  E-stop 조건 (매 루프 재평가, 하나라도 충족 시 발동) :
//    ① 40번 핀 단락(LOW)  ② 모드값 1 입력  ③ 5초 이상 입력 없음
//  E-stop 동작 : 인휠/조향 PWM 0, 리니어 최고출력 체결 방향, "STOP" 출력
//  E-stop 해제 : ①②가 해소되면 해제 (③은 새 입력이 들어오면 자동 해소).
//    해제 후에도 5초 무입력이면 재발동(③). 해제 시 속도/조향은 0부터 재시작하며,
//    브레이크는 별도 해제 명령 없이 직전 상태 유지(새 브레이크 입력이 와야 변경).
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN20 = 20;   // 모니터링 전용
const uint8_t HALL_PIN21 = 21;   // 왼쪽 모터컨트롤러 (PID 피드백에 사용)

// --- 인휠 주행 PWM ---
const uint8_t INWHEEL_PWM_PIN = 10;

// --- DC 조향모터 (MD20A, 가변저항 미사용) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- E-stop ---
const uint8_t ESTOP_PIN = 40;          // INPUT_PULLUP, 단락(LOW) 시 e-stop
const bool ESTOP_ENABLED = false;      // TODO: 임시 비활성화, 테스트 끝나면 true로 복구


// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long INPUT_TIMEOUT_MS = 5000;   // 5초 이상 무입력 → e-stop


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= 인휠 PID (iw_0708_pidtuning.ino 실측 튜닝값) =================
float kp = 20.0;
float ki = 2.0;
float kd = 2.0;

const float I_MAX = 100.0;     // 적분 클램프 값

// 피드포워드 (실측 정적 맵: 데드존 ≈60 PWM, 25펄스 ≈ 220 PWM)
// ff = FF_DEADZONE + FF_GAIN * wheelREF  (wheelREF=0이면 0)
const float FF_DEADZONE = 60.0;
const float FF_GAIN     = 6.4;


// ================= 조향 (열린루프, 각도 → PWM 직결) =================
const int STEER_PWM_PER_DEG = 4;     // PWM = |각도| * 4
const int STEER_ANGLE_MAX   =  40;   // 입력 각도 클램프 범위
const int STEER_ANGLE_MIN   = -STEER_ANGLE_MAX;

#define DIR_CW   HIGH   // 왼쪽 (dc_0701_dirtest.ino 방향 정의와 동일)
#define DIR_CCW  LOW    // 오른쪽


// ================= 브레이크(리니어) 열린루프 타이밍 구동 =================
const int BRAKE_MAX = 255;               // 브레이크출력 크기 상한 (부호는 방향, 절댓값은 크기)
const unsigned long LINEAR_RUN_MS = 2000;   // 입력받은 방향/크기로 구동하는 시간

// e-stop 시 리니어를 강제로 체결시키는 방향
// 실측: DIR_CW(HIGH)=나옴(신장), DIR_CCW(LOW)=들어감(수축)
// TODO: 브레이크 체결이 신장/수축 중 어느 쪽인지 확인 후 확정
const uint8_t LINEAR_ENGAGE_DIR = DIR_CW;


// ================= 인휠 상태 =================
volatile long encCount20 = 0;   // 홀센서 20번 펄스 카운트 (모니터링)
volatile long encCount21 = 0;   // 홀센서 21번 펄스 카운트 (왼쪽, PID 피드백)
long   wheelREF     = 0;        // 목표 속도 (펄스/주기)
float  i_term       = 0;
int    lastSpeed21  = 0;        // 측정값 기준 미분용 (21번 핀 직전 속도)
int    wheel_pwm    = 0;
int    wheel_speed20 = 0;       // 20번 핀 측정 펄스 (주기당, 모니터링)
int    wheel_speed21 = 0;       // 21번 핀 측정 펄스 (주기당, PID 사용)
unsigned long wheel_t = 0;


// ================= 조향 상태 =================
int  steer_angle_cmd = 0;   // 입력받은 각도값 (출력 시 그대로 대입)


// ================= 브레이크(리니어) 상태 =================
int  brake_cmd     = 0;       // 입력받은 브레이크출력 (-BRAKE_MAX ~ BRAKE_MAX, 부호=방향)
int  brake_output  = 0;       // 현재 구동 중인 출력 크기 (0이면 정지)
bool linear_running = false;  // LINEAR_RUN_MS 동안 구동 중인지 여부
unsigned long linear_start_t = 0;


// ================= 모드 (4번째 입력값) =================
int mode_cmd = 0;   // 1 = e-stop 발동 (loop()에서 매 루프 검사, 없으면 0)


// ================= E-stop 상태 =================
bool estop_active = false;
unsigned long lastInputTime = 0;


// ================= 출력용 =================
int pulse20_out = 0;
int pulse21_out = 0;
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 함수 선언 =================
void encISR20();
void encISR21();
void inwheelWrite(int pwm);
void dcStop(); void dcCW(int p); void dcCCW(int p);
void linearStop(); void linearCW(int p); void linearCCW(int p);
void allMotorsStop();
void applyEstop();
void applySteer(int angle);
void startLinear(int cmd);
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateWheel(unsigned long now);
void updateBrake(unsigned long now);
void sendOutput(unsigned long now);


// ================= ISR (홀센서 20, 21번) =================
void encISR20() { encCount20++; }
void encISR21() { encCount21++; }


// ================= 모터 출력 =================
void inwheelWrite(int pwm) {
  pwm = constrain(pwm, 0, 255);
  analogWrite(INWHEEL_PWM_PIN, pwm);
  wheel_pwm = pwm;
}
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }

void allMotorsStop() { inwheelWrite(0); dcStop(); }

// e-stop 상태에서 매 루프 호출되는 안전 동작
void applyEstop() {
  allMotorsStop();

  // 속도/조향 목표를 0으로 리셋 → 해제 후 재개 시 직전 값이 아닌 0부터 시작
  wheelREF        = 0;
  steer_angle_cmd = 0;

  // 리니어(브레이크) 최고출력으로 체결 방향 강제 구동, 진행 중이던 타이밍 구동은 취소
  // (해제 시에는 이 값을 되돌리는 코드가 없으므로, 새 브레이크 입력이 오기 전까지 그대로 유지됨)
  digitalWrite(LINEAR_DIR_PIN, LINEAR_ENGAGE_DIR);
  analogWrite(LINEAR_PWM_PIN, BRAKE_MAX);
  brake_output = BRAKE_MAX;
  linear_running = false;

  // 인휠 PID 상태 초기화 (해제 후 재개 시 적분 잔재/펄스 누적 방지)
  i_term = 0;
  lastSpeed21 = 0;
  noInterrupts();
  encCount20 = 0;
  encCount21 = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 조향각도 → 열린루프 PWM 구동 =================
// 음수 = 왼쪽(DIR_CW), 양수 = 오른쪽(DIR_CCW), 0 = 정지
// PWM = |각도| * STEER_PWM_PER_DEG, 새 명령이 올 때까지 유지
void applySteer(int angle) {
  int pwm = abs(angle) * STEER_PWM_PER_DEG;
  if (angle < 0)      dcCW(pwm);    // 왼쪽
  else if (angle > 0) dcCCW(pwm);   // 오른쪽
  else                dcStop();
}


// ================= 브레이크출력 → 리니어 열린루프 구동 시작 =================
// cmd > 0: DIR HIGH, cmd < 0: DIR LOW, cmd == 0: 즉시 정지
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

  // 홀센서 (21번 = 왼쪽 PID 피드백, 20번 = 모니터링)
  pinMode(HALL_PIN20, INPUT_PULLUP);
  pinMode(HALL_PIN21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN20), encISR20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN21), encISR21, CHANGE);

  // 인휠
  pinMode(INWHEEL_PWM_PIN, OUTPUT);
  inwheelWrite(0);

  // 조향 (가변저항 미사용, DIR+PWM만)
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  // 리니어(브레이크)
  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, DIR_CW);
  linearStop();

  // E-stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  wheel_t = tele_t = now;
  lastInputTime = now;   // 부팅 직후 5초의 유예시간 부여
}


// ================= 입력 파서 =================
// "<target> <angle> <brake> <mode>" 형식(공백 구분). 형식이 안 맞으면 그냥 무시.
// 4번째 토큰(모드)은 여기서 저장만 하고, e-stop 판정은 loop()에서 수행 (없으면 0 = 평상시).
void handleLine(char* line) {
  char* tok1 = strtok(line, " ");
  char* tok2 = tok1 ? strtok(NULL, " ") : NULL;
  char* tok3 = tok2 ? strtok(NULL, " ") : NULL;
  char* tok4 = tok3 ? strtok(NULL, " ") : NULL;
  char* tok5 = tok4 ? strtok(NULL, " ") : NULL;   // 토큰이 5개 이상이면 형식 오류

  if (!tok1 || !tok2 || !tok3 || tok5 ||
      !isValidNumber(tok1) || !isValidNumber(tok2) || !isValidNumber(tok3)) {
    return;   // 형식 오류 → 해당 줄 무시 (e-stop 발동 없음)
  }

  lastInputTime = millis();

  long driveTarget = atol(tok1);
  int  angle        = atoi(tok2);
  int  brake         = atoi(tok3);

  // 4번째 값(모드): 숫자면 저장만 해두고 현재는 무시 (숫자가 아니어도 줄 자체는 유효 처리)
  mode_cmd = (tok4 && isValidNumber(tok4)) ? atoi(tok4) : 0;

  long newREF = max(0L, driveTarget);
  if (newREF != wheelREF) {
    i_term = 0;   // 목표값 변경 시 적분 리셋 (이전 목표의 적분 잔재 제거)
  }
  wheelREF        = newREF;
  steer_angle_cmd = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  applySteer(steer_angle_cmd);
  brake_cmd       = constrain(brake, -BRAKE_MAX, BRAKE_MAX);
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
      rxLen = 0;   // 버퍼 초과 → 해당 줄 폐기
    }
  }
}


// ================= 인휠 제어 (FF + PID, iw_0708_pidtuning.ino 검증 구조) =================
void updateWheel(unsigned long now) {
  if (now - wheel_t < CONTROL_WINDOW_MS) return;
  wheel_t += CONTROL_WINDOW_MS;

  // 홀센서 21번(왼쪽, PID) / 20번(모니터링) 펄스로 속도 측정
  noInterrupts();
  long c20 = encCount20;
  long c21 = encCount21;
  encCount20 = 0;
  encCount21 = 0;
  interrupts();
  wheel_speed20 = (int)c20;
  wheel_speed21 = (int)c21;

  int err = (int)wheelREF - wheel_speed21;

  // 측정값 기준 미분 (derivative on measurement, 목표 변경 시 미분 킥 없음)
  int d = -(wheel_speed21 - lastSpeed21);
  lastSpeed21 = wheel_speed21;

  // 피드포워드: 목표값에 대응하는 기본 PWM (PID는 보정만 담당)
  float ff = (wheelREF > 0) ? (FF_DEADZONE + FF_GAIN * wheelREF) : 0.0;

  // 현재 적분값으로 출력 후보 계산 (포화 판정용)
  float pwm_raw = ff + kp * err + ki * i_term + kd * d;

  // anti-windup: 출력이 포화된 방향으로는 적분 누적 중단
  bool sat_low  = (pwm_raw <= 0);
  bool sat_high = (pwm_raw >= 255);
  if (!(sat_low && err < 0) && !(sat_high && err > 0)) {
    i_term += err;
    if (i_term > I_MAX) i_term = I_MAX;
    if (i_term < -I_MAX) i_term = -I_MAX;
  }

  // 누적 반영하여 최종 출력 계산
  int pwm = ff + kp * err + ki * i_term + kd * d;
  if (pwm > 255) pwm = 255;
  if (pwm < 0) pwm = 0;

  if (wheelREF == 0) {
    pwm = 0;
    i_term = 0;
  }

  inwheelWrite(pwm);
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
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");   // println: 줄 단위로 읽는 수신측(walker_k)과의 프레이밍 유지
    return;
  }

  pulse20_out = wheel_speed20;       // 모니터링
  pulse21_out = wheel_speed21;       // 왼쪽 (PID 피드백)
  int angle_out = steer_angle_cmd;   // 입력받은 각도값 그대로 (가변저항 환산 없음)

  Serial.print(pulse20_out);
  Serial.print(' ');
  Serial.print(pulse21_out);
  Serial.print(' ');
  Serial.println(angle_out);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();

  bool pinEstop     = (digitalRead(ESTOP_PIN) == LOW);
  bool modeEstop    = (mode_cmd == 1);
  bool timeoutEstop = (now - lastInputTime > INPUT_TIMEOUT_MS);
  estop_active = ESTOP_ENABLED && (pinEstop || modeEstop || timeoutEstop);

  if (estop_active) {
    applyEstop();
  } else {
    updateWheel(now);
    updateBrake(now);
  }

  sendOutput(now);
}
