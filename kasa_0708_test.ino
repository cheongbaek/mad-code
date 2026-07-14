// ============================================================
//  통합 차량 제어 펌웨어 (Arduino Mega 2560) - TEST
//  입력 : "<주행목표펄스> <조향각도> <브레이크출력> <모드>"  (스페이스 구분, 개행 종료)
//         예) 25 -20 120 0   /   25 -20 -120 0
//         - 3번째 값(브레이크출력)은 -255~255 부호있는 값
//         - 4번째 값(모드)은 현재 파싱만 하고 무시됨 (추후 e-stop/가변저항 확인 모드용 자리)
//         - 형식이 안 맞는 줄은 그냥 무시 (e-stop 발동 없음)
//  출력 : "<20번펄스> <21번펄스> <현재조향각도>"
//         21번 = 왼쪽 모터컨트롤러 (PID 피드백에 사용), 20번 = 모니터링 전용
//         현재조향각도 = DC 가변저항 필터값을 각도로 환산 (부호 포함, 좌측 음수/우측 양수)
//
//  인휠 모터 : iw_0708_pidtuning.ino의 FF + PID 이식 (출력 PWM 10번)
//    - 21번 홀센서 펄스로 루프를 닫음
//    - 피드포워드(FF) + PID 보정, 포화 방향 적분 중단(anti-windup), I_MAX 클램프
//    - 목표값 변경 시 적분 리셋, 측정값 기준 미분
//  제어주기 : 인휠/조향 모두 20ms 공유 (CONTROL_WINDOW_MS)
//
//  DC(조향)모터 : MD20A로 제어 (DIR+PWM), 가변저항(A0, dc_0702_pd.ino 기준)
//    피드백으로 목표 위치를 PD 제어로 추종. 좌/우 하드리밋도 dc_0702_pd.ino
//    실측값(933/751) 반영.
//
//  가변저항 노이즈 필터(readPotFiltered) : 9회 샘플링 후 정렬, 중앙 3개 평균
//    (RC카 통합 제어 코드에서 이식). DC 조향 가변저항 읽기에 적용,
//    조향 PD 제어와 초기 위치 확인도 이 필터값 기준으로 동작.
//
//  리니어(브레이크) 모터 : MD20A로 제어, 위치 피드백 없이(A14는 형식상 정의만)
//    시리얼 3번째 값의 부호/크기로 열린루프 타이밍 구동
//    - 양수(0~255) 입력 → DIR HIGH, 해당 크기로 2초간 구동 → 실측: 로드 나옴(전진/신장)
//    - 음수(-255~0) 입력 → DIR LOW,  |값| 크기로 2초간 구동 → 실측: 로드 들어감(후퇴/수축)
//    - 0 입력 → 즉시 정지
//
//  [실측 확인, 2026-07-08] 조향각도 부호에 따른 DC조향모터 제어방향 일치 확인.
//    리니어는 양수 입력 시 나오고(신장), 음수 입력 시 들어감(수축).
//
//  ※ 현재 ESTOP_ENABLED = false 로 임시 비활성화 상태 (아래 조건들은 판정만 되고 무시됨)
//  E-stop 조건 (아래 중 하나라도 충족 시 발동)
//    ① 40번 핀 단락 (LOW)
//    ② 5초 이상 입력 없음
//    (입력 형식 오류는 이제 e-stop이 아니라 해당 줄 무시로 처리)
//
//  E-stop 동작
//    - 조향(DC)모터 PWM = 0, 인휠모터 PWM = 0
//    - 리니어(브레이크) 모터 최고출력으로 체결 방향 구동 (MD20A), 진행 중이던 타이밍 구동은 취소
//    - 출력값을 999 999 999 로 강제 표시
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN20 = 20;   // 모니터링 전용
const uint8_t HALL_PIN21 = 21;   // 왼쪽 모터컨트롤러 (PID 피드백에 사용)

// --- 인휠 주행 PWM ---
const uint8_t INWHEEL_PWM_PIN = 10;

// --- DC 조향모터 (MD20A) ---
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A0;          // 조향 위치 가변저항 (dc_0702_pd.ino 기준)

// --- 리니어(브레이크)모터 (MD20A) ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;
const uint8_t LINEAR_POT_PIN = A14;     // 형식상 정의만, 실제 제어에는 미사용 (열린루프 타이밍 구동)

// --- E-stop ---
const uint8_t ESTOP_PIN = 40;          // INPUT_PULLUP, 단락(LOW) 시 e-stop
const bool ESTOP_ENABLED = false;      // TODO: 임시 비활성화, 테스트 끝나면 true로 복구


// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long INPUT_TIMEOUT_MS = 5000;   // 5초 이상 무입력 → e-stop


// ================= 공통 제어주기 =================
// 인휠/조향/브레이크 제어 루프가 모두 공유하는 주기
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= 가변저항 노이즈 필터 =================
const int POT_SAMPLES = 9;


// ================= 인휠 PID (iw_0708_pidtuning.ino 실측 튜닝값) =================
float kp = 20.0;
float ki = 2.0;
float kd = 2.0;

const float I_MAX = 100.0;     // 적분 클램프 값

// 피드포워드 (실측 정적 맵: 데드존 ≈60 PWM, 25펄스 ≈ 220 PWM)
// ff = FF_DEADZONE + FF_GAIN * wheelREF  (wheelREF=0이면 0)
const float FF_DEADZONE = 60.0;
const float FF_GAIN     = 6.4;


// ================= 조향 PD =================
#define KP_S   6.0f
#define KD_S   0.1f
const int  STEER_TOLERANCE = 5;
const int  STEER_MIN_PWM   = 30;
const int  STEER_MAX_PWM   = 255;
// 조향 루프 주기는 공통 제어주기(CONTROL_WINDOW_MS, 20ms) 사용

#define DIR_CW   HIGH
#define DIR_CCW  LOW

// --- 조향각도(도) ↔ 포텐셔미터(raw) 변환 ---
// dc_0701_potential.ino / dc_0702_pd.ino 로 측정한 좌/우 끝값, 재측정 시 갱신
const int STEER_POT_LEFT   = 1023;   // angle = -STEER_ANGLE_MAX 에 대응하는 raw값
const int STEER_POT_RIGHT  = 732;    // angle = +STEER_ANGLE_MAX 에 대응하는 raw값
const int STEER_ANGLE_MAX  =  40;
const int STEER_ANGLE_MIN  = -STEER_ANGLE_MAX;


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
int  steer_angle_cmd  = 0;
int  steer_target_pos = 512;
int  steer_prev_pos   = 0;
int  steer_cur_pos    = 512;   // 최근 필터값 (출력용 각도 환산에 사용)
unsigned long steer_t = 0;


// ================= 브레이크(리니어) 상태 =================
int  brake_cmd     = 0;       // 입력받은 브레이크출력 (-BRAKE_MAX ~ BRAKE_MAX, 부호=방향)
int  brake_output  = 0;       // 현재 구동 중인 출력 크기 (0이면 정지)
bool linear_running = false;  // LINEAR_RUN_MS 동안 구동 중인지 여부
unsigned long linear_start_t = 0;


// ================= 모드 (4번째 입력값, 현재 파싱만 하고 미사용) =================
int mode_cmd = 0;   // TODO: 1=e-stop 발동, 2=가변저항값 확인 모드 등 추후 구현


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
int  angleToPot(int angle);
int  potToAngle(int pot);
void startLinear(int cmd);
int  readPotFiltered(uint8_t pin);
bool isValidNumber(const char* s);
void handleLine(char* line);
void pollSerial();
void updateWheel(unsigned long now);
void updateSteer(unsigned long now);
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

  // 리니어(브레이크) 최고출력으로 체결 방향 강제 구동, 진행 중이던 타이밍 구동은 취소
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


// ================= 가변저항 필터 읽기 (최빈값) =================
// 9샘플 정렬 후 가장 많이 나온 값을 '그대로' 반환 (평균 없음, 나머지는 버림)
// 최빈값이 여러 개면 중앙값(buf[POT_SAMPLES/2])에 가까운 쪽을 선택
int readPotFiltered(uint8_t pin) {
  int buf[POT_SAMPLES];
  for (int i = 0; i < POT_SAMPLES; i++) buf[i] = analogRead(pin);
  for (int i = 1; i < POT_SAMPLES; i++) {
    int key = buf[i];
    int j = i - 1;
    while (j >= 0 && buf[j] > key) { buf[j + 1] = buf[j]; j--; }
    buf[j + 1] = key;
  }

  int median    = buf[POT_SAMPLES / 2];
  int bestVal   = median;
  int bestCount = 0;
  int i = 0;
  while (i < POT_SAMPLES) {
    int val = buf[i];
    int count = 0;
    while (i < POT_SAMPLES && buf[i] == val) { count++; i++; }   // 정렬됐으므로 같은 값은 연속됨
    if (count > bestCount ||
        (count == bestCount && abs(val - median) < abs(bestVal - median))) {
      bestVal   = val;
      bestCount = count;
    }
  }
  return bestVal;
}


// ================= 조향각도 → 포텐셔미터 raw값 =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, STEER_POT_LEFT, STEER_POT_RIGHT);
}

// ================= 포텐셔미터 raw값 → 조향각도 (angleToPot의 역변환) =================
// 리밋 밖 raw값은 그대로 외삽되어 ±STEER_ANGLE_MAX 밖 각도로 표시될 수 있음 (실위치 확인용)
int potToAngle(int pot) {
  return map(pot, STEER_POT_LEFT, STEER_POT_RIGHT, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
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

  // 조향
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  // 리니어(브레이크) : A14는 형식상 정의만, pinMode/analogRead 미사용
  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, DIR_CW);
  linearStop();

  // E-stop
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  steer_target_pos = readPotFiltered(DC_POT_PIN);   // 시작 시 현재 위치 유지
  steer_prev_pos   = steer_target_pos;
  steer_cur_pos    = steer_target_pos;

  unsigned long now = millis();
  wheel_t = steer_t = tele_t = now;
  lastInputTime = now;   // 부팅 직후 5초의 유예시간 부여
}


// ================= 입력 파서 =================
// "<target> <angle> <brake> <mode>" 형식(공백 구분). 형식이 안 맞으면 그냥 무시.
// 4번째 토큰(모드)은 현재 파싱만 하고 사용하지 않음 (없어도 됨).
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
  steer_target_pos = angleToPot(steer_angle_cmd);
  brake_cmd        = constrain(brake, -BRAKE_MAX, BRAKE_MAX);
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


// ================= 조향 제어 (목표 위치 PD) =================
void updateSteer(unsigned long now) {
  if (now - steer_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_t = now;

  int cur = readPotFiltered(DC_POT_PIN);
  steer_cur_pos = cur;   // 출력용 각도 환산에 사용

  int err = steer_target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - steer_prev_pos) / dt);
  float output = p + d;
  steer_prev_pos = cur;

  if (abs(err) > STEER_TOLERANCE) {
    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    if (output > 0) dcCW(spd); else dcCCW(spd);
  } else {
    dcStop();
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
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  int angle_out;
  if (estop_active) {
    pulse20_out = 999;
    pulse21_out = 999;
    angle_out   = 999;
  } else {
    pulse20_out = wheel_speed20;  // 모니터링
    pulse21_out = wheel_speed21;  // 왼쪽 (PID 피드백)
    angle_out   = potToAngle(steer_cur_pos);  // 현재 조향각도 (부호 포함)
  }

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
  bool timeoutEstop = (now - lastInputTime > INPUT_TIMEOUT_MS);
  estop_active = ESTOP_ENABLED && (pinEstop || timeoutEstop);

  if (estop_active) {
    applyEstop();
  } else {
    updateWheel(now);
    updateSteer(now);
    updateBrake(now);
  }

  sendOutput(now);
}
