// ============================================================
//  DC 조향모터 PD 위치제어 테스트 코드
//  (핀/구조는 kasa_0708_together.ino의 조향부를 참고)
//
//  입력 : 시리얼로 목표각도(정수, -40~40) 한 줄 입력 (개행 종료)
//  출력 : CONTROL_WINDOW_MS(20ms)마다 한 줄, benz_steer1.ino와 동일한 형식
//    "<명령각도> <목표 환산 실측값(raw)> <현재 가변저항 실측값(raw)> <PWM(왼쪽=음수, 오른쪽=양수)>"
//    - 목표 환산 실측값: 명령각도를 angleToPot()으로 환산한 raw 목표값 (target_pos)
//    - 현재 실측값: 가변저항 raw ADC 값 (필터 없이 그대로)
//    - PWM: 정지/하드리밋/도달(대기) 상태면 0, 구동 중이면 왼쪽=음수/오른쪽=양수
//      (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT 배선이라 dcCW=raw증가=왼쪽, dcCCW=raw감소=오른쪽)
//
//  동작 :
//    - 목표각도 -40/+40 은 실측 좌/우 하드 리밋(raw)보다
//      SAFETY_MARGIN(5)만큼 안쪽으로 매핑됨
//    - 현재 위치가 실측 하드 리밋에 도달하면 즉시 PWM 0 (페일세이프)
//    - |오차| <= STEER_TOLERANCE(3) 상태가 SETTLE_MS(0.5초) 이상
//      지속되면 "도달"로 판정, 모터 정지 후 대기 상태로 전환
//    - 대기 상태에서는 가변저항 값이 바뀌어도 무시하고 정지 유지,
//      새 각도값이 입력될 때만 제어를 재개함
// ============================================================

// ================= 핀 정의 =================
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A0;

#define DIR_CW   HIGH
#define DIR_CCW  LOW

// ================= 게인 (여기서 조절) =================
float KP_S = 6.0f;
float KD_S = 0.1f;

// ================= PWM 상한/하한 =================
const int STEER_MIN_PWM = 30;
const int STEER_MAX_PWM = 255;

// ================= 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;   // kasa_0708_together.ino와 동일 (PD게인 호환)

// ================= 입력 각도 범위 =================
const int ANGLE_MIN = -40;
const int ANGLE_MAX =  40;

// ================= 실측 좌/우 하드 리밋 (raw, 0~1023) =================
// dc_0701_potential.ino 로 재측정한 값
const int RAW_LEFT_LIMIT  = 1001;   // 왼쪽 끝 (하드 리밋)
const int RAW_RIGHT_LIMIT = 749;    // 오른쪽 끝 (하드 리밋)

// ================= 안전 여유값 =================
const int SAFETY_MARGIN = 5;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// -40도/+40도에 대응하는 목표 raw값 (하드 리밋보다 SAFETY_MARGIN만큼 안쪽)
const int POT_AT_ANGLE_MIN = RAW_LEFT_LIMIT  - SAFETY_MARGIN;   // 각도 -40 -> 이 raw값
const int POT_AT_ANGLE_MAX = RAW_RIGHT_LIMIT + SAFETY_MARGIN;   // 각도 +40 -> 이 raw값

// ================= 도달 판정 =================
const int STEER_TOLERANCE = 3;
const unsigned long SETTLE_MS = 500;   // 허용범위 유지 시간 -> 도달 판정


// ================= 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int  cmdAngle   = 0;    // 마지막으로 수신한 명령각도 (디버그 출력용)
int  target_pos = 512;
int  prev_pos   = 0;
unsigned long win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;

int lastPwmSigned = 0;   // 디버그 출력용: 왼쪽=음수, 오른쪽=양수

char rxBuf[16];
uint8_t rxLen = 0;


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }


// ================= 각도 -> raw 변환 =================
int angleToPot(int angle) {
  angle = constrain(angle, ANGLE_MIN, ANGLE_MAX);
  return map(angle, ANGLE_MIN, ANGLE_MAX, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX);
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


// ================= 시리얼 입력 처리 =================
void handleLine(char* line) {
  if (!isValidNumber(line)) return;      // 형식 오류 -> 무시

  int angle = atoi(line);
  if (angle < ANGLE_MIN || angle > ANGLE_MAX) return;   // 범위 밖 -> 무시

  cmdAngle    = angle;
  target_pos  = angleToPot(angle);
  settleTimerRunning = false;
  state = ST_ACTIVE;                      // 새 각도 입력 -> 제어 재개
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
      rxLen = 0;   // 버퍼 초과 -> 폐기
    }
  }
}


// ================= setup =================
void setup() {
  Serial.begin(115200);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  dcStop();

  target_pos = analogRead(DC_POT_PIN);   // 시작 시 현재 위치를 목표로 유지 (대기 상태)
  prev_pos   = target_pos;
  win_t = millis();
}


// ================= 조향 PD 제어 =================
void updateSteer(unsigned long now) {
  if (now - win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  win_t = now;

  int cur = analogRead(DC_POT_PIN);

  // ── 하드 리밋 도달 시 즉시 정지 (페일세이프) ──
  bool atHardLimit = (RAW_LEFT_LIMIT > RAW_RIGHT_LIMIT)
                        ? (cur >= RAW_LEFT_LIMIT || cur <= RAW_RIGHT_LIMIT)
                        : (cur <= RAW_LEFT_LIMIT || cur >= RAW_RIGHT_LIMIT);

  if (atHardLimit) {
    dcStop();
    lastPwmSigned = 0;
    prev_pos = cur;
  } else if (state == ST_SETTLED) {
    // ── 대기 상태: 가변저항 변화 무시, 정지 유지 ──
    dcStop();
    lastPwmSigned = 0;
    prev_pos = cur;
  } else {
    // ── PD 제어 ──
    int err = target_pos - cur;
    float p = KP_S * (float)err;
    float d = -KD_S * ((float)(cur - prev_pos) / dt);
    float output = p + d;
    prev_pos = cur;

    if (abs(err) <= STEER_TOLERANCE) {
      dcStop();
      lastPwmSigned = 0;
      if (!settleTimerRunning) {
        settleTimerRunning = true;
        settleStart = now;
      } else if (now - settleStart >= SETTLE_MS) {
        state = ST_SETTLED;   // 0.5초 이상 허용범위 유지 -> 도달 판정, 대기로 전환
      }
    } else {
      settleTimerRunning = false;   // 허용범위를 벗어나면 도달 판정 타이머 리셋
      int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
      if (output > 0) {
        dcCW(spd);
        lastPwmSigned = -spd;   // dcCW = raw 증가 방향 = 왼쪽 -> 음수로 표기
      } else {
        dcCCW(spd);
        lastPwmSigned = spd;    // dcCCW = raw 감소 방향 = 오른쪽 -> 양수로 표기
      }
    }
  }

  Serial.print(cmdAngle);
  Serial.print(' ');
  Serial.print(target_pos);
  Serial.print(' ');
  Serial.print(cur);
  Serial.print(' ');
  Serial.println(lastPwmSigned);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();
  updateSteer(now);
}
