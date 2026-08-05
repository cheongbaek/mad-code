// ============================================================
//  리니어(브레이크)모터 위치 PID 제어 테스트 (Arduino Mega 2560) - 0727 버전
//  linear_0718.ino(핀/방향, 열린루프 타이밍 구동)을 기반으로 브레이크 페달에 붙은
//  가변저항(A2) 피드백을 추가해 "페달 위치"를 목표로 하는 PID 위치제어로 바꾼 테스트 코드.
//  (구조는 dc_0702_pd.ino의 DC 조향 PD 위치제어와 동일 — DC모터 대신 리니어모터, A0 대신
//   A2를 쓸 뿐. 텔레메트리 출력은 iw_0708_pidtuning.ino와 동일하게 시리얼 플로터용 공백구분)
//
//  ★ 이번 테스트 목적: I값 폭주 방지(조건부 적분/클램프 등)가 전혀 없는 순수 PID로
//    P=1, I=0, D=0부터 시작해서 순차적으로 게인을 조정해보는 것. 아래 KP/KI/KD 3개
//    상수만 바꿔서 재업로드하면 된다. (적분은 그냥 누적만 함 — KI=0이면 사실상 미사용)
//
//  가변저항(A2) 원시값 읽기는 이미 확인됐다고 보고 별도 확인 로직은 넣지 않았다.
//  다만 페달 양 끝단의 실측 하드 리밋값은 아직 없으므로, dc_0702_pd.ino에 있던
//  하드리밋 페일세이프(RAW_LEFT_LIMIT/RAW_RIGHT_LIMIT)는 이 코드에 없다.
//  페달을 끝까지 밀어붙이는 테스트는 반드시 사람이 지켜보면서 진행할 것.
//
//  입력 : "<목표 raw>"  (정수 0~1023, A2 원시값 그대로 — 각도/퍼센트 환산 없음. 개행 종료)
//         - 범위 밖/숫자 아닌 줄은 무시
//  동작 : |오차| <= TOLERANCE(3) 상태가 SETTLE_MS(0.5초) 이상 지속되면 "도달"로 판정,
//         모터 정지 후 대기 상태로 전환 (dc_0702_pd.ino와 동일한 상태기계).
//         새 목표가 들어오면 적분을 리셋하고 제어를 재개한다.
//  출력 : 제어주기(20ms)마다 "목표 현재값 오차 PWM" 공백구분 4필드
//         (Arduino IDE 시리얼 플로터로 바로 그래프 확인 가능)
// ============================================================


// ================= 핀 정의 (linear_0718.ino와 동일) =================
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;
const uint8_t BRAKE_POT_PIN  = A2;

#define DIR_CW   HIGH   // 브레이크출력 양수 방향 (linear_0718.ino와 동일 매핑)
#define DIR_CCW  LOW    // 브레이크출력 음수 방향
// ※ 위 부호와 실제 페달 압박 방향이 반대로 나오면 DIR_CW/DIR_CCW 값만 서로 바꿀 것


// ================= PID 게인 (여기서 조절 — P=1, I=0, D=0부터 순차 테스트) =================
float KP = 1.0f;
float KI = 0.0f;
float KD = 0.0f;


// ================= PWM 상한/하한 =================
// BRAKE_MIN_PWM: 모터가 실제로 움직이기 시작하는 최소 PWM(정지마찰 극복). 아직 실측 전
// 임시값이니 반드시 실측 후 조정할 것 — 너무 낮으면 오차가 남아도 모터가 안 움직이는
// 죽은 영역이 생기고, 너무 높으면 목표 근처에서 오버슈트/진동이 심해진다.
const int BRAKE_MIN_PWM = 30;
const int BRAKE_MAX_PWM = 255;


// ================= 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;   // 20ms — 이 프로젝트 PID 코드들과 동일 관례


// ================= 도달 판정 =================
const int TOLERANCE = 3;
const unsigned long SETTLE_MS = 500;


// ================= 상태 =================
enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState state = ST_SETTLED;   // 부팅 직후: 목표 입력 전이므로 대기 상태

int   target_pos = 512;   // 목표 raw값 (A2, 0~1023) — 시리얼로 갱신
int   prev_pos   = 0;     // 미분항 계산용 이전 raw값
float integral   = 0;     // 순수 누적(조건부적분/클램프 없음 — 요청대로 KI=0으로 시작)
unsigned long win_t = 0;

bool settleTimerRunning = false;
unsigned long settleStart = 0;

char rxBuf[16];
uint8_t rxLen = 0;


// ================= 모터 출력 =================
void linearStop()     { analogWrite(LINEAR_PWM_PIN, 0); }
void linearCW(int p)  { digitalWrite(LINEAR_DIR_PIN, DIR_CW);  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }
void linearCCW(int p) { digitalWrite(LINEAR_DIR_PIN, DIR_CCW); analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255)); }


// ================= 입력 형식 검사 (부호 없는 정수, 0~1023) =================
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t i = 0; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}


// ================= setup =================
void setup() {
  Serial.begin(115200);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  pinMode(BRAKE_POT_PIN, INPUT);
  linearStop();

  target_pos = analogRead(BRAKE_POT_PIN);   // 시작 시 현재 위치를 목표로 유지 (전원 인가 즉시 급동작 방지)
  prev_pos   = target_pos;
  win_t = millis();
}


// ================= 시리얼 입력 처리 =================
// "<목표 raw>" 정수 1개(0~1023). 형식 오류/범위 밖은 무시.
void handleLine(char* line) {
  if (!isValidNumber(line)) return;

  int v = atoi(line);
  if (v < 0 || v > 1023) return;

  target_pos = v;
  integral = 0;               // 새 목표 -> 이전 목표의 적분 잔재 제거
  settleTimerRunning = false;
  state = ST_ACTIVE;          // 새 목표 입력 -> 제어 재개
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


// ================= PID 위치제어 (순수 PID — 적분 폭주방지 없음) =================
void updateBrake(unsigned long now) {
  if (now - win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  win_t = now;

  int cur = analogRead(BRAKE_POT_PIN);
  int err = target_pos - cur;
  int pwm = 0;   // 텔레메트리용(정지 중이면 0)

  if (state == ST_SETTLED) {
    // ── 대기 상태: 가변저항 값이 흔들려도 무시하고 정지 유지 ──
    linearStop();
    prev_pos = cur;   // 미분항 기준점은 계속 갱신(대기 해제 직후 미분 튐 방지)
  } else {
    // ── 순수 PID: P + I(그냥 누적) + D, anti-windup 없음 ──
    float p = KP * (float)err;
    integral += (float)err * dt;
    float i_term = KI * integral;
    float d = -KD * ((float)(cur - prev_pos) / dt);
    float output = p + i_term + d;
    prev_pos = cur;

    if (abs(err) <= TOLERANCE) {
      linearStop();
      if (!settleTimerRunning) {
        settleTimerRunning = true;
        settleStart = now;
      } else if (now - settleStart >= SETTLE_MS) {
        state = ST_SETTLED;   // 0.5초 이상 허용범위 유지 -> 도달 판정, 대기로 전환
      }
    } else {
      settleTimerRunning = false;   // 허용범위를 벗어나면 도달 판정 타이머 리셋
      pwm = constrain((int)fabs(output), BRAKE_MIN_PWM, BRAKE_MAX_PWM);
      if (output > 0) linearCW(pwm); else linearCCW(pwm);
    }
  }

  // ── 텔레메트리 (시리얼 플로터용, 매 제어주기) ──
  Serial.print(target_pos);
  Serial.print(' ');
  Serial.print(cur);
  Serial.print(' ');
  Serial.print(err);
  Serial.print(' ');
  Serial.println(pwm);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();
  updateBrake(now);
}
