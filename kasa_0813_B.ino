// ============================================================
//  kasa_0813_B.ino : B보드 — 조향(PD) + 제동(★가변저항 2단★) + 주행모드 스위치
//                    (Arduino Mega 2560)
//  kasa_0804_B.ino 기반. ★입출력 양식은 완전히 동일하다★
//  바뀐 것은 둘뿐이다 : ①브레이크를 가변저항 위치제어로, ②조향 영점 캘리브레이션 추가.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0813-1] 브레이크 : 엔코더 시간표 → ★가변저항 절대위치★ ★★
// ════════════════════════════════════════════════════════════════════════════
//    0804 는 리니어에 로터리 엔코더를 붙이고 "몇 ms 밀면 몇 카운트" 실측표로 시간을
//    환산해 밀었다. 그런데 증분 엔코더는 절대위치를 모르고, 실측에서 기본위치가
//    사이클마다 −37~−55 카운트씩 밀렸다(linear_0803_pot.ino 가 잰 미끄러짐).
//    그래서 매번 하드스톱까지 밀어 재영점을 잡아야 했고, 그 재영점이 조금만 어긋나도
//    이동량이 통째로 흔들렸다. 표 자체도 PWM 255 전용이라 힘을 바꾸면 전부 무효였다.
//
//    ★가변저항(A5)은 절대값이다★ — 재영점도, 드리프트 보정도, 시간표도 필요 없다.
//    raw 값 하나가 곧 위치다. 조향(DC모터 + A2 가변저항)과 같은 구조가 된다.
//    linear_0813_throttle.ino / linear_0813_auto.ino 로 검증한 방식을 그대로 옮겼다.
//
//    ▶ 단계 (BRAKE_POS[])
//        0 = 기본위치 : ★위치를 보지 않는다★ 무조건 들어가는 방향(REV)으로 1000ms.
//                       리니어는 안쪽으로 끝까지 들어가도 문제가 없고 자체 잠금이라,
//                       어디에 있든 이 시간이면 확실히 끝까지 들어간다.
//                       ★'값이 멎으면 하드스톱' 판정을 쓰지 않는 이유★ — 이미 끝에
//                       닿아 있으면 값이 안 변하는 것이 당연한데, 그것을 '모터가 안
//                       돈다'로 읽고 헛동작을 반복하다 두 판을 날렸다(0813 실측).
//        1 = 약한 제동 : raw 600 에 닿으면 정지
//        2 = 풀브레이킹 : raw 850 에 닿으면 정지
//    ▶ PWM 은 ★255 하나뿐★ 이다. 세기를 나누지 않는다.
//    ▶ 어떤 구동도 ★1초를 넘기지 않는다★ (BRAKE_MAX_MS) — 가변저항 선이 빠지면 값이
//      떠서 영영 목표에 못 닿는데, 그때 모터를 계속 돌리지 않기 위한 최후 방어다.
//    ▶ 도달 판정은 ★목표선을 정확히★ 본다(여유를 빼지 않는다). 잡음은 ★연속 2회★
//      확인으로 막는다(실측 잡음 ±10, 중앙값 9점 뒤).
//    ▶ ★[0813-3] 처짐 보충★ 1·2단 도달 뒤 값이 목표보다 BRAKE_SAG(10) 만큼 내려가면
//      그때만 다시 밀어 목표까지 올린다. ★올라간 쪽은 절대 건드리지 않는다★ —
//      사람이 발로 더 밟은 경우가 그것이고, 되돌리려 들면 사람과 힘겨루기가 된다.
//    ▶ E-stop = 2단 체결. 목표도 판정도 일반 이동과 같은 식을 쓴다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0813-2] 조향 영점 캘리브레이션 ('a' 한 줄) + EEPROM ★★
// ════════════════════════════════════════════════════════════════════════════
//    ★왜 필요한가★ 조향 가변저항의 중심값은 링키지를 만질 때마다 달라졌다(실측
//    C=469 → 473 → …). 그때마다 RAW_LEFT/RIGHT_LIMIT 를 고쳐 다시 굽는 것은 번거롭고,
//    현장에서는 아예 불가능하다.
//
//    ▶ 사용법 : ★핸들을 일자로 둔 상태에서★ 시리얼로 "a" 한 줄을 보낸다.
//    ▶ 동작 : CAL_WATCH_MS 동안 가변저항을 지켜보며 ★최빈값★ 을 잡는다(평균이 아니라
//      최빈값인 이유는 스파이크가 평균을 끌고 가기 때문이다. 잡음 ±10 을 CAL_BIN 폭으로
//      묶어 가장 표가 많이 몰린 칸의 중앙을 쓴다).
//    ▶ 그 값을 중심으로 ★+STEER_HALF_SPAN 을 좌측 최대, −STEER_HALF_SPAN 을 우측 최대★
//      로 잡는다(145 = 지금 코드값 684/394 의 반폭과 같다. 행정 폭은 그대로 두고
//      중심만 옮기는 것이다).
//    ▶ EEPROM 에 저장한다. 이후 재부팅하면 ★EEPROM 값이 코드값보다 우선★ 이다.
//      저장된 값이 없거나 깨졌으면(매직·체크섬 불일치) 코드값을 쓴다.
//    ▶ 캘리브레이션 중에는 조향·제동 모두 정지하고, 끝나면 그 자리를 목표로 잡아
//      대기 상태로 돌아간다(급조향 방지).
//    ※ E-stop 중에는 'a' 를 받지 않는다 — 그 상황에서 사람이 핸들을 잡고 있을 리 없다.
//
// ════════════════════════════════════════════════════════════════════════════
//  통신 (0804 와 ★완전히 동일★)
// ════════════════════════════════════════════════════════════════════════════
//    입력 : "<조향각>,<브레이크단계>"   예) "-12,0"  "x,2"
//             조향각  : −40~+40 (− 좌 / + 우), 또는 'x' = 힘빼기(릴리즈)
//             브레이크: 0/1/2
//           "a"      : ★신규★ 조향 영점 캘리브레이션 (위 [0813-2])
//    출력 : "P,<실측조향각>,<주행모드>"  (평상시) / "STOP" (e-stop 중)
//             주행모드 : 1 자율 / 0 수동조종
//
//  핀 (0804 와 동일)
//      A2      조향 가변저항          D6/D7   조향 DC DIR/PWM
//      A5      ★리니어 가변저항★     D8/D9   리니어 DIR/PWM
//      D5      주행모드 스위치        D12     E-stop (NC)
//      ※ D2/D3(리니어 엔코더)는 ★더 이상 쓰지 않는다★ — 관련 코드 전부 삭제
// ============================================================

#include <EEPROM.h>

// ================= 통신 =================
const unsigned long BAUD = 115200;
const unsigned long TELE_MS = 50;


// ================= 핀 =================
const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;
const uint8_t DC_POT_PIN = A2;    // 조향 가변저항

const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;
const uint8_t LINEAR_POT_PIN = A5;   // ★[0813-1] 리니어 가변저항★

const uint8_t ESTOP_PIN = 12;
const bool ESTOP_ENABLED = true;     // false 로 두면 핀 e-stop 비활성

const uint8_t MODE_PIN = 5;          // 주행모드 스위치 (D5 ── 스위치 ── GND)


// ================= 방향 규약 =================
#define DIR_CW   HIGH   // 조향 왼쪽 (raw 증가 방향)
#define DIR_CCW  LOW    // 조향 오른쪽 (raw 감소 방향)

// 리니어 : 정방향(밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= 조향 각도 범위 =================
const int STEER_ANGLE_MAX =  40;
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

// ★코드 기본값★ EEPROM 에 저장된 값이 있으면 그쪽이 우선한다([0813-2]).
const int DEF_RAW_LEFT_LIMIT  = 684;   // 왼쪽 끝 (하드 리밋)
const int DEF_RAW_RIGHT_LIMIT = 394;   // 오른쪽 끝 (하드 리밋)

// 캘리브레이션이 쓰는 반폭. 684/394 의 반폭(145)과 같게 두어 행정 크기를 보존한다.
const int STEER_HALF_SPAN = 145;

const int SAFETY_MARGIN = 10;   // 하드 리밋에서 안쪽으로 두는 여유(raw 카운트)

// ★런타임 값★ setup 에서 EEPROM 또는 코드값으로 채운다. 캘리브레이션이 갱신한다.
int raw_left_limit  = DEF_RAW_LEFT_LIMIT;
int raw_right_limit = DEF_RAW_RIGHT_LIMIT;
int pot_at_angle_min = 0;   // 각도 −40 에 대응하는 raw (왼쪽)
int pot_at_angle_max = 0;   // 각도 +40 에 대응하는 raw (오른쪽)
int raw_hi_limit = 0;       // 하드 리밋 게이팅용 (큰 쪽)
int raw_lo_limit = 0;       //                    (작은 쪽)

// 위 파생값을 한꺼번에 다시 계산한다(코드값·EEPROM·캘리브레이션 공통 경로).
void applySteerLimits() {
  pot_at_angle_min = raw_left_limit  - SAFETY_MARGIN;
  pot_at_angle_max = raw_right_limit + SAFETY_MARGIN;
  raw_hi_limit = (raw_left_limit > raw_right_limit) ? raw_left_limit  : raw_right_limit;
  raw_lo_limit = (raw_left_limit > raw_right_limit) ? raw_right_limit : raw_left_limit;
}


// ================= 조향 PD (0804 와 동일) =================
float KP_S = 6.0f;
float KD_S = 0.1f;
const int STEER_MIN_PWM = 110;
const int STEER_MAX_PWM = 255;
const unsigned long CONTROL_WINDOW_MS = 20;

const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;
int   lastPotMedian = 512;

const int STEER_TOLERANCE_ENTER = 3;
const int STEER_TOLERANCE_EXIT  = 6;
const unsigned long SETTLE_MS = 500;

enum CtrlState { ST_ACTIVE, ST_SETTLED };
CtrlState steer_state = ST_SETTLED;

int  steer_angle_cmd = 0;
int  target_pos = 512;
int  prev_pos   = 512;
bool settleTimerRunning = false;
unsigned long settleStart = 0;
unsigned long steer_win_t = 0;


// ================= ★[0813-1] 브레이크 : 가변저항 2단★ =================
const uint8_t BRAKE_NONE = 0;
const uint8_t BRAKE_SOFT = 1;
const uint8_t BRAKE_FULL = 2;
const uint8_t BRAKE_LEVEL_MAX = 2;

// 1단·2단의 목표 raw. ★고치는 곳은 여기뿐★ (0단은 위치를 보지 않는다)
const int BRAKE_POS_1 = 600;
const int BRAKE_POS_2 = 850;

const int BRAKE_PWM = 255;              // ★세기는 하나뿐★
const unsigned long BRAKE_HOME_MS = 1000;   // 0단 : 무조건 REV 로 이 시간
const unsigned long BRAKE_MAX_MS  = 1000;   // ★어떤 구동도 1초를 넘기지 않는다★

const uint8_t BRAKE_CONFIRM_N = 2;    // 연속 이만큼 성립해야 '도달'로 인정
const uint8_t LIN_MEDIAN_N    = 9;    // 리니어 가변저항 중앙값 점수 (홀수)

// 유효구간 밖으로 이만큼 나가면 즉시 정지 (기구 보호)
const int BRAKE_GUARD_HI = BRAKE_POS_2 + 60;

uint8_t brake_cmd_level = BRAKE_NONE;   // 상위에서 지시한 단계
uint8_t brake_level     = BRAKE_NONE;   // 지금 있다고 보는 단계
int     brake_output    = 0;            // 텔레메트리·진단용

enum LinState { LIN_IDLE, LIN_MOVE };
LinState lin_state = LIN_IDLE;
uint8_t  lin_tgt_level = BRAKE_NONE;
int      lin_target_raw = 0;
bool     lin_homing = false;      // 지금 구동이 '0단으로 무조건 밀기'인가
bool     lin_raw_up = false;      // 목표를 향해 raw 를 키우는 방향인가
unsigned long lin_end_t = 0;
uint8_t  lin_hit_n = 0;
int      lin_pot = 0;             // 최근 리니어 가변저항 중앙값


// ================= E-stop (0804 와 동일한 규약) =================
bool estop_active = false;
bool estop_latched = false;
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 500;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 500;
unsigned long estop_high_t = 0;
unsigned long estop_low_t  = 0;


// ================= 주행모드 스위치 =================
bool auto_mode = false;
uint8_t mode_pin_last = HIGH;
unsigned long mode_change_t = 0;
const unsigned long MODE_CONFIRM_MS = 50;


// ================= ★[0813-2] 캘리브레이션★ =================
const unsigned long CAL_WATCH_MS = 1200;   // 이 시간 동안 지켜보며 최빈값을 잡는다
const int  CAL_BIN      = 8;               // 최빈값 히스토그램 칸 폭 (잡음 ±10 을 묶는다)
const int  CAL_BIN_LO   = 200;             // 히스토그램 범위 (조향 pot 의 현실적 구간)
const int  CAL_BIN_HI   = 840;
const uint8_t CAL_BINS  = (CAL_BIN_HI - CAL_BIN_LO) / CAL_BIN + 1;

bool cal_running = false;
unsigned long cal_end_t = 0;
uint8_t cal_hist[CAL_BINS];

// EEPROM 배치 : [매직 2B][좌 2B][우 2B][체크섬 1B]
//   ★구조체를 쓰지 않는다★ 아두이노 IDE 는 함수 프로토타입을 파일 맨 위(구조체 정의보다
//   앞)에 자동 삽입한다. 그래서 사용자 정의 타입을 함수 인자에 쓰면 "does not name a
//   type" 컴파일 오류가 난다. 기본형만 주고받으면 그 문제가 원천적으로 없다.
const int  EE_ADDR_MAGIC = 0;
const int  EE_ADDR_LEFT  = 2;
const int  EE_ADDR_RIGHT = 4;
const int  EE_ADDR_SUM   = 6;
const uint16_t EE_MAGIC  = 0x4B31;   // 'K1' — 이 판의 형식임을 표시

uint8_t calChecksum(uint16_t magic, int16_t l, int16_t r) {
  return (uint8_t)(magic ^ (magic >> 8) ^ l ^ (l >> 8) ^ r ^ (r >> 8) ^ 0xA5);
}


// ================= 출력용 =================
unsigned long tele_t = 0;
char rxBuf[24];
uint8_t rxLen = 0;

// ★진단이 필요할 때만 true★ 출력 양식("P,..."/"STOP")을 지키기 위해 기본은 false.
//   '#' 로 시작하므로 파서는 무시한다(이 저장소의 관례).
const bool DEBUG_B = false;


// ================= 선언 =================
int  readPotMedian();
int  readLinPotMedian();
int  smoothPot(int med);
int  angleToPot(int angle);
int  potToAngle(int raw);
void releaseSteer();
void updateSteer(unsigned long now);
void updateBrake(unsigned long now);
void updateLinear(unsigned long now);
void startLinMove(uint8_t lv, unsigned long now);
void updateEstop(unsigned long now);
void applyEstop(unsigned long now);
bool isValidNumber(const char* s);
bool isReleaseToken(const char* s);
bool isCalToken(const char* s);
void handleLine(char* line);
void pollSerial();
void updateMode(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();
void startCal(unsigned long now);
void updateCal(unsigned long now);
bool loadCal();
void saveCal();


// ================= 모터 출력 =================
void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, constrain(p, 0, 255)); }

void linearStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
  brake_output = 0;
}

// ★DIR 을 먼저 쓰고 PWM 을 나중에 쓴다★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 든다.
void linearDrive(uint8_t dir, int pwm) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  brake_output = constrain(pwm, 0, 255);
  analogWrite(LINEAR_PWM_PIN, brake_output);
}


// ================= 가변저항 읽기 =================
// 삽입정렬 중앙값 — N 이 작아 이게 제일 싸다. 스파이크에 사실상 면역이다.
static int medianOf(uint8_t pin, uint8_t n) {
  int s[9];                       // POT_MEDIAN_N / LIN_MEDIAN_N 중 큰 값
  if (n > 9) n = 9;
  for (uint8_t i = 0; i < n; i++) {
    int v = analogRead(pin);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) { s[j] = s[j - 1]; j--; }
    s[j] = v;
  }
  return s[n / 2];
}

int readPotMedian()    { return medianOf(DC_POT_PIN, POT_MEDIAN_N); }
int readLinPotMedian() { return medianOf(LINEAR_POT_PIN, LIN_MEDIAN_N); }

int smoothPot(int med) {
  if (steerAdcFiltered < 0) steerAdcFiltered = med;
  else steerAdcFiltered += STEER_ADC_SMOOTH_ALPHA * ((float)med - steerAdcFiltered);
  return (int)steerAdcFiltered;
}


// ================= 각도 <-> raw =================
int angleToPot(int angle) {
  angle = constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return map(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX, pot_at_angle_min, pot_at_angle_max);
}

int potToAngle(int raw) {
  int angle = map(raw, pot_at_angle_min, pot_at_angle_max, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}

int readSteerAngle() { return potToAngle(lastPotMedian); }


// ================= 조향 =================
void releaseSteer() {
  dcStop();
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
  target_pos = lastPotMedian;
  steer_angle_cmd = potToAngle(lastPotMedian);
}

void updateSteer(unsigned long now) {
  if (now - steer_win_t < CONTROL_WINDOW_MS) return;
  float dt = (now - steer_win_t) / 1000.0f;
  dt = constrain(dt, 0.005f, 0.2f);
  steer_win_t = now;

  int med = readPotMedian();
  lastPotMedian = med;
  int cur = smoothPot(med);

  if (steer_state == ST_SETTLED) {   // 대기 : 가변저항 변화 무시, 정지 유지
    dcStop();
    prev_pos = cur;
    return;
  }

  int err = target_pos - cur;
  float p = KP_S * (float)err;
  float d = -KD_S * ((float)(cur - prev_pos) / dt);
  float output = p + d;
  prev_pos = cur;

  int absErr = abs(err);

  if (absErr <= STEER_TOLERANCE_ENTER) {
    dcStop();
    if (!settleTimerRunning) {
      settleTimerRunning = true;
      settleStart = now;
    } else if (now - settleStart >= SETTLE_MS) {
      steer_state = ST_SETTLED;
    }
  } else if (absErr > STEER_TOLERANCE_EXIT) {
    settleTimerRunning = false;
    int spd = constrain((int)fabs(output), STEER_MIN_PWM, STEER_MAX_PWM);
    bool wantRawUp = (output > 0);

    // 하드 리밋 게이팅 : 파고드는 방향만 차단, 벗어나는 방향은 허용
    if (wantRawUp && med >= raw_hi_limit)       dcStop();
    else if (!wantRawUp && med <= raw_lo_limit) dcStop();
    else if (wantRawUp)                          dcCW(spd);
    else                                         dcCCW(spd);
  } else {
    dcStop();   // 죽은 영역 : 정지하되 타이머는 유지
  }
}


// ================= ★[0813-1] 브레이크 : 가변저항 2단★ =================
// 지금 위치가 목표에 닿았는가. ★일반 이동과 E-stop 이 같은 식을 쓴다★
//   ★[0813-3] 목표선을 '정확히' 본다★ 여유(±TOL)를 빼고 보면 도달 자리가 늘 목표 이상
//   (밟는 쪽) 또는 이하(놓는 쪽)가 되어, 아래 처짐 판정(목표−10)과 겹치지 않는다.
//   여유를 빼 두면 588 에 서고도 곧바로 '처졌다(<590)'가 되어 무한히 다시 미는 꼴이 된다.
//   잡음은 여유가 아니라 ★연속 확인(BRAKE_CONFIRM_N)★ 으로 막는다.
bool linReached(int raw) {
  return lin_raw_up ? (raw >= lin_target_raw) : (raw <= lin_target_raw);
}

// 단계 이동 시작. ★0단은 위치를 보지 않고 무조건 REV 로 BRAKE_HOME_MS★ (헤더 [0813-1])
void startLinMove(uint8_t lv, unsigned long now) {
  lin_tgt_level = lv;
  lin_hit_n = 0;

  if (lv == BRAKE_NONE) {
    lin_homing = true;
    lin_end_t = now + BRAKE_HOME_MS;
    linearDrive(LINEAR_REV, BRAKE_PWM);
    lin_state = LIN_MOVE;
    return;
  }

  lin_homing = false;
  lin_target_raw = (lv == BRAKE_FULL) ? BRAKE_POS_2 : BRAKE_POS_1;
  lin_pot = readLinPotMedian();
  int err = lin_target_raw - lin_pot;

  if (err == 0) {                    // 정확히 그 자리 — 구동할 것이 없다
    linearStop();
    lin_state = LIN_IDLE;
    brake_level = lv;
    return;
  }

  lin_raw_up = (err > 0);
  // ★밟을수록 raw 가 커진다★ 실측 일관(0813). 그래서 raw 를 키우려면 FWD 다.
  lin_end_t = now + BRAKE_MAX_MS;
  linearDrive(lin_raw_up ? LINEAR_FWD : LINEAR_REV, BRAKE_PWM);
  lin_state = LIN_MOVE;
}

// 구동 중 종료 판정. ★목표 도달 / 시간 상한 / 가드★ 셋뿐이다.
void updateLinear(unsigned long now) {
  if (lin_state != LIN_MOVE) return;

  lin_pot = readLinPotMedian();

  if (lin_pot > BRAKE_GUARD_HI) {          // 유효구간 이탈 — 무조건 끊는다
    linearStop();
    lin_state = LIN_IDLE;
    brake_level = lin_tgt_level;
    if (DEBUG_B) Serial.println(F("# LIN GUARD"));
    return;
  }

  if (!lin_homing) {
    // 잡음 한 방에 멈추지 않게 연속 BRAKE_CONFIRM_N 회 성립해야 인정한다
    if (linReached(lin_pot)) {
      if (++lin_hit_n >= BRAKE_CONFIRM_N) {
        linearStop();
        lin_state = LIN_IDLE;
        brake_level = lin_tgt_level;
        return;
      }
    } else {
      lin_hit_n = 0;
    }
  }
  // ★homing 중에는 위치를 보지 않는다★ 0단은 시간이 정지 조건 그 자체다.

  if ((long)(now - lin_end_t) >= 0) {
    linearStop();
    lin_state = LIN_IDLE;
    brake_level = lin_tgt_level;   // 시간이 다 됐으면 그 단계에 있다고 본다
  }
}

// 지시 단계와 현재 단계가 다르면 이동을 건다(구동 중에는 끝난 뒤에).
//
// ★[0813-3] 처짐 보충 : ★내려갈 때만★ 다시 민다★
//   1·2단에 도달한 뒤 유압·기구 처짐으로 값이 목표보다 BRAKE_SAG 만큼 내려가면 그때만
//   다시 밀어 목표까지 올린다. ★올라간 쪽은 절대 건드리지 않는다★ — 사람이 발로 더 밟은
//   경우가 그것이고, 되돌리려 들면 사람과 힘겨루기가 된다.
//   미는 것도 목표(600/850)에 닿는 즉시 끊는다 — 일반 이동과 같은 판정을 쓴다.
const int BRAKE_SAG = 10;   // 목표보다 이만큼 내려가면 다시 민다

void updateBrake(unsigned long now) {
  if (lin_state != LIN_IDLE) return;

  if (brake_cmd_level != brake_level) {
    startLinMove(brake_cmd_level, now);
    return;
  }

  // 같은 단계를 유지하는 중 — 0단은 처짐 개념이 없다(위치를 보지 않는다)
  if (brake_level == BRAKE_NONE) return;

  int tgt = (brake_level == BRAKE_FULL) ? BRAKE_POS_2 : BRAKE_POS_1;
  lin_pot = readLinPotMedian();
  if (lin_pot < tgt - BRAKE_SAG) {
    startLinMove(brake_level, now);   // 같은 목표로 다시 — 도달하면 즉시 끊긴다
  }
}


// ================= E-stop =================
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_active = false;
    estop_high_t = 0;
    estop_low_t  = 0;
    return;
  }

  bool open_now = (digitalRead(ESTOP_PIN) == HIGH);   // NC 개방 = e-stop 요청

  if (!estop_active) {
    estop_low_t = 0;
    if (!open_now) {
      estop_high_t = 0;
    } else if (estop_high_t == 0) {
      estop_high_t = now;
    } else if (now - estop_high_t >= ESTOP_TRIGGER_CONFIRM_MS) {
      estop_active = true;
      estop_high_t = 0;
    }
  } else {
    estop_high_t = 0;
    if (open_now) {
      estop_low_t = 0;
    } else if (estop_low_t == 0) {
      estop_low_t = now;
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_active = false;
      estop_low_t = 0;
    }
  }
}

// ★E-stop = 2단 체결★ 조향은 힘빼기, 브레이크는 최대.
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;

  if (!estop_latched) {
    estop_latched = true;
    cal_running = false;              // 캘리브레이션 중이었다면 취소
    if (lin_state == LIN_MOVE) {      // 진행 중이던 구동을 끊고 다시 건다
      linearStop();
      lin_state = LIN_IDLE;
    }
    brake_cmd_level = BRAKE_FULL;
    startLinMove(BRAKE_FULL, now);
  }
  brake_cmd_level = BRAKE_FULL;
  updateLinear(now);
}


// ================= ★[0813-2] 캘리브레이션★ =================
void startCal(unsigned long now) {
  cal_running = true;
  cal_end_t = now + CAL_WATCH_MS;
  for (uint8_t i = 0; i < CAL_BINS; i++) cal_hist[i] = 0;
  dcStop();
  linearStop();
  lin_state = LIN_IDLE;
  steer_state = ST_SETTLED;           // 지켜보는 동안 조향은 정지
  settleTimerRunning = false;
  // ★항상 찍는다★ 사람이 손으로 하는 일회성 조작이라 확인이 필요하다.
  //   '#' 로 시작하므로 상위 파서는 무시한다(arduino.py 의 parse_b 는 'P,' 로
  //   시작하지 않는 줄을 조용히 버린다) — 출력 양식은 깨지지 않는다.
  Serial.println(F("# CAL start (1.2s) — 핸들을 일자로 유지할 것"));
}

void updateCal(unsigned long now) {
  int raw = readPotMedian();
  lastPotMedian = raw;                // 텔레메트리는 계속 실측을 내보낸다

  if (raw >= CAL_BIN_LO && raw <= CAL_BIN_HI) {
    uint8_t b = (uint8_t)((raw - CAL_BIN_LO) / CAL_BIN);
    if (b < CAL_BINS && cal_hist[b] < 255) cal_hist[b]++;
  }

  if ((long)(now - cal_end_t) < 0) return;

  // ★최빈값 채택★ 평균이 아니라 최빈값인 이유 : 스파이크 하나가 평균을 끌고 간다.
  uint8_t best = 0;
  uint8_t bestCnt = 0;
  for (uint8_t i = 0; i < CAL_BINS; i++) {
    if (cal_hist[i] > bestCnt) { bestCnt = cal_hist[i]; best = i; }
  }
  cal_running = false;

  if (bestCnt == 0) {                 // 유효 표본이 없다 — 그냥 되돌아간다
    Serial.println(F("# CAL FAIL — 유효 표본 없음 (배선·범위 확인). 기존값 유지"));
  } else {
    int center = CAL_BIN_LO + (int)best * CAL_BIN + CAL_BIN / 2;
    raw_left_limit  = center + STEER_HALF_SPAN;
    raw_right_limit = center - STEER_HALF_SPAN;
    applySteerLimits();
    saveCal();
    // ★결과는 항상 찍는다★ 이 한 줄이 '캘리브레이션이 실제로 됐는가'의 유일한 증거다.
    Serial.print(F("# CAL OK center="));  Serial.print(center);
    Serial.print(F(" L="));               Serial.print(raw_left_limit);
    Serial.print(F(" R="));               Serial.print(raw_right_limit);
    Serial.println(F(" (EEPROM 저장됨)"));
  }

  // 끝난 자리를 목표로 잡고 대기로 — 캘리브레이션 직후 급조향을 막는다
  int cur = readPotMedian();
  lastPotMedian    = cur;
  steerAdcFiltered = cur;
  target_pos = cur;
  prev_pos   = cur;
  steer_state = ST_SETTLED;
  settleTimerRunning = false;
}

// EEPROM 이 우선. 매직·체크섬이 맞을 때만 채택한다(깨진 값으로 조향하면 위험하다).
bool loadCal() {
  uint16_t magic;
  int16_t  l, r;
  uint8_t  sum;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  EEPROM.get(EE_ADDR_LEFT,  l);
  EEPROM.get(EE_ADDR_RIGHT, r);
  EEPROM.get(EE_ADDR_SUM,   sum);

  if (magic != EE_MAGIC) return false;
  if (sum != calChecksum(magic, l, r)) return false;
  if (l == r) return false;

  raw_left_limit  = l;
  raw_right_limit = r;
  return true;
}

void saveCal() {
  uint16_t magic = EE_MAGIC;
  int16_t  l = (int16_t)raw_left_limit;
  int16_t  r = (int16_t)raw_right_limit;
  uint8_t  sum = calChecksum(magic, l, r);
  // put 은 값이 같으면 쓰지 않는다(수명 보호)
  EEPROM.put(EE_ADDR_MAGIC, magic);
  EEPROM.put(EE_ADDR_LEFT,  l);
  EEPROM.put(EE_ADDR_RIGHT, r);
  EEPROM.put(EE_ADDR_SUM,   sum);
}


// ================= 입력 형식 검사 =================
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

bool isReleaseToken(const char* s) {
  return s && (s[0] == 'x' || s[0] == 'X') && s[1] == '\0';
}

bool isCalToken(const char* s) {
  return s && (s[0] == 'a' || s[0] == 'A') && s[1] == '\0';
}


// ================= 한 줄 처리 =================
void handleLine(char* line) {
  // ★[0813-2] "a" 한 줄 = 조향 영점 캘리브레이션★ (E-stop 중에는 받지 않는다)
  if (isCalToken(line)) {
    if (!estop_active) startCal(millis());
    return;
  }

  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 || !isValidNumber(tok2)) return;

  bool release = isReleaseToken(tok1);
  if (!release && !isValidNumber(tok1)) return;

  int brake = atoi(tok2);

  // e-stop 중에는 구동 명령(조향/브레이크) 미적용 → 리니어 재구동 방지
  if (estop_active) return;
  // 캘리브레이션 중에도 받지 않는다 — 핸들을 사람이 잡고 있는 상황이다
  if (cal_running) return;

  if (release) {
    releaseSteer();
  } else {
    steer_angle_cmd = constrain(atoi(tok1), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
    target_pos = angleToPot(steer_angle_cmd);
    settleTimerRunning = false;
    steer_state = ST_ACTIVE;
  }

  if (brake >= BRAKE_NONE && brake <= BRAKE_LEVEL_MAX) {
    brake_cmd_level = (uint8_t)brake;
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


// ================= 주행모드 스위치 =================
void updateMode(unsigned long now) {
  uint8_t lv = digitalRead(MODE_PIN);

  if (lv != mode_pin_last) {
    mode_pin_last = lv;
    mode_change_t = now;
  } else if (mode_change_t != 0 && now - mode_change_t >= MODE_CONFIRM_MS) {
    auto_mode = (lv == LOW);
    mode_change_t = 0;
  }
}


// ================= 출력 (0804 와 동일) =================
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("P,");
  Serial.print(readSteerAngle());
  Serial.print(',');
  Serial.println(auto_mode ? 1 : 0);
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
  pinMode(LINEAR_POT_PIN, INPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_active = false;
  estop_high_t = 0;
  estop_low_t  = 0;

  pinMode(MODE_PIN, INPUT_PULLUP);
  mode_pin_last = digitalRead(MODE_PIN);
  auto_mode     = (mode_pin_last == LOW);
  mode_change_t = 0;

  // ★[0813-2] EEPROM 이 코드값보다 우선★ 없거나 깨졌으면 코드값을 쓴다.
  bool fromEE = loadCal();
  applySteerLimits();
  if (DEBUG_B) {
    Serial.print(F("# LIMITS "));
    Serial.print(fromEE ? F("EEPROM ") : F("default "));
    Serial.print(raw_left_limit);
    Serial.print('/');
    Serial.println(raw_right_limit);
  }

  unsigned long now = millis();
  tele_t = now;

  // 브레이크 : 부팅 상태는 '단계 미상'이 아니라 0단으로 두고 대기한다.
  //   상위가 0을 보내도 아무 일이 없고, 1/2를 보내면 그때 움직인다.
  brake_level     = BRAKE_NONE;
  brake_cmd_level = BRAKE_NONE;
  lin_state       = LIN_IDLE;
  lin_pot         = readLinPotMedian();

  // 조향 PD : 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;

  // ★여기서 안내를 출력하지 않는다★ 출력 양식("P,..."/"STOP")을 그대로 지키기 위함.
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // 진행 중인 리니어 구동의 종료 판정 — e-stop 여부와 무관하게 매 루프
  updateLinear(now);

  // ★E-stop 판정은 pollSerial 보다 앞★ 발동 직후 들어온 줄이 한 번 먹는 틈을 없앤다
  updateEstop(now);

  pollSerial();

  // 주행모드 스위치는 e-stop 여부와 무관하게 항상 갱신 (보고 전용이므로 안전)
  updateMode(now);

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      estop_latched = false;
      // ★해제는 0단 복귀와 같은 작용★ 송신측이 0을 보내주기를 기다리지 않는다.
      if (lin_state == LIN_MOVE) {
        linearStop();
        lin_state = LIN_IDLE;
      }
      brake_cmd_level = BRAKE_NONE;
      brake_level     = BRAKE_FULL;   // 지금은 2단에 있으므로, 0단으로 내려가게 된다

      // 조향 PD 목표를 해제 시점의 현재 위치로 재동기화
      int cur = readPotMedian();
      lastPotMedian    = cur;
      steerAdcFiltered = cur;
      target_pos = cur;
      prev_pos   = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
    }

    if (cal_running) {
      updateCal(now);          // 캘리브레이션 중에는 조향·제동을 건드리지 않는다
    } else {
      updateBrake(now);
      updateSteer(now);
    }
  }

  sendOutput(now);
}
