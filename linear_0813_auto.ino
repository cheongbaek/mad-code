// ============================================================
//  linear_0813_auto.ino : 리니어(브레이크) ★단계 전이 시간 자동 실측★ — 2단 체제
//                         (Arduino Mega 2560, 가변저항 + 리니어모터만 사용)
//
//  ★ 무엇을 하는가 ★
//    "0단에서 2단까지 PWM 255 로 몇 ms 를 밀어야 하는가" 를 코드가 찾는다.
//    세 가지 전이(0→2, 0→1, 2→1)마다 시간을 10ms 씩 늘려 가며 시험하고, 목표에 처음
//    닿은 시간을 채택해 마지막에 표로 뽑고 끝난다.
//
//  ★★ [2026-08-13] 3단 → 2단, 재위치 폐지 ★★
//    믿을 수 있는 가변저항 값이 ★500 근처와 최댓값 850★ 뿐이라는 관측에 따라 단수를
//    줄였다. 그리고 그 결과 구조가 크게 단순해졌다:
//      · 0단 = 안쪽 하드스톱, 2단 = 바깥 하드스톱(850). ★둘 다 '끝까지 밀면 되는 자리'★
//      · 시험이 0→2, 0→1, 2→1 뿐이라 ★출발점은 0단 아니면 2단★ — 둘 다 하드스톱이다
//      · 그래서 ★재위치(PWM 70 미세조정)가 아예 필요 없다★. 1차 실측에서 재위치가
//        수렴하지 못해 1→2·1→3 이 통째로 FAIL 로 남았는데, 그 실패 지점이 사라졌다.
//
//  ★ 단계 위치 ★
//      0단 = ★코드 시작 후 안쪽으로 1초 밀어 실측한 값★ — 한 번 잡으면 끝까지 그대로다
//      1단 = 0단에서 (850 − 0단) 의 LEVEL1_NUM/LEVEL1_DEN (기본 3/5) 지점
//      2단 = 850 (고정 — 바깥 끝)
//    0단을 상수로 두지 않는 이유는 실측할 때마다 값이 달랐기 때문이다(428→397→390→310…).
//    시작할 때 한 번 직접 재면 그 변동이 통째로 사라진다.
//
//  ★ 한 번의 시행(trial) ★
//      1) 출발 단으로 이동 : 0단이면 REV 1초, 2단이면 FWD 1초 (판정 없이 고정 시간)
//      2) PWM 255 로 t ms 구동
//      3) 정지 후 SETTLE_MS 대기 → 위치 측정
//      4) 판정
//           미달(UNDER) → t 를 10ms 늘려 1) 부터 다시
//           도달(HIT)   → ★그 t 를 채택★
//           초과(OVER)  → ★직전 t 를 채택★ (재위치가 없으므로 되돌릴 수단이 없다.
//                         모자란 쪽이 안전한 쪽이다 — 브레이크를 덜 밟는다)
//
//  ★ 출력 ★
//    진행 중에는 '#' 로 시작하는 사람용 줄, 끝나면 ★탭 구분 표★ 를 '#' 없이 찍는다
//    (엑셀·시트에 그대로 붙여넣기 — linear_0803_speed.ino 와 같은 관례).
//
//  ★ 중단 ★ 시리얼로 아무 글자나 보내면 그 자리에서 모터를 끊고 지금까지의 표를 찍는다.
//
//  ★ 안전 ★
//    · 유효구간을 POT_GUARD 만큼 벗어나면 어떤 구동이든 즉시 끊는다.
//    · 한 시행의 구동시간은 MAX_TRIAL_MS 를 넘지 않는다(가변저항 단선·고착 대비).
//    · 끝까지 밀기는 ★판정 없이 고정 시간★ 이다. '값이 멎으면 하드스톱' 방식은 두 판
//      연속 실패했다 — 이미 끝에 닿아 있으면 값이 안 변하는 것이 당연한데 그것을
//      '모터가 안 돈다'로 읽고 헛동작을 반복했다(상수 HOME_MS 절 참고).
//    · ★E-stop 은 이 코드에 없다★ 지시대로 리니어와 가변저항만 쓴다.
//
//  ★ 핀 ★
//      A5     리니어 위치 가변저항
//      D8/D9  리니어 DIR / PWM
// ============================================================

// ================= 핀 =================
const uint8_t LINEAR_POT_PIN = A5;
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// 정방향(브레이크를 밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH

// ★밟을수록 raw 가 커진다★ 실측이 일관되게 그랬다(219→869, 390→855, 310→850).
//   배선을 뒤집었다면 이 값만 false 로 바꾸면 판정·방향이 전부 따라온다.
const bool PRESS_RAISES = true;


// ================= ★★ 단계 위치 ★★ =================
const int POT_AT_FULL = 850;      // 2단 = 최대 제동 (바깥 하드스톱, 고정)

// 1단 = 0단 + (850 − 0단) × NUM/DEN.  ★지시값 3/5★
//   ※ 0단이 310 근처면 1단은 310 + 540×3/5 = 634 가 된다.
//     이 두 값만 고치면 1단 위치가 통째로 따라온다.
const long LEVEL1_NUM = 3;
const long LEVEL1_DEN = 5;

const uint8_t LEVEL_MAX = 2;      // 0 / 1 / 2 세 자리

// 0단은 시작할 때 실측한다. 이 값은 ★측정 전 가드용 대략값★ 일 뿐이다.
const int POT_MIN_HINT = 250;


// ================= 판정·잡음 =================
// 허용오차는 ★잡음(±10)보다 커야★ 한다.
const int     POT_TOLERANCE = 12;
const uint8_t POT_MEDIAN_N  = 9;   // 반드시 홀수
const int     POT_GUARD     = 60;  // 유효구간 밖으로 이만큼 나가면 즉시 정지


// ================= 구동 =================
const int TRIAL_PWM = 255;   // ★시험 구동은 항상 255★ 이 값에 대한 시간표를 만드는 것이다
const int STOP_PWM  = 255;   // 하드스톱까지 밀 때도 같은 힘

const unsigned long STEP_MS      = 10;    // 시행마다 늘리는 시간
const unsigned long MAX_TRIAL_MS = 2000;  // 이 시간까지 못 닿으면 그 전이는 실패로 남긴다
const unsigned long SETTLE_MS    = 250;   // 끊은 뒤 관성이 멎기를 기다리는 시간

// ★★ [2026-08-13] 정지검출을 버리고 ★고정 시간★ 으로 간다 ★★
//   '값이 멎으면 하드스톱' 방식은 두 판 연속 실패했다. 안쪽 끝에 닿아 있으면 REV 로
//   밀어도 당연히 값이 안 변하는데, 코드는 그것을 '모터가 안 돈다'로 읽고 위로 올렸다
//   내리기를 반복하며 매번 한 번씩 더 처박았다. 판정 자체가 필요 없는 동작이었다.
//   ★리니어는 안쪽으로 끝까지 들어가도 문제가 없고, 자체 잠금이라 손으로도 안 움직인다★
//   그러니 그냥 정해진 시간만큼 밀면 된다 — 어디에 있든 끝까지 들어간다.
const unsigned long HOME_MS = 1000;   // 0단 : 항상 REV PWM 255 로 1초
const unsigned long TOP_MS  = 1000;   // 2단 : 항상 FWD PWM 255 로 1초 (바깥 끝)

// 0단 실측 : 다 들어간 뒤 이만큼 더 기다렸다가 그 값을 최솟값으로 채택한다.
const unsigned long MIN_SETTLE_MS = 800;

// 시행 출발 위치 확인 : 어긋나면 ★알리기만★ 한다.
//   고정 시간 구동이라 준비는 항상 같은 자리에서 끝난다. 그런데도 pot 값이 다르면
//   그것은 준비 실패가 아니라 ★pot 이 미끄러진 것★ 이므로, 시행을 버리는 대신
//   그 사실을 로그에 남기고 계속한다(start 값이 매 줄에 찍히므로 사후 판별이 된다).
const int START_TOL = 35;


// ================= 출력 =================
const unsigned long BAUD = 115200;
const unsigned long START_DELAY_MS = 1500;   // 배너를 읽고 손을 뗄 시간


// ================= 시험 목록 =================
const uint8_t N_TESTS = 3;
const uint8_t T_FROM[N_TESTS] = { 0, 0, 2 };
const uint8_t T_TO  [N_TESTS] = { 2, 1, 1 };

// 결과
enum { R_NONE, R_OK, R_PREV, R_OVER1, R_FAIL, R_ABORT };
unsigned long r_ms[N_TESTS];
int           r_landed[N_TESTS];
uint8_t       r_status[N_TESTS];

int  level0_ref = POT_MIN_HINT;   // 0단 기준 raw (시작할 때 실측해 덮어쓴다)
bool aborted = false;


// ================= 가변저항 =================
int readPotMedian() {
  int s[POT_MEDIAN_N];
  for (uint8_t i = 0; i < POT_MEDIAN_N; i++) {
    int v = analogRead(LINEAR_POT_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {   // 삽입정렬 — N 이 작아 이게 제일 싸다
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[POT_MEDIAN_N / 2];
}

// 단계 → 목표 raw
int levelTarget(uint8_t level) {
  if (level == 0) return level0_ref;
  if (level >= LEVEL_MAX) return POT_AT_FULL;
  long span = (long)POT_AT_FULL - (long)level0_ref;
  return (int)((long)level0_ref + span * LEVEL1_NUM / LEVEL1_DEN);
}

bool outOfGuard(int raw) {
  int lo = (level0_ref < POT_AT_FULL) ? level0_ref : POT_AT_FULL;
  int hi = (level0_ref < POT_AT_FULL) ? POT_AT_FULL : level0_ref;
  return (raw < lo - POT_GUARD) || (raw > hi + POT_GUARD);
}

// ★'밟기 진행량' 좌표★ 배선이 어느 쪽이든 '더 밟을수록 커지는' 값으로 바꾼다.
//   이 좌표에서는 밟기가 항상 +, 놓기가 항상 − 라 판정식이 하나로 끝난다.
long pressCoord(int raw) {
  return PRESS_RAISES ? (long)raw : -(long)raw;
}


// ================= 모터 =================
void motorStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
}

// ★DIR 을 먼저 쓰고 PWM 을 나중에 쓴다★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 든다.
void motorDrive(uint8_t dir, int pwm) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  analogWrite(LINEAR_PWM_PIN, constrain(pwm, 0, 255));
}

// 시리얼로 뭐든 들어오면 중단 신호로 본다
bool abortRequested() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    aborted = true;
    motorStop();
    Serial.println(F("# ABORT — 시리얼 입력으로 중단"));
    return true;
  }
  return false;
}

void settleWait(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) { /* 관성이 멎기를 기다린다 */ }
}

// 정해진 시간만큼 구동. 가드에 걸리거나 중단 요청이 오면 일찍 끊고 false.
bool driveMs(uint8_t dir, int pwm, unsigned long ms) {
  motorDrive(dir, pwm);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (outOfGuard(readPotMedian())) {
      motorStop();
      Serial.println(F("# GUARD — 유효구간 이탈로 구동 중단"));
      return false;
    }
    if (abortRequested()) return false;
  }
  motorStop();
  return true;
}


// ================= 끝까지 밀기 (고정 시간) =================
// ★판정이 없다★ 정해진 시간만큼 밀면 어디에 있든 끝에 닿는다. 위 상수 절 참고.
//   진단용으로 시작·끝 값만 남긴다 — pot 이 미끄러졌는지는 이 줄로 사후 판별한다.
bool driveToEnd(uint8_t dir, unsigned long ms) {
  int p_start = readPotMedian();
  if (!driveMs(dir, STOP_PWM, ms)) return false;
  settleWait(SETTLE_MS);

  Serial.print(F("#   END "));
  Serial.print(dir == LINEAR_REV ? F("REV ") : F("FWD "));
  Serial.print(p_start);
  Serial.print(F(" -> "));
  Serial.println(readPotMedian());
  return true;
}

// 출발 단으로 이동 — ★0단도 2단도 끝이라 시간만 주면 된다★
bool gotoLevel(uint8_t lv) {
  if (lv == 0)         return driveToEnd(LINEAR_REV, HOME_MS);   // 안쪽 끝
  if (lv == LEVEL_MAX) return driveToEnd(LINEAR_FWD, TOP_MS);    // 바깥 끝
  return false;   // 1단에서 출발하는 시험은 없다(있으면 여기서 걸린다)
}


// ================= 한 전이 시험 =================
void runTest(uint8_t i) {
  const uint8_t from = T_FROM[i], to = T_TO[i];
  const int target = levelTarget(to);
  const bool press = (to > from);
  const uint8_t dir = press ? LINEAR_FWD : LINEAR_REV;
  const long tgt_c = pressCoord(target);

  unsigned long t = STEP_MS;
  unsigned long prev_t = 0;
  int prev_pos = 0;

  Serial.print(F("# ===== ["));
  Serial.print(i + 1);
  Serial.print('/');
  Serial.print(N_TESTS);
  Serial.print(F("] "));
  Serial.print(from);
  Serial.print(F("->"));
  Serial.print(to);
  Serial.print(F("  목표 "));
  Serial.println(target);

  while (t <= MAX_TRIAL_MS) {
    // 출발 단으로 (고정 시간 — 끝까지 밀면 그만이다)
    if (!gotoLevel(from)) { r_status[i] = aborted ? R_ABORT : R_FAIL; return; }

    int start = readPotMedian();
    // ★어긋나도 시행을 버리지 않는다★ 준비는 시간으로 하므로 자리는 늘 같다.
    //   그런데도 값이 다르면 pot 이 미끄러진 것이니, 알리기만 하고 계속한다.
    if (abs(start - levelTarget(from)) > START_TOL) {
      Serial.print(F("#   (주의: 출발 "));
      Serial.print(start);
      Serial.print(F(" / 기대 "));
      Serial.print(levelTarget(from));
      Serial.println(F(" — pot 미끄러짐 의심)"));
    }

    if (!driveMs(dir, TRIAL_PWM, t)) { r_status[i] = aborted ? R_ABORT : R_FAIL; return; }
    settleWait(SETTLE_MS);
    int pos = readPotMedian();

    // ★밟기 좌표에서 판정★ 밟기든 놓기든 식이 하나다.
    long pos_c = pressCoord(pos);
    long err_c = press ? (pos_c - tgt_c) : (tgt_c - pos_c);   // + 면 지나침
    bool reached = (err_c >= -POT_TOLERANCE);
    bool over    = (err_c >  POT_TOLERANCE);

    Serial.print(F("#   t="));
    Serial.print(t);
    Serial.print(F("ms  "));
    Serial.print(start);
    Serial.print(F(" -> "));
    Serial.print(pos);
    Serial.print(F("  "));
    Serial.println(!reached ? F("UNDER") : (over ? F("OVER") : F("HIT")));

    if (reached) {
      if (!over) {
        r_ms[i] = t;  r_landed[i] = pos;  r_status[i] = R_OK;
        Serial.print(F("#   ★확정 "));
        Serial.print(t);
        Serial.println(F("ms"));
      } else if (prev_t > 0) {
        // ★재위치가 없으므로 지나친 값은 못 쓴다 — 직전 값을 택한다★
        //   모자란 쪽이 안전한 쪽이다(브레이크를 덜 밟는다).
        r_ms[i] = prev_t;  r_landed[i] = prev_pos;  r_status[i] = R_PREV;
        Serial.print(F("#   ★확정 "));
        Serial.print(prev_t);
        Serial.println(F("ms (초과 → 직전 값 채택)"));
      } else {
        // 첫 시행부터 지나쳤다 = STEP_MS 가 이 전이에 비해 너무 크다
        r_ms[i] = t;  r_landed[i] = pos;  r_status[i] = R_OVER1;
        Serial.print(F("#   ★확정 "));
        Serial.print(t);
        Serial.println(F("ms (첫 시행부터 초과 — STEP_MS 를 더 줄일 것)"));
      }
      return;
    }

    prev_t = t;
    prev_pos = pos;
    t += STEP_MS;
  }

  r_status[i] = R_FAIL;
  Serial.println(F("#   ★실패 : MAX_TRIAL_MS 까지 목표에 못 닿았다"));
}


// ================= 결과 표 =================
void printTable() {
  Serial.println();
  Serial.println(F("# ===== 결과 (탭 구분 — 시트에 그대로 붙여넣기) ====="));
  Serial.print(F("# 0단 실측 "));
  Serial.print(level0_ref);
  Serial.print(F(" / 1단 "));
  Serial.print(levelTarget(1));
  Serial.print(F(" / 2단 "));
  Serial.println(levelTarget(2));
  Serial.println(F("from\tto\tms\tlanded\ttarget\terr\tstatus"));

  for (uint8_t i = 0; i < N_TESTS; i++) {
    int target = levelTarget(T_TO[i]);
    Serial.print(T_FROM[i]);  Serial.print('\t');
    Serial.print(T_TO[i]);    Serial.print('\t');

    if (r_status[i] == R_OK || r_status[i] == R_PREV || r_status[i] == R_OVER1) {
      Serial.print(r_ms[i]);               Serial.print('\t');
      Serial.print(r_landed[i]);           Serial.print('\t');
      Serial.print(target);                Serial.print('\t');
      Serial.print(r_landed[i] - target);  Serial.print('\t');
    } else {
      Serial.print(F("-\t-\t"));
      Serial.print(target);
      Serial.print(F("\t-\t"));
    }

    switch (r_status[i]) {
      case R_OK:    Serial.println(F("OK"));    break;
      case R_PREV:  Serial.println(F("PREV"));  break;   // 초과 → 직전 값
      case R_OVER1: Serial.println(F("OVER1")); break;   // 첫 시행부터 초과
      case R_FAIL:  Serial.println(F("FAIL"));  break;
      case R_ABORT: Serial.println(F("ABORT")); break;
      default:      Serial.println(F("SKIP"));  break;
    }
  }
  Serial.println(F("# ===== 끝 ====="));
}


// ================= setup : 여기서 전부 진행하고 끝낸다 =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  motorStop();
  pinMode(LINEAR_POT_PIN, INPUT);

  for (uint8_t i = 0; i < N_TESTS; i++) {
    r_ms[i] = 0; r_landed[i] = 0; r_status[i] = R_NONE;
  }

  Serial.println();
  Serial.println(F("# ===== linear_0813_auto : 2단 전이 시간 자동 실측 ====="));
  Serial.print(F("# PWM "));
  Serial.print(TRIAL_PWM);
  Serial.print(F(" / 스텝 "));
  Serial.print(STEP_MS);
  Serial.print(F("ms / 허용오차 "));
  Serial.print(POT_TOLERANCE);
  Serial.println(F(" / 재위치 없음"));
  Serial.println(F("# ★모터가 곧 움직인다 — 기구에서 손을 뗄 것★ (아무 글자나 보내면 중단)"));
  settleWait(START_DELAY_MS);

  // ── 0단 실측 : 안쪽으로 1초 밀고, 한참 기다렸다가 그 값을 채택 ──
  //   ★여기서 한 번만 잡고 끝까지 그대로 쓴다★ 이후 다시 재지 않는다.
  Serial.println(F("# 0단(안쪽 끝)으로 이동 중…"));
  if (!driveToEnd(LINEAR_REV, HOME_MS)) { printTable(); return; }
  settleWait(MIN_SETTLE_MS);
  level0_ref = readPotMedian();

  Serial.print(F("# 0단 실측 = "));
  Serial.println(level0_ref);
  Serial.print(F("# 단계 목표 : 0단 "));
  Serial.print(levelTarget(0));
  Serial.print(F(" / 1단 "));
  Serial.print(levelTarget(1));
  Serial.print(F(" ("));
  Serial.print(LEVEL1_NUM);
  Serial.print('/');
  Serial.print(LEVEL1_DEN);
  Serial.print(F(") / 2단 "));
  Serial.println(levelTarget(2));

  // ── 세 가지 전이 ──
  for (uint8_t i = 0; i < N_TESTS && !aborted; i++) {
    runTest(i);
  }

  motorStop();
  printTable();
}


void loop() {
  // 시험은 setup() 에서 끝났다. 여기서는 아무것도 하지 않는다.
}
