// ============================================================
//  linear_0813_throttle.ino : 리니어(브레이크)를 ★가변저항 절대위치★ 로 6단 제어
//                             (Arduino Mega 2560, PID 없음)
//
//  ★ 왜 엔코더를 버렸는가 ★
//    linear_0813.ino 까지는 리니어에 붙인 로터리 엔코더로 위치를 셌다. 그런데
//    증분 엔코더는 절대위치를 모르고, 실측에서 기본위치가 사이클마다 −37~−55 카운트씩
//    밀렸다(linear_0803_pot.ino 가 잰 미끄러짐과 같은 값). 그래서 매번 하드스톱까지
//    밀어 재영점을 잡아야 했고, 그 재영점이 조금만 어긋나도 이동량이 통째로 흔들렸다.
//    ★가변저항은 절대값이다★ — 재영점도, 드리프트 보정도, '지금 몇 단인지' 기억도
//    필요 없다. raw 값 하나가 곧 위치다. 조향(DC모터 + A2 가변저항)과 같은 구조가 된다.
//
//  ★ 왜 PID 를 쓰지 않는가 ★
//    브레이크는 반응성이 최우선이고 러프해도 되는 대상이다(0804 계열이 내내 지켜 온
//    판단). 그래서 여기서는 ★목표를 향해 PWM 255 로 밀고, 닿으면 즉시 끊는다★ 뿐이다.
//    관성으로 얼마나 더 가는지(오버슛)를 이 코드로 먼저 재고, 그 다음에 시간차 보정을
//    얹는 순서로 간다. 처음부터 PD 를 넣으면 무엇이 오버슛이고 무엇이 게인 탓인지
//    구별할 수 없다.
//
//  ★ 단계 ★ 조향이 −40~+40 으로 촘촘한 것과 달리 제동은 둔하게 간다.
//    유효 구간(POT_AT_RELEASE ~ POT_AT_FULL)을 LEVEL_MAX 등분한 지점이 각 단이 된다.
//    [2026-08-13] 5단 → ★3단★ 으로 줄였다 — 0:390 1:545 2:700 3:855(465 raw ÷ 3 = 155,
//    나머지 없이 딱 떨어진다). ★고치는 곳은 위 두 상수 + LEVEL_MAX 뿐★ 나머지는
//    자동으로 따라온다.
//
//  ★★ [2026-08-13] 0단(놓음)은 다른 단과 다르게 움직인다 — 실제 있었던 버그 ★★
//    처음엔 0도 다른 단과 똑같이 '목표 raw − 현재 raw 의 부호로 방향을 정해' 움직였다.
//    그런데 현재 위치가 이미 목표보다 놓는 쪽으로 더 가 있으면(하드스톱 근처 잡음,
//    직전 놓기가 살짝 지나침 등) 그 부호가 뒤집혀 ★"0"을 눌렀는데 밟는 쪽으로 잠깐
//    움직이는★ 일이 있었다 — 같은 입력인데 한 번은 들어가고 한 번은 나오던 현상이
//    이것이다(첫 실측값 428 이 실제 하드스톱보다 안쪽 점이라 이 버그를 자주 냈다).
//    그래서 0은 목표를 계산하지 않고 ★무조건 들어가는 방향(REV)으로 HOME_MS 만큼만
//    민다★(하드스톱까지 밀어붙이는 재영점 동작 — linear_0813.ino 엔코더판의
//    '0 = 하드스톱까지' 와 같은 태도). 자세한 것은 startHome() 주석 참고.
//    1 이상은 여전히 방향을 부호로 정하고 목표 도달 즉시 정지한다(아래 그대로).
//
//  ★ 잡음 ★ 실측으로 값이 ±10 raw 튄다. 중앙값 9점 + 허용오차 12 + 연속 2회 확인의
//    세 가지로 막는다 — 셋이 한 세트라 따로 고치면 어긋난다(아래 '잡음 대책' 절).
//
//  ★★ [2026-08-13] 명령은 절대 씹히지 않는다 — BUSY 를 없앴다 ★★
//    '0→1 이 종종 안 움직인다'의 실제 원인은 구동·정착 중(ST_MOVE·ST_SETTLE) 들어온
//    명령을 "# BUSY" 로 조용히 버린 것이었다. 0(홈)만 해도 HOME_MS+SETTLE_MS ≈ 1.7초
//    동안 아무 것도 못 받았으니, 그 사이 다음 단을 누르면 흔적도 없이 사라졌다.
//    ★고침★ pollSerial() 은 이제 어떤 상태에서든 새 "0"~"LEVEL_MAX" 를 즉시 받아
//    startMove() 를 다시 부른다. startMove()/startMoveRaw()/startHome() 은 호출될
//    때마다 현재 raw 를 다시 읽어 목표·방향을 새로 계산하므로, 반대 방향으로 구동
//    중이었어도 그 자리에서 뒤집혀 새 목표로 향한다 — 별도의 취소 절차가 없어도 된다.
//    ★단 하나의 예외 : E-stop 체결 중(ST_ES_PUSH·ST_ES_HOLD)★ 은 여전히 무시한다.
//    안전상 당연하다(kasa_0804_B.ino 의 "E-stop 중엔 전부 무시"와 같은 태도). 이때는
//    "# ESTOP_HOLD" 로 찍혀 위 BUSY 와 구별된다 — 지금은 ESTOP_ENABLED=false 라
//    이 경로 자체가 걸리지 않는다.
//    ★그래도 안 움직이는 것처럼 보인다면★ HOME_MS(1500) 가 부족해 하드스톱 전에
//    멈추고, 그 자리가 우연히 다음 단 목표(±POT_TOLERANCE) 안에 들어 'ALREADY' 로
//    빠지는 경우가 남아 있다 — 그 순간 # ALREADY 가 찍히면 HOME_MS 를 늘릴 것.
//    ★부작용★ 명령을 아주 빠르게 연타하면 방향이 그때마다 뒤집힐 수 있다 — 이 코드는
//    방향전환 보호시간(REVERSE_DEADTIME 류)이 없다. 실제로 문제가 되면 추가할 것.
//
//  ★ E-stop = 최고단(LEVEL_MAX) 체결 ★ 목표도 도달판정도 일반 이동과 같은 식을 쓴다.
//    다른 것은 도달한 뒤 IDLE 로 돌아가지 않고 해제될 때까지 물고 있는 것뿐이다.
//
//  ★ 입력 (시리얼 모니터, 115200, ★개행 전송★) — 정확히 "0"~"5" 한 줄만 인정 ★
//    그 외(빈 줄, "12", "3 ", 문자 …)는 전부 무시한다. 구동 중의 입력도 무시한다.
//
//  ★ 출력 : REPORT_MS(5ms)마다 A5 가변저항 값 정수 하나 ★
//    다른 것은 아무것도 찍지 않는다(시리얼 플로터로 바로 볼 수 있게). 진단이 필요하면
//    VERBOSE 를 true 로 — '#' 로 시작하는 줄이 함께 나간다(이 저장소의 관례).
//
//  ★ 배선 방향은 코드가 알아서 맞춘다 ★
//    ★밟을 때 raw 가 커지는지 작아지는지 모르는 상태로 시작해도 된다★ —
//    POT_AT_RELEASE / POT_AT_FULL 두 값의 대소만 보고 구동 방향을 정하기 때문이다.
//    실제로 밟았는데 raw 가 줄어들면 두 상수를 서로 바꿔 넣으면(855 / 390) 그대로 맞는다.
//
//  ★ 안전 ★
//    · 유효구간을 POT_GUARD 만큼 벗어나면 즉시 끊는다.
//    · 어떤 이동도 MAX_DRIVE_MS 를 넘기지 않는다. ★가변저항 선이 빠지면 값이 떠서
//      영영 목표에 못 닿는데, 그때 모터를 계속 돌리지 않기 위한 최후 방어다★
//    · E-stop(D12, NC)은 그대로 살려 둔다. 발동하면 최고단(LEVEL_MAX)으로 민다.
//
//  ★ 핀 ★
//      A5     리니어 위치 가변저항 — 실사용 구간은 POT_AT_RELEASE ~ POT_AT_FULL 참고
//      D8/D9  리니어 DIR / PWM      (이전과 동일)
//      D12    E-stop (NC, 평상시 LOW / 개방 시 HIGH)
//      ※ D2/D3(엔코더 INT4/INT5)는 ★더 이상 쓰지 않는다★ — 관련 코드 전부 삭제
// ============================================================

// ================= 핀 =================
const uint8_t LINEAR_POT_PIN = A5;    // ★신규★ 리니어 위치 — 실사용 구간은 아래 참고
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

const uint8_t ESTOP_PIN = 12;
const bool ESTOP_ENABLED = false;     // [2026-08-13] D12 배선 확인 전까지 꺼둔다.
                                      //   뜬 핀(INPUT_PULLUP 만 걸린 고임피던스)은 PWM
                                      //   구동 중 EMI 로 흔들려 발동/해제를 반복할 수
                                      //   있다 — 실제로 "0만 눌렀는데 왔다갔다" 처럼
                                      //   보인 사례가 있었다(원인은 이거 하나가 아니었
                                      //   지만, 배선 전에는 꺼 두는 것이 안전하다).

// 정방향(브레이크를 밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= ★★ 유효 구간 (여기만 고친다) ★★ =================
// 실측(2026-08-13)으로 얻은 놓음~최대제동 구간이다. 처음엔 170~800 정도로 어림했으나
// 실제 하드스톱은 그 밖(855)에 있었다 — 아래 두 상수가 지금의 유일한 근거다.
// ★두 값의 대소가 곧 배선 방향이다★ — 밟았는데 raw 가 줄면 둘을 바꿔 넣으면 된다.
const int POT_AT_RELEASE = 390;   // 0단 (놓음)      [2026-08-13 재실측]
const int POT_AT_FULL    = 855;   // 최고단 (최대 제동) [2026-08-13 재실측]
//   ★처음 잰 428 이 틀렸던 이유★ 실제 하드스톱보다 밟는 쪽으로 더 안쪽의 점이었다.
//   그래서 0(홈)이 하드스톱까지 밀리면 raw 가 그 값 아래로 내려갔고, 다음 "0" 명령이
//   '목표보다 이미 낮다 → raw 를 올려야 한다(밟는 쪽)'로 방향을 거꾸로 계산했다 —
//   같은 "0" 인데 한 번은 들어가고 한 번은 나오던 버그의 실체다. startHome() 이
//   방향을 고정한 뒤로는 이 값이 다시 조금 어긋나도 방향은 더 이상 안 뒤집힌다.

const uint8_t LEVEL_MAX = 3;      // 0 ~ LEVEL_MAX 단계 (구간은 LEVEL_MAX 등분) [2026-08-13 5→3]
//   → 390 / 545 / 700 / 855  (465 raw ÷ 3 = 155, 나머지 없이 딱 떨어진다)


// ================= ★★ 잡음 대책 (실측 ±10 raw) ★★ =================
//  아래 셋은 ★한 세트★ 다. 하나만 고치면 다른 둘이 어긋난다.
//
//  ① 중앙값 점수를 9로 — 튀는 표본을 죽인다. 중앙값은 9개 중 5개가 오염되어야 흔들리므로
//     간헐적 스파이크에는 사실상 면역이다. 대신 읽는 데 약 1ms 가 걸리고, 그동안 리니어가
//     약 1.4 raw 움직인다(전 행정 430 raw 를 약 0.3초에 지난다). 그 정도는 감수한다.
//  ② 허용오차를 12로 — ★잡음보다 커야 한다★. 이보다 작으면 '도달'이 실제 움직임이
//     아니라 잡음 한 방으로 결정된다(목표보다 10 앞에서 멈춰 버린다). 반대로 너무 키우면
//     한 단(지금 155 raw)을 갉아먹으므로 잡음 바로 위에 붙인다.
//  ③ 연속 2회 확인 — 스파이크 한 개로는 멈추지 않게. 두 번이면 약 1ms 더 밀 뿐이다.
//
//  ※ bang-bang 이라 실제 정지 위치를 지배하는 것은 이 값들이 아니라 ★관성 오버슛★ 이다.
//    허용오차 12 는 '덜 밟는' 안전한 쪽 오차이고, 오버슛 보정은 실측 뒤에 따로 넣는다.
const int     POT_TOLERANCE = 12;
const uint8_t POT_CONFIRM_N = 2;

// 유효구간 밖으로 이만큼 더 나가면 즉시 정지 (기구 보호)
const int POT_GUARD = 60;


// ================= 구동 =================
// ★PWM 을 120 으로 낮췄다★ [2026-08-13] 255 는 관성 오버슛이 커서 허용오차(12)를
//   넘나들며 목표를 번번이 놓쳤다. 반응성보다 정확도가 급한 지금은 느리게 가는 쪽을
//   택한다 — 이 값도 실측 뒤 다시 조정할 초기값이다.
const int DRIVE_PWM = 120;

// 어떤 이동도 이 시간을 넘기지 않는다(가변저항 단선·목표 도달 실패 대비).
const unsigned long MAX_DRIVE_MS = 2000;

// PWM 을 끊은 뒤 관성이 멎기를 기다리는 시간. 이 뒤의 값이 '실제로 선 자리'다.
const unsigned long SETTLE_MS = 200;

// ================= ★★ 0단(놓음) 전용 — 시간 기반 홈 ★★ =================
// [2026-08-13] 위 '0단은 특별하다' 절 참고. 0은 pot 목표로 판정하지 않고 항상 이
// 시간만큼 들어가는 방향(REV)으로 민다 — 시작 위치가 어디든(밟혀 있든 이미 놓여
// 있든) 방향이 뒤집히지 않는다. ★실측 전 초기값★ 이다: 첫 시험에서 하드스톱에
// 못 미치면(멈춘 뒤에도 raw 가 계속 390 위쪽이면, VERBOSE 로 그 자리에서 "1"을
// 보냈을 때 # ALREADY 가 찍히면) 늘리고, 이미 다 간 뒤에도 한참 더 밀고 있는
// 느낌이면 줄인다. 위 '0→1 이 종종 안 움직인다' 절의 ② 가 바로 이 값이 부족한 경우다.
const unsigned long HOME_MS = 1500;

// 부팅 직후 0단(놓음)으로 보낼 것인가. 가변저항은 절대값이라 영점 잡기가 필요 없고,
// 이것은 순전히 '켜면 브레이크는 풀려 있어야 한다'는 안전 기본값이다.
const bool HOME_ON_BOOT = true;


// ================= E-stop (kasa_0804_B.ino 와 같은 규약) =================
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 500;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 500;


// ================= 가변저항 읽기 =================
// ★중앙값 필터★ 조향(readPotMedian)과 같은 방식. 점수는 위 '잡음 대책 ①' 참고.
const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수


// ================= 출력 =================
const unsigned long BAUD = 115200;
const unsigned long REPORT_MS = 5;
const bool VERBOSE = true;        // [2026-08-13] '0→1 안 움직임' 진단 위해 우선 켜 둔다.
                                   //   # BUSY / # ALREADY / # TIMEOUT / # GUARD 가 숫자
                                   //   줄 사이에 섞여 나간다 — 플로터에서 보기엔 지저분
                                   //   해도 해석에는 지장 없다. 원인이 잡히면 false 로.


// ================= 상태 =================
enum {
  ST_IDLE,        // 대기 : 0~5 를 받는다
  ST_MOVE,        // 목표를 향해 구동 중
  ST_SETTLE,      // 끊고 관성이 멎기를 기다리는 중
  ST_ES_PUSH,     // E-stop : 최고단으로 미는 중
  ST_ES_HOLD      // E-stop : 체결 완료, 해제 대기
};
uint8_t st = ST_IDLE;

uint8_t  cur_level = 0;           // 마지막으로 명령한 단계 (표시·판단용)
int      target_raw = 0;          // 이번 구동의 목표 raw
bool     raw_up = false;          // 목표를 향해 raw 를 ★키우는★ 방향인가
unsigned long phase_t = 0;        // 지금 상태로 들어온 시각
unsigned long drive_end = 0;      // 구동 시간 상한

bool estop_active = false;
bool es_driving   = false;        // E-stop 체결 구동을 이미 걸었는가
unsigned long estop_high_t = 0;
unsigned long estop_low_t  = 0;

unsigned long report_t = 0;
char rxBuf[8];
uint8_t rxLen = 0;

int pot_now = 0;                  // 최근 중앙값 (report 와 제어가 함께 쓴다)
uint8_t hit_n = 0;                // 도달 조건이 연속으로 성립한 횟수 (잡음 대책 ③)
bool homing = false;              // 지금 구동이 '0(놓음)으로 무조건 밀기'인가


void note(const char* msg) {
  if (VERBOSE) { Serial.print("# "); Serial.println(msg); }
}


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

// 단계 → 목표 raw. 유효구간을 LEVEL_MAX 등분한다(정수 나눗셈이라 나머지는 내림).
int levelTarget(uint8_t level) {
  if (level > LEVEL_MAX) level = LEVEL_MAX;
  long span = (long)POT_AT_FULL - (long)POT_AT_RELEASE;
  return (int)((long)POT_AT_RELEASE + span * (long)level / (long)LEVEL_MAX);
}

// 유효구간 밖(가드 포함)인가 — 방향과 무관하게 판정한다(배선 방향을 모르므로).
bool potOutOfGuard(int raw) {
  int lo = (POT_AT_RELEASE < POT_AT_FULL) ? POT_AT_RELEASE : POT_AT_FULL;
  int hi = (POT_AT_RELEASE < POT_AT_FULL) ? POT_AT_FULL    : POT_AT_RELEASE;
  return (raw < lo - POT_GUARD) || (raw > hi + POT_GUARD);
}


// ================= 리니어 출력 =================
void linearStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
}

// ★DIR 을 먼저 쓰고 PWM 을 나중에 쓴다★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 든다.
void linearDrive(uint8_t dir, int pwm) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  analogWrite(LINEAR_PWM_PIN, constrain(pwm, 0, 255));
}


// ================= 이동 =================
void enterSettle(unsigned long now) {
  linearStop();
  st = ST_SETTLE;
  phase_t = now;
}

// 도달 판정 — ★일반 이동과 E-stop 이 같은 식을 쓴다★ (E-stop = 최고단 체결과 동일하므로
//   판정이 갈리면 두 경로가 서로 다른 자리에 서게 된다).
bool reachedTarget(int raw) {
  return raw_up ? (raw >= target_raw - POT_TOLERANCE)
                : (raw <= target_raw + POT_TOLERANCE);
}

// 목표 raw 를 향해 민다. ★raw 를 키워야 하는 방향인지만 알면 배선 방향은 따라온다★
void startMoveRaw(int target, unsigned long now) {
  pot_now = readPotMedian();
  int err = target - pot_now;
  if (err > -POT_TOLERANCE && err < POT_TOLERANCE) {   // 이미 그 자리
    note("ALREADY");
    st = ST_IDLE;
    phase_t = now;
    return;
  }
  target_raw = target;
  raw_up = (err > 0);
  hit_n = 0;
  // raw 를 키우는 방향이 '밟는 쪽'인가 '놓는 쪽'인가는 두 상수의 대소가 말해 준다.
  //   (실측 : 발 뗌 390 → 최대로 밟음 855. 즉 밟을수록 커진다 = press_raises true)
  bool press_raises = (POT_AT_FULL > POT_AT_RELEASE);
  uint8_t dir = (raw_up == press_raises) ? LINEAR_FWD : LINEAR_REV;

  drive_end = now + MAX_DRIVE_MS;
  linearDrive(dir, DRIVE_PWM);
  st = ST_MOVE;
  phase_t = now;
}

// ★0(놓음) 전용★ 목표를 계산하지 않는다. 무조건 들어가는 방향(REV)으로 HOME_MS 만큼만
//   민다 — '0단은 특별하다' 절(파일 상단)의 버그(방향이 현재값에 따라 뒤집히던 것)를
//   구조적으로 없앤다. linear_0813.ino(엔코더판)의 '0 = 하드스톱까지' 와 같은 태도다.
void startHome(unsigned long now) {
  homing = true;
  hit_n = 0;
  drive_end = now + HOME_MS;
  linearDrive(LINEAR_REV, DRIVE_PWM);
  st = ST_MOVE;
  phase_t = now;
}

void startMove(uint8_t level, unsigned long now) {
  cur_level = level;
  if (level == 0) {
    startHome(now);
    return;
  }
  homing = false;
  startMoveRaw(levelTarget(level), now);
}

void updateMove(unsigned long now) {
  pot_now = readPotMedian();

  if (potOutOfGuard(pot_now)) {          // 유효구간을 벗어났다 — 무조건 끊는다
    note("GUARD");
    enterSettle(now);
    return;
  }

  if (!homing) {
    // ★닿으면 즉시 끊는다★ (PID 없음. 지나친 만큼은 관성이고, 그 양을 재는 것이 목적이다)
    //   단 ★연속 POT_CONFIRM_N 회★ 성립해야 인정한다 — 잡음 한 방에 멈추지 않게.
    if (reachedTarget(pot_now)) {
      if (++hit_n >= POT_CONFIRM_N) {
        enterSettle(now);
        return;
      }
    } else {
      hit_n = 0;
    }
  }
  // ★homing 중에는 위 목표판정을 보지 않는다★ 0(놓음)은 얼마나 밟혀 있었든 항상
  //   HOME_MS 를 다 쓴다 — pot 값으로 먼저 끊으면 하드스톱까지 못 갈 수 있다.

  if ((long)(now - drive_end) >= 0) {    // homing 은 이 시간 상한이 정지 조건 그 자체다
    if (!homing) note("TIMEOUT");
    enterSettle(now);
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


// ================= 시리얼 입력 ("0"~"5" 한 줄만) =================
void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = ch;
      else rxLen = sizeof(rxBuf);          // 넘치면 그 줄 전체를 버린다
      continue;
    }
    if (rxLen == 1 && rxBuf[0] >= '0' && rxBuf[0] <= ('0' + LEVEL_MAX)) {
      uint8_t lv = (uint8_t)(rxBuf[0] - '0');
      // ★[2026-08-13] 더 이상 BUSY 로 버리지 않는다★ (파일 상단 '명령은 절대
      //   씹히지 않는다' 절 참고) 구동·정착 중이어도 새 명령이 즉시 이긴다 —
      //   startMove() 가 현재 raw 를 다시 읽어 목표·방향을 그 자리에서 새로 잡는다.
      //   ★단 E-stop 체결 중(ST_ES_PUSH·ST_ES_HOLD)만은 안전상 계속 무시한다★
      //   (kasa_0804_B.ino 의 "E-stop 중엔 전부 무시"와 같은 태도).
      if (st == ST_ES_PUSH || st == ST_ES_HOLD) {
        note("ESTOP_HOLD");
      } else {
        startMove(lv, now);
      }
    }
    rxLen = 0;
  }
}


// ================= 출력 (가변저항 값 하나) =================
void report(unsigned long now) {
  if (now - report_t < REPORT_MS) return;
  report_t = now;
  Serial.println(pot_now);
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  pinMode(LINEAR_POT_PIN, INPUT);   // analogRead 는 없어도 되지만 명시한다
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  unsigned long now = millis();
  pot_now = readPotMedian();
  report_t = now;
  phase_t = now;
  st = ST_IDLE;

  // ★★ 부팅 경고 : E-stop 이 켜져 있는데 핀이 개방이면 알린다 ★★  [2026-08-13]
  //   D12 를 배선하지 않은 채 ESTOP_ENABLED 를 true 로 두면 고임피던스 입력이 떠서,
  //   PWM 255 로 도는 모터의 EMI 에 흔들려 발동/해제를 반복한다. 그러면 사람이 0 만
  //   넣어도 ★최고단(855) 밀기 ↔ 0단(390) 복귀★ 가 저절로 왕복한다. 실제로 겪은 증상이라
  //   VERBOSE 와 무관하게 한 번은 찍는다 — '#' 로 시작하므로 플로터는 무시한다.
  if (ESTOP_ENABLED && digitalRead(ESTOP_PIN) == HIGH) {
    Serial.println("# WARN: E-stop(D12) open at boot "
                   "— 배선하거나 ESTOP_ENABLED=false 로 둘 것");
  }

  // 켜면 브레이크는 풀려 있어야 한다 — 가변저항이 절대값이라 그냥 0단으로 보내면 된다
  // (엔코더 판처럼 '하드스톱까지 밀어 영점 잡기' 를 할 이유가 없다).
  if (HOME_ON_BOOT) startMove(0, now);
  note("BOOT");
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  updateEstop(now);

  // E-stop 은 어느 상태에서든 가로챈다. 목표는 언제나 최고단(최대 제동)이고, 방향은
  // ★따질 것 없이 밟는 쪽(FWD)★ 이다 — 최고단보다 더 밟혀 있을 수는 있어도 그 경우엔
  // 아래 도달 판정이 첫 틱에 성립해 구동 없이 끝난다.
  if (estop_active && st != ST_ES_PUSH && st != ST_ES_HOLD) {
    linearStop();
    target_raw = levelTarget(LEVEL_MAX);
    raw_up     = (POT_AT_FULL > POT_AT_RELEASE);   // 최고단 = 가장 밟은 쪽
    cur_level  = LEVEL_MAX;
    drive_end  = now + MAX_DRIVE_MS;
    es_driving = false;
    hit_n      = 0;
    st = ST_ES_PUSH;
    phase_t = now;
    note("ESTOP");
  }

  pollSerial(now);      // ★E-stop 판정 뒤에 둔다★ 발동한 루프의 명령이 먹는 틈을 없앤다

  switch (st) {
    case ST_IDLE:
      pot_now = readPotMedian();      // 대기 중에도 값은 계속 갱신한다(사람이 밟을 수 있다)
      break;

    case ST_MOVE:
      updateMove(now);
      break;

    case ST_SETTLE:
      pot_now = readPotMedian();
      if (now - phase_t >= SETTLE_MS) {
        st = ST_IDLE;
        phase_t = now;
      }
      break;

    case ST_ES_PUSH: {
      // ★E-stop 은 최고단 체결과 완전히 같다★ 목표·도달판정·허용오차를 일반 이동과
      //   공유한다. 다른 것은 도달한 뒤에 IDLE 로 돌아가지 않고 해제까지 물고 있는 것뿐.
      pot_now = readPotMedian();
      bool done = false;
      if (reachedTarget(pot_now)) {
        if (++hit_n >= POT_CONFIRM_N) done = true;
      } else {
        hit_n = 0;
      }
      if (done || potOutOfGuard(pot_now) || (long)(now - drive_end) >= 0) {
        linearStop();
        st = ST_ES_HOLD;
        phase_t = now;
      } else if (!es_driving) {
        linearDrive(LINEAR_FWD, DRIVE_PWM);   // ★밟는 방향은 언제나 FWD★
        es_driving = true;
      }
      break;
    }

    case ST_ES_HOLD:
      pot_now = readPotMedian();
      if (!estop_active) {
        note("ESTOP CLEAR");
        startMove(0, now);            // 해제 → 0단(놓음)으로 복귀
      }
      break;
  }

  report(now);
}
