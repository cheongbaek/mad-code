// ============================================================
//  linear_0813.ino : 리니어(브레이크) 단계 이동 ★구동시간 튜닝용★ (Arduino Mega 2560)
//
//  ★ 목적 ★
//    kasa_0804_B.ino 를 새로 쓰기 전에, 바뀐 전제 아래에서 "각 단계 이동에 몇 ms 를
//    주어야 하는가"를 손으로 찾는 코드다. 바뀐 전제는 셋이다:
//        ① 구동 PWM 255 → ★150★ (E-stop 발동 때만 255)
//        ② 2단(풀브레이킹) 위치 250 → ★300★ 카운트
//        ③ 1단 위치 = POS_FULL 의 1/3(83) → ★200★ 카운트 (비율이 아니라 절대값)
//
//    ★★ 0804 의 실측표는 여기서 전부 무효다 ★★ kasa_0804_B.ino 의 SPD_MS/SPD_CNT
//    (50ms=64 … 230ms=317카운트)는 PWM 255 에서 잰 값이고, 그 파일 자신이 "이 값을
//    바꾸면 표가 전부 무효가 된다"고 적어 두었다. 게다가 그 표는 선형이 아니다 —
//    50~140ms 는 0.9카운트/ms 인데 150ms 부터 1.8카운트/ms 로 빨라진다(마찰을 벗어나며
//    가속). 그래서 150/255 로 비례 축소하는 환산이 성립하지 않는다.
//    이 코드는 그 표를 다시 만드는 대신, ★단계 이동마다 시간을 하나씩 박아 놓고 눈으로
//    맞추는★ 방식을 쓴다. 필요한 값이 6개(0→1, 0→2, 1→2, 2→1, 1→0, 2→0)뿐이라
//    표를 새로 뜨는 것보다 이쪽이 빠르다.
//
//  ★ 입력 (시리얼 모니터, 115200, ★개행 전송★) — 정확히 "0"/"1"/"2" 한 줄만 인정 ★
//      0 = 기본위치  : 들어가는 방향으로 MOVE_MS[현재][0] 만큼 구동 (하드스톱까지)
//      1 = 1단       : 200카운트 지점
//      2 = 2단       : 300카운트 지점
//    그 외(빈 줄, "10", "0 ", 문자 …)는 전부 무시한다. 구동·정착 중의 입력도 무시한다.
//
//  ★ 출력 : REPORT_MS(5ms)마다 엔코더 값 정수 하나 ★
//    다른 것은 아무것도 찍지 않는다(시리얼 플로터로 바로 볼 수 있게). 진단 줄이 필요하면
//    VERBOSE 를 true 로 두면 '#' 로 시작하는 줄이 함께 나간다 — 숫자 줄 형식을 깨지 않는
//    이 저장소의 관례(linear_0803_pot.ino 의 `# ERR n`)를 따른 것이다.
//
//  ★★ 영점을 다시 잡지 않는다 ★★
//    0804 계열은 구동이 끝날 때마다 그 자리를 새 영점으로 삼았다(단계별 상대 이동 모델).
//    이 코드는 ★부팅 직후 한 번만★ 영점을 잡고 그 뒤로는 건드리지 않는다. 그래서 화면에
//    나오는 숫자가 곧 '기본위치에서 얼마나 밟혀 있는가'의 절대값이고,
//        0 → 200 → 300 → 200 → 0
//    이 그대로 보인다. 0 으로 돌아왔을 때 숫자가 0 이 아니면 그 차이가 곧 ★누적 드리프트★
//    다(linear_0803_pot.ino 실측 : 페달 1사이클마다 평균 −55카운트). 매번 영점을 다시
//    잡으면 이 정보가 지워진다.
//
//  ★ 튜닝 요령 — ★보는 것은 절대값이 아니라 '이동량(끝값 − 시작값)'이다★ ★
//    기본위치가 사이클마다 밀리므로(아래 ③) 화면의 절대값으로 판정하면 매번 어긋난다.
//    1) 0 을 넣어 기본위치로 보내고, 그때 숫자를 적어 둔다(= 이번 사이클의 시작값).
//    2) 1 을 넣고 멈춘 숫자에서 시작값을 뺀다. 그 이동량이 200 이어야 한다.
//       모자라면 MOVE_MS[0][1] 을 늘리고 넘으면 줄인다.
//    3) 같은 방법으로 0→2(300), 1→2(100), 2→1(−100) 을 맞춘다. →0 은 '하드스톱까지'라
//       시간이 넉넉하면 되고 정확히 맞출 필요가 없다.
//    ※ 한 번에 하나씩만 고칠 것. 절대값으로 바로 읽고 싶으면 ZERO_AT_HOME 을 true 로.
//    ※ 밟는 쪽과 놓는 쪽의 속도가 다르므로 1→2 와 2→1 의 시간이 같지 않은 것이 정상이다.
//
//  ★ 안전 ★
//    · 밟는 방향으로 ENC_HARD_MAX(360)를 넘으면 시간이 남아 있어도 즉시 끊는다.
//      전 행정이 약 400카운트(linear_0803_pot.ino 실측)이므로 2단(300) 위로 60카운트
//      여유를 둔 값이다. ★0804 의 330 을 그대로 쓰면 2단 목표에서 바로 걸린다★
//    · 어떤 계산 결과도 MAX_DRIVE_MS 보다 길게 밀지 않는다.
//    · E-stop(D12, NC)은 켜 둔 채로 시험한다. 발동하면 ★PWM 255★ 로 2단까지 밀고,
//      해제되면 기본위치로 되돌아온다. 배선 전이라면 ESTOP_ENABLED 를 false 로.
//
//  ★ 핀 (kasa_0804_B.ino 와 동일) ★
//      D2/D3  리니어 로터리 엔코더 A/B (INT4/INT5 벡터 직결 — 핀 고정)
//      D8/D9  리니어 DIR / PWM
//      D12    E-stop (NC, 평상시 LOW / 개방 시 HIGH)
// ============================================================

// ================= 핀 =================
const uint8_t ENC_A_PIN = 2;      // ★D2/D3 고정★ INT4/INT5 벡터에 직접 붙는다
const uint8_t ENC_B_PIN = 3;

const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

const uint8_t ESTOP_PIN = 12;
const bool ESTOP_ENABLED = true;   // false 로 두면 핀 e-stop 비활성(배선 전 시험용)

// 정방향(브레이크를 밟는 방향, 로드가 나옴) = LOW / 역방향(놓는 방향, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= 구동 PWM =================
// ★평시 150★ 0804 는 항상 255 였다. 255 는 관성 오버슛이 26~75카운트(목표의 49%)라
//   시간으로도 엔코더로도 정밀하게 세우기 어려웠다. 150 은 그만큼 느리므로 오버슛이
//   작아야 정상이다 — 이 코드로 확인할 것 중 하나다.
const int DRIVE_PWM = 150;
// ★E-stop 만 255★ 비상정지는 정확도보다 속도다. 과함은 ENC_HARD_MAX 가 막는다.
const int ESTOP_PWM = 255;


// ================= 단계별 위치 (엔코더 카운트, 부팅 영점 기준 절대값) =================
// 0804 는 POS_FULL 하나에서 1단을 비율(1/3)로 만들었지만, 여기서는 두 값을 따로 준다.
const long POS_LEVEL0 = 0;     // 기본위치(놓음)
const long POS_LEVEL1 = 200;   // 1단
const long POS_LEVEL2 = 300;   // 2단
const long LEVEL_POS[3] = { POS_LEVEL0, POS_LEVEL1, POS_LEVEL2 };


// ============ ★★ 단계 이동 구동시간 [ms] — 1차 실측으로 재산출 ★★ ============
//  MOVE_MS[지금단계][갈단계]. 대각선(같은 단계)은 0 = 아무것도 하지 않는다.
//
//  ┌ 1차 실측 (2026-08-13, PWM 150). 이동 = 끝값 − 시작값 ─────────────────────┐
//  │   0→1  320ms : +278, +271           평균 274.5   (0.858 카운트/ms)        │
//  │   0→2  440ms : +364, +355, +352     평균 357.0   (0.811)                  │
//  │   1→2  130ms : +101, +100           평균 100.5   (0.773)                  │
//  │   2→1  110ms : −131, −133, −131     평균 131.7   (1.197)                  │
//  │   →0  1000ms : 하드스톱까지 (−218 ~ −410)                                 │
//  └───────────────────────────────────────────────────────────────────────────┘
//
//  ★① 밟는 쪽은 깊을수록 느려진다★ 0~274 구간이 0.858 인데 274~357 구간은 0.687 이다.
//     브레이크가 압축될수록 부하가 커지니 물리적으로 당연하고, 그래서 ★단일 비례식
//     (카운트 = ms × k)으로는 맞출 수 없다★. 속도가 위치에 비례해 준다고 두고 위 두
//     점(320ms→274.5, 440ms→357)에 맞추면:
//
//         v(c) = 1.0024 − 0.0010·c  [카운트/ms]
//         t(C) = 1000 · ln( 1.0024 / (1.0024 − 0.001·C) )   [ms]
//
//     ★검산★ 이 모델로 1→2(200→300)를 예측하면 132ms 인데 실측이 130ms 다. 피팅에
//     쓰지 않은 독립 점이 맞으므로 모델을 믿고 아래 값을 뽑았다:
//         200카운트 → 223ms      300카운트 → 355ms      200→300 → 132ms
//
//  ★② 놓는 쪽이 1.4배 빠르다★ (1.20 vs 0.86) — 스프링이 복귀를 돕는다. 2→1 은 점이
//     하나뿐이라 비례로만 환산했다 : 110ms × (100/131.7) ≈ 84 → 85ms.
//
//     ※ [0][0] 만 0 이 아니다 — 기본위치에서 0 을 다시 넣으면 하드스톱까지 한 번 더
//       민다. 이것을 반복하며 숫자가 얼마나 밀리는지 보면 ★사이클당 드리프트★ 를
//       그 자리에서 잴 수 있다(아래 ③).
// ★→0 은 PWM 255 로 1초★ [2026-08-13 2차] 아래 ③ 참고. 여기만 255 를 쓰는 이유는
//   '정확히 얼마나'가 아니라 '확실히 끝까지'가 목적이기 때문이다. 부팅 시퀀스도 같은
//   값을 쓴다 — 부팅 영점과 이후 홈이 서로 다른 자리면 기준 자체가 둘이 된다.
const unsigned long HOME_MS  = 1000;
const int           HOME_PWM = 255;
const unsigned long MOVE_MS[3][3] = {
  /* 0 → */ { HOME_MS,  235,  380 },
  /* 1 → */ { HOME_MS,    0,  120 },
  /* 2 → */ { HOME_MS,   85,    0 },
};

//  ★③ [2026-08-13 2차] 홈을 600ms 로 줄였더니 ★기준이 무너졌다★ — 되돌렸다
//     ┌ 2차 실측 (홈 150PWM/600ms) ────────────────────────────────────────────┐
//     │   0→1  223ms : +185, +101   ← 편차 84 (1차는 7)                        │
//     │   0→2  355ms : +267, +277                                              │
//     │   1→2  130ms : +122         (1차 +100.5)                               │
//     │   2→1   85ms : −114, −90    → 평균 −102  ★목표 −100 명중★              │
//     │   1→0  600ms : −220, −105, −202   ← ★145 에서 출발해 −202, 150 에서     │
//     │                                     출발해 −105. 하드스톱에 닿았다 말았다│
//     └────────────────────────────────────────────────────────────────────────┘
//     600ms 로는 끝까지 못 간다. 시작 위치가 매번 달라지니 같은 시간에도 이동량이
//     널뛰었고(0→1 편차 7 → 84), 1차의 깨끗한 데이터가 2차에 무너진 원인이 이것이다.
//     ★그래서 →0 은 PWM 255 로 1초★ — '정확히 얼마나'가 아니라 '확실히 끝까지'가
//     목적인 유일한 동작이므로 힘과 시간을 아끼지 않는다.
//
//  ★④ 3차 시간 산출 ★ 1·2차의 모델을 평균해 v(c) = 0.95 − 0.001·c 를 쓴다
//     (1차 v0=1.0024 / 2차 v0=0.9104. 2차가 낮은 것은 위 ③의 시작점 흔들림 때문이다).
//         t(200) = 236ms → 235      t(300) = 380ms → 380
//     1→2 는 모델이 144ms 라 하지만 두 차례 실측이 130ms 에서 +100.5 / +122 였다
//     (평균 111). ★모델이 중간 구간을 20% 정도 과대평가한다★ — 정지에서 떼어낼 때의
//     초기 가속을 모델이 못 담기 때문이다. 그래서 이 칸만은 실측을 따라 120 으로 둔다.
//     2→1 은 85ms 에서 −102 가 나왔으므로 그대로 둔다.

// 어떤 값도 이보다 길게 밀지 않는다(표를 잘못 고쳤을 때의 최종 방어).
const unsigned long MAX_DRIVE_MS = 1500;


// ================= 부팅 시퀀스 =================
// 들어가는 방향으로 BOOT_PUSH_MS 밀어 하드스톱까지 보낸 뒤(= 기본위치),
// BOOT_WAIT_MS 를 쉬고 ★그 자리를 영점으로★ 잡는다. 이후 영점은 다시 잡지 않는다.
const unsigned long BOOT_PUSH_MS = 1000;
const unsigned long BOOT_WAIT_MS = 2000;

// 구동을 끊은 뒤 관성이 멎기를 기다리는 시간. 0804 의 LIN_SETTLE_MS 를 그대로 쓴다.
// 이 시간이 끝나야 다음 명령을 받는다(★영점은 잡지 않는다★).
const unsigned long SETTLE_MS = 100;

// ================= ★★ 수동 브레이크 (사람이 페달을 밟는 경우) ★★ =================
//  ★전제 : 사람은 리니어보다 ★더★ 밟을 수만 있고 덜 밟을 수는 없다★ (기구 구조상)
//  이 한 줄이 두 가지를 한꺼번에 정한다.
//
//  ① '뗐다'를 어떻게 아는가 — ★리니어의 현재 위치가 곧 엔코더의 바닥선이다★
//     사람이 그 아래로 내려갈 수 없으므로, 엔코더가 리니어 위치까지 되돌아와 멎으면
//     그것이 곧 '뗐다'다. 증분 엔코더의 절대 기준이 없다는 문제를 여기서만은 피해
//     갈 수 있다 — 기준을 절대 좌표가 아니라 ★리니어 자신★ 에서 가져오기 때문이다.
//     그래서 lin_pos(마지막 구동이 놓고 온 엔코더 값)를 들고 다닌다. 미끄러짐이
//     있어도 lin_pos 는 그때그때 실측이라 함께 밀리므로 판정이 흔들리지 않는다.
//
//  ② 밟힘을 감지해도 ★리니어를 집어넣지 않는다★
//     넣어 봐야 페달은 사람 발이 잡고 있어 따라 올라오지 않는다(위 전제). 대신
//     ★다음 명령에서 이미 밟힌 만큼을 빼고 민다★ — 1단에서 사람이 250까지 밟아 둔
//     상태로 2단(300) 명령이 오면 300−250 = 50 만큼만 나간다. 목표를 이미 지나쳐
//     있으면(예 2단 명령인데 320) 그 자리를 지키고 아무것도 하지 않는다. 사람이
//     발을 떼면 페달이 리니어 위치까지 내려와 멎고, 그것이 ①의 '뗐다'다.
//
//  ★보정에 쓰는 모델★ 표(MOVE_MS)는 '정상 경로' 전용이다. 사람이 끼어들어 시작점이
//  달라지면 표를 쓸 수 없으므로 위 ④의 속도 모델로 시간을 만든다:
//      t(C) = (1/M)·ln( V0 / (V0 − M·C) )        C = 홈 기준 카운트
//  놓는 쪽은 위치 의존을 아직 재지 못해 상수 속도로 둔다.
const bool  MANUAL_DETECT   = true;    // false 면 사람 개입을 무시하고 표대로만 민다
const long  MANUAL_ENTER    = 3;       // 리니어 위치보다 이만큼 더 밟히면 '사람이 밟았다'
const long  MANUAL_BACK     = 5;       // 리니어 위치 + 이 안으로 돌아오면 '뗐다'
const unsigned long MANUAL_STILL_MS = 300;   // 그 상태로 이만큼 멎어야 확정

const float MODEL_V0  = 0.95f;    // [카운트/ms] 홈에서의 밟기 속도 (1·2차 평균)
const float MODEL_M   = 0.0010f;  // 1카운트 깊어질 때마다 느려지는 양
const float MODEL_REV = 1.20f;    // [카운트/ms] 놓는 쪽 (2→1 실측 −131.7/110ms)


// ★기본위치(0)에 도착했을 때 그 자리를 영점으로 다시 잡을 것인가★
//   false (기본) : 부팅 때 잡은 영점을 끝까지 쓴다. 화면 숫자에 ③의 드리프트가 그대로
//                  누적되어 보인다 — 대신 1단이 '200 근처'가 아니라 '기본위치+200' 에
//                  선다(물리적 위치는 맞고 표시만 밀린다).
//   true         : 하드스톱에 닿을 때마다 영점을 회복한다. 1단·2단이 화면에서도 곧바로
//                  200·300 으로 읽혀 다음 튜닝이 쉬워지지만, 드리프트는 안 보인다.
//   ※ 매 이동마다 영점을 잡는 0804 방식과는 다르다. 여기서 잡는 것은 ★물리적 기준점인
//     하드스톱★ 한 곳뿐이다.
const bool ZERO_AT_HOME = false;


// ================= 안전 =================
// 밟는 방향 절대 상한. 전 행정 약 400카운트에서 2단(300) 위로 60 여유.
const long ENC_HARD_MAX = 360;

// 방향 반전 보호 (E-stop 이 해제 구동을 취소하고 체결로 넘어갈 때만 쓰인다)
const unsigned long REVERSE_DEADTIME_MS = 30;

// E-stop 체결 시간. 부족한 것이 위험하므로 남은거리를 계산하지 않고 최대로 준다
// (PWM 255 기준 300카운트가 약 223ms 이므로 그 위로 잡았다).
const unsigned long ESTOP_PUSH_MS = 250;

// 발동·해제 모두 이 시간 동안 레벨이 연속 유지되어야 인정한다(kasa_0804_B.ino 와 동일).
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 500;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 500;


// ================= 출력 =================
const unsigned long BAUD = 115200;
const unsigned long REPORT_MS = 5;      // 값이 변하지 않아도 매 주기 = 등간격 표본
const bool VERBOSE = false;             // true 면 '#' 진단 줄이 함께 나간다


// ================= 엔코더 (kasa_0804_B.ino 에서 그대로) =================
const bool USE_PULLUP = false;         // 토템폴(T) 출력이라 INPUT 이 맞다
const bool INVERT_DIR = false;         // ★밟는 방향이 음수로 세어지면 true★
const bool RECOVER_LOST_EDGES = true;  // 유실 엣지 ±2 보정

const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};

volatile long     enc_count = 0;
volatile uint8_t  enc_state = 0;
volatile uint16_t enc_err   = 0;
volatile int8_t   enc_dir   = 0;

static inline uint8_t readAB() {
  uint8_t p = PINE;
  uint8_t a = (p >> 4) & 0x01;   // PE4 = D2
  uint8_t b = (p >> 5) & 0x01;   // PE5 = D3
  return (uint8_t)((a << 1) | b);
}

static inline void encStep() __attribute__((always_inline));
static inline void encStep() {
  uint8_t cur  = readAB();
  uint8_t prev = enc_state;
  if (cur == prev) return;
  enc_state = cur;

  int8_t step = QDEC_TABLE[(prev << 2) | cur];
  if (step == 0) {
    // A와 B가 동시에 바뀐 것으로 보인다 = 중간 상태를 건너뛰었다(= 정확히 2카운트).
    enc_err++;
    if (RECOVER_LOST_EDGES && enc_dir != 0) {
      enc_count += (long)(INVERT_DIR ? -enc_dir : enc_dir) * 2;
    }
    return;
  }
  enc_dir = step;
  enc_count += INVERT_DIR ? -step : step;
}

ISR(INT4_vect) { encStep(); }
ISR(INT5_vect) { encStep(); }

long encRead() {
  long c;
  noInterrupts();
  c = enc_count;
  interrupts();
  return c;
}

void zeroHere() {
  noInterrupts();
  enc_count = 0;
  enc_state = readAB();
  enc_err   = 0;
  enc_dir   = 0;
  interrupts();
}

void setupEncoder() {
  const uint8_t mode = USE_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_A_PIN, mode);
  pinMode(ENC_B_PIN, mode);

  // 현재 레벨을 시작 상태로 채택한 뒤 인터럽트를 건다
  enc_state = readAB();
  enc_count = 0;
  enc_err   = 0;
  enc_dir   = 0;

  // INT4/INT5 를 양쪽 엣지(01)로
  EICRB = (EICRB & ~((1 << ISC41) | (1 << ISC40) | (1 << ISC51) | (1 << ISC50)))
                 | (1 << ISC40) | (1 << ISC50);
  EIFR  = (1 << INTF4) | (1 << INTF5);
  EIMSK |= (1 << INT4) | (1 << INT5);
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


// ================= 상태 =================
enum {
  ST_BOOT_PUSH,   // 부팅 : 들어가는 방향으로 미는 중
  ST_BOOT_WAIT,   // 부팅 : 멎기를 기다리는 중 (끝나면 영점)
  ST_IDLE,        // 대기 : 0/1/2 를 받는다
  ST_MOVE,        // 단계 이동 구동 중
  ST_SETTLE,      // 구동을 끊고 관성이 멎기를 기다리는 중
  ST_ES_PUSH,     // E-stop 체결 중 (255)
  ST_ES_HOLD      // E-stop 체결 완료, 해제를 기다리는 중
};
uint8_t st = ST_BOOT_PUSH;

uint8_t  cur_level  = 0;         // 지금 있다고 보는 단계
uint8_t  move_to    = 0;         // 이번 구동이 끝나면 될 단계
bool     move_fwd   = false;     // 이번 구동 방향이 '밟는 쪽'인가
unsigned long phase_t   = 0;     // 지금 상태로 들어온 시각
unsigned long move_end  = 0;     // 구동을 끊을 시각 (ST_MOVE·ST_ES_PUSH 공용)
unsigned long es_gate   = 0;     // 반전 보호가 풀리는 시각
bool     es_driving = false;
bool     zeroed     = false;     // 부팅 영점을 잡았는가 (E-stop 이 부팅을 가로챈 경우 대비)

// ★리니어가 마지막으로 놓고 온 자리(엔코더 기준)★ = 사람이 밟을 수 있는 바닥선.
//   미끄러짐이 있어도 매번 실측으로 갱신되므로 판정 기준이 함께 밀린다.
long     lin_pos   = 0;
long     home_enc  = 0;          // 마지막 하드스톱의 엔코더 값 (모델의 C=0 기준)
bool     manual_on = false;      // 사람이 리니어보다 더 밟고 있는가
unsigned long still_t = 0;       // '리니어 위치로 돌아와 멎었다'를 재기 시작한 시각

// E-stop 판정
bool estop_active = false;
unsigned long estop_high_t = 0;
unsigned long estop_low_t  = 0;

unsigned long report_t = 0;
char rxBuf[8];
uint8_t rxLen = 0;


void note(const char* msg) {
  if (VERBOSE) { Serial.print("# "); Serial.println(msg); }
}


// ================= 단계 이동 =================
void enterSettle(unsigned long now) {
  linearStop();
  st = ST_SETTLE;
  phase_t = now;
}

// 사람이 밟아 둔 상태에서 쓰는 시간 환산 (표를 못 쓰는 경우). 위 '수동 브레이크' 절 참고.
unsigned long msForSpan(long from_abs, long to_abs) {
  float c0 = (float)(from_abs - home_enc);
  float c1 = (float)(to_abs   - home_enc);
  float t;
  if (c1 > c0) {                      // 밟는 쪽 : 깊을수록 느려진다
    float d0 = MODEL_V0 - MODEL_M * c0;
    float d1 = MODEL_V0 - MODEL_M * c1;
    if (d0 < 0.05f) d0 = 0.05f;       // 모델이 속도 0 을 내는 영역은 상한이 막는다
    if (d1 < 0.05f) d1 = 0.05f;
    t = logf(d0 / d1) / MODEL_M;
  } else {                            // 놓는 쪽 : 상수 속도로 둔다
    t = (c0 - c1) / MODEL_REV;
  }
  if (t < 0.0f) t = 0.0f;
  return (unsigned long)t;
}

void startMove(uint8_t to_level, unsigned long now) {
  // 0(기본위치)만은 같은 단계에서도 받아 준다 — '하드스톱까지 다시 밀기'는 언제나
  // 의미가 있다(드리프트 확인). 1·2 는 이미 그 단계면 아무것도 하지 않는다.
  if (to_level > 2) return;
  if (to_level == cur_level && to_level != 0) return;

  long e = encRead();
  unsigned long ms;
  int pwm;

  if (to_level == 0) {
    // ★끝까지 밀기★ 목표가 카운트가 아니라 하드스톱이다. 사람이 밟고 있어도 그대로 민다
    //   (사람이 발을 떼는 순간 페달이 따라 내려오도록 리니어가 먼저 들어가 있어야 한다).
    ms = HOME_MS;
    pwm = HOME_PWM;
    move_fwd = false;
  } else {
    // 목표는 ★리니어가 놓고 온 자리 기준★ 이다 — 절대 좌표를 쓰면 미끄러짐만큼 어긋난다.
    long target = lin_pos + (LEVEL_POS[to_level] - LEVEL_POS[cur_level]);
    long span = target - e;
    if (span > -MANUAL_ENTER && span < MANUAL_ENTER) {
      // 사람이 이미 그 자리까지 밟아 놓았다 — 구동 없이 단계만 확정한다
      cur_level = to_level;
      lin_pos = e;
      note("ALREADY");
      return;
    }
    move_fwd = (span > 0);
    pwm = DRIVE_PWM;
    long off = e - lin_pos;
    if (off > -MANUAL_ENTER && off < MANUAL_ENTER) {
      ms = MOVE_MS[cur_level][to_level];      // 정상 경로 = 실측 표(튜닝 대상)
    } else {
      ms = msForSpan(e, target);              // 사람이 밟아 둔 상태 = 모델 보정
      note("COMP");
    }
  }

  if (ms == 0) return;
  if (ms > MAX_DRIVE_MS) ms = MAX_DRIVE_MS;

  move_to  = to_level;
  move_end = now + ms;
  linearDrive(move_fwd ? LINEAR_FWD : LINEAR_REV, pwm);
  st = ST_MOVE;
  phase_t = now;
}

void updateMove(unsigned long now) {
  // ★밟는 방향에서만 상한을 본다★ 놓는 방향은 하드스톱이 물리적으로 막아 준다.
  if (move_fwd && encRead() >= ENC_HARD_MAX) {
    note("HARDMAX");
    enterSettle(now);
    return;
  }
  if ((long)(now - move_end) >= 0) {
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

// 발동 진입 : 진행 중이던 구동을 끊고 체결로 넘어간다.
// 놓는 방향으로 가던 중이었다면 급반전을 피해 REVERSE_DEADTIME_MS 만 쉰다.
void enterEstop(unsigned long now) {
  bool was_rev = (st == ST_MOVE && !move_fwd);
  linearStop();
  es_gate = was_rev ? (now + REVERSE_DEADTIME_MS) : now;
  es_driving = false;
  st = ST_ES_PUSH;
  phase_t = now;
  note("ESTOP");
}


// ================= 시리얼 입력 ("0"/"1"/"2" 한 줄만) =================
void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch != '\n') {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = ch;
      else rxLen = sizeof(rxBuf);          // 넘치면 그 줄 전체를 버린다
      continue;
    }
    // 줄 끝 — 정확히 한 글자이고 0/1/2 일 때만 인정한다
    if (rxLen == 1 && rxBuf[0] >= '0' && rxBuf[0] <= '2') {
      uint8_t lv = (uint8_t)(rxBuf[0] - '0');
      if (st == ST_IDLE) startMove(lv, now);
      else               note("BUSY");     // 구동·정착·E-stop 중에는 받지 않는다
    }
    rxLen = 0;
  }
}


// ================= 출력 (엔코더 값 하나) =================
void report(unsigned long now) {
  if (now - report_t < REPORT_MS) return;
  report_t = now;
  Serial.println(encRead());
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  pinMode(ESTOP_PIN, INPUT_PULLUP);

  setupEncoder();

  // ★부팅 시퀀스 시작★ 들어가는 방향으로 BOOT_PUSH_MS
  //   ★0 명령과 같은 힘·시간을 쓴다★ 둘이 다르면 부팅 영점과 이후 홈이 서로 다른
  //   자리에 서고, 그러면 기준이 둘이 되어 이동량 판정이 통째로 흔들린다(위 ③).
  unsigned long now = millis();
  linearDrive(LINEAR_REV, HOME_PWM);
  st = ST_BOOT_PUSH;
  phase_t = now;
  report_t = now;
  note("BOOT");
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  updateEstop(now);

  // E-stop 은 어느 상태에서든 가로챈다(부팅 시퀀스 포함).
  if (estop_active && st != ST_ES_PUSH && st != ST_ES_HOLD) {
    enterEstop(now);
  }

  pollSerial(now);      // ★E-stop 판정 뒤에 둔다★ 발동한 루프에서 들어온 명령이
                        //   한 번 먹는 틈을 없앤다(kasa_0804_B.ino 와 같은 순서).

  switch (st) {
    case ST_BOOT_PUSH:
      if (now - phase_t >= BOOT_PUSH_MS) {
        linearStop();
        st = ST_BOOT_WAIT;
        phase_t = now;
      }
      break;

    case ST_BOOT_WAIT:
      if (now - phase_t >= BOOT_WAIT_MS) {
        zeroHere();               // ★영점은 여기 한 번뿐이다★
        zeroed = true;
        cur_level = 0;
        lin_pos = 0;
        home_enc = 0;
        manual_on = false;
        still_t = 0;
        st = ST_IDLE;
        phase_t = now;
        note("ZERO");
      }
      break;

    case ST_IDLE: {
      // ★수동 브레이크 감지★ 사람은 리니어보다 더 밟을 수만 있다(위 절 참고).
      //   그래서 '리니어 위치 대비 얼마나 더 들어가 있나'만 보면 된다.
      if (!MANUAL_DETECT) break;
      long e = encRead();
      if (!manual_on) {
        if (e - lin_pos >= MANUAL_ENTER) {
          manual_on = true;
          still_t = 0;
          note("MANUAL");        // 여기서 리니어를 빼지 않는다 — 다음 명령이 보정한다
        }
      } else if (e - lin_pos <= MANUAL_BACK) {
        // 바닥선까지 되돌아왔다. 그 아래로는 못 가므로 ★멎으면 뗀 것★.
        if (still_t == 0) still_t = now;
        else if (now - still_t >= MANUAL_STILL_MS) {
          manual_on = false;
          still_t = 0;
          note("RELEASED");
        }
      } else {
        still_t = 0;             // 다시 밟았다 — 멎음 판정을 처음부터
      }
      break;
    }

    case ST_MOVE:
      updateMove(now);
      break;

    case ST_SETTLE:
      if (now - phase_t >= SETTLE_MS) {
        cur_level = move_to;      // 영점은 잡지 않는다 — 숫자는 절대값으로 남는다
        if (ZERO_AT_HOME && move_to == 0) {
          zeroHere();             // 하드스톱 = 유일한 물리 기준점 (위 상수 주석 참고)
          note("REZERO");
        }
        // ★리니어가 놓고 온 자리를 실측으로 붙든다★ 사람이 밟았는지, 다음 명령에
        //   얼마를 빼야 하는지가 전부 이 값 기준이다.
        lin_pos = encRead();
        if (move_to == 0) home_enc = lin_pos;   // 모델의 C=0 기준도 함께 회복
        manual_on = false;
        still_t = 0;
        st = ST_IDLE;
        phase_t = now;
      }
      break;

    case ST_ES_PUSH:
      if (!es_driving) {
        if ((long)(now - es_gate) < 0) break;      // 반전 보호 대기
        linearDrive(LINEAR_FWD, ESTOP_PWM);        // ★E-stop 만 255★
        move_end = now + ESTOP_PUSH_MS;
        es_driving = true;
      }
      if (encRead() >= ENC_HARD_MAX || (long)(now - move_end) >= 0) {
        linearStop();
        cur_level = 2;                             // 체결했으므로 2단으로 본다
        lin_pos = encRead();                       // 바닥선도 여기로 옮겨 둔다
        manual_on = false;
        still_t = 0;
        st = ST_ES_HOLD;
        phase_t = now;
      }
      break;

    case ST_ES_HOLD:
      if (!estop_active) {
        note("ESTOP CLEAR");
        if (!zeroed) {
          // ★부팅 시퀀스 도중에 E-stop 이 들어온 경우★ 영점을 아직 안 잡았으므로
          //   기본위치 복귀로 끝내지 않고 부팅 시퀀스를 처음부터 다시 탄다.
          linearDrive(LINEAR_REV, HOME_PWM);
          st = ST_BOOT_PUSH;
          phase_t = now;
        } else {
          startMove(0, now);                       // 해제 → 기본위치 복귀
        }
      }
      break;
  }

  report(now);
}
