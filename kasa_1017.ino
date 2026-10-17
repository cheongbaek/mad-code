// ============================================================
//  kasa_1017.ino : ★통합 보드★ — A보드(인휠 좌우 독립 PID + 쓰로틀 페달) + B보드(조향 PD + 제동
//                  시간 제어 2단 + 주행모드 스위치 + E-stop + 제동등) 를 Mega 2560 ★한 장★ 으로
//                  (Arduino Mega 2560)
//
//  kasa_1016_A.ino + kasa_1016_B.ino 기반. ★두 보드의 제어 코드는 한 줄도 바꾸지 않았다★ —
//  인휠 PID·FF·블랭킹·중앙값·무입력 워치독 / 조향 PD·캘리브레이션·EEPROM / 리니어 시간표 /
//  E-stop 100ms 확인·AS 상태([1007])·E_STOP_TEST([1015-1]) / 제동등([1016-B]) / 상태 칸([1016-C]).
//  ★바뀐 것은 넷이다★ — 핀([1017-1]) · 입력 규약([1017-2]) · 출력 규약([1017-3]) ·
//  펌웨어 안의 구동 차단([1017-4]). 각 기능의 근거·이력은 kasa_1016_A / kasa_1016_B 머리말이 정본이다.
//
//  ⚠️ ★gold nxde/arduino.py 와 짝이다★ — 2026-10-17 의 새 arduino.py(한 보드, "I," 한 줄)와 함께 쓴다.
//     옛 두 보드용은 nxde/arduino_old.py 로 남겼다. 옛 상위의 줄(1칸·2칸)은 4칸이 아니라 버려지므로
//     옛 상위가 붙어도 차는 움직이지 않는다(안전 쪽).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1017-1] 핀 — A보드 핀은 그대로, B보드 핀만 옮겼다 (사용자 지시 2026-10-17) ★★
// ════════════════════════════════════════════════════════════════════════════
//      기능                         핀     출처  타이머 (Mega)
//      인휠 홀 왼쪽 (CHANGE 인터럽트)  D21    A     INT0
//      인휠 홀 오른쪽 (CHANGE 인터럽트) D20    A     INT1
//      인휠 PWM 왼쪽                  D8     A     Timer4 (OC4C)
//      인휠 PWM 오른쪽                 D10    A     Timer2 (OC2A)
//      쓰로틀 페달 가변저항             A15    A     ADC
//      조향 가변저항                   ★A10★  B     ADC          (1016_B : A2)
//      조향 DC PWM                    ★D44★  B     Timer5 (OC5C) (1016_B : D6)
//      조향 DC DIR                    ★D46★  B     digitalWrite 만 (1016_B : D7)
//      리니어 PWM                     ★D45★  B     Timer5 (OC5B) (1016_B : D8 ← 인휠 왼쪽과 겹쳤다)
//      리니어 DIR                     ★D47★  B     digitalWrite 만 (1016_B : D9)
//      E-stop (NC)                    D12    B     입력
//      주행모드 스위치                  D5     B     입력
//      제동등 출력                     D11    B     digitalWrite 만
//      제동등 스위치                    A5     B     입력 (INPUT_PULLUP) — ★지시에 없어 1016_B 그대로 두었다★
//    ▶ 번호 중복 없음. ★PWM 타이머도 겹치지 않는다★ — 인휠 Timer4·Timer2, 조향·리니어 Timer5,
//      millis() 는 Timer0(D4·D13 — 안 쓴다). D46 은 OC5A 지만 DIR 이라 digitalWrite 만 쓴다
//      (digitalWrite 는 그 핀의 채널 OC5A 만 끄므로 D44·D45 PWM 에 영향이 없다). D11 은 Timer1 이지만
//      digitalWrite 만 쓴다(0821 그대로). Timer5 기본 주파수는 ≈490 Hz 로 1016_B 의 D6·D8(Timer4)과 같다.
//    ▶ D0/D1(시리얼)·D20/D21 의 I2C 기능은 쓰지 않는다(Wire 없음 — D20/D21 은 홀 인터럽트 전용).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1017-2] 입력 규약 — ★한 줄 4칸★ (사용자 지시 2026-10-17) ★★
// ════════════════════════════════════════════════════════════════════════════
//      "<왼쪽값>,<오른쪽값>,<명령조향각>,<브레이크단수>"      예) "7,7,-12,0"   "0,0,x,2"   "140,140,x,0"
//         왼쪽값·오른쪽값 : 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지   (= 1016_A 의 콤마 2값 그대로)
//                           ★좌우가 같아도 두 칸 다 쓴다★ ("2,2") — 1016_A 의 단일값 양식은 없다.
//         명령조향각     : −40~+40 (− 좌 / + 우) 또는 x (힘빼기)         (= 1016_B 첫 칸 그대로)
//         브레이크단수   : 0 / 1 / 2                                       (= 1016_B 둘째 칸 그대로)
//      "a"   조향 영점 캘리브레이션 (= 1016_B)
//      "-"   연결확인 핸드셰이크 → "YES" (= 1016_A·B)
//    ▶ ★네 칸이 다 맞아야 받는다★ — 칸이 모자라거나 남거나 빈 칸·공백·형식 오류가 있으면 줄 통째로 버린다.
//      (1016_B 는 브레이크 값이 범위 밖이면 그 칸만 버리고 조향은 받았다 — 그 규칙은 그대로다.)
//    ▶ 구동 두 칸은 ★언제나★ 받는다(E-stop·캘리브레이션 중에도 — 1016_A 에는 그 둘이 없었다).
//      E-stop 이면 [1017-4] 가 인휠을 끊고 있다. 조향·브레이크 두 칸은 1016_B 처럼 E-stop·캘리브레이션
//      중에는 받지 않고, 수동조종에서는 브레이크를 0 으로 받는다([1007-2]).
//    ▶ ★조향은 '바뀌었을 때 + 0.9초마다' 만 다시 건다 (STEER_REARM_MS)★ — 1016_B 는 줄을 받을 때마다
//      조향 도달 판정(SETTLE_MS 500ms)을 처음부터 다시 셌다. 그래서 상위는 B 줄을 '값이 바뀔 때와 1초
//      keepalive' 에만 보냈다. 한 줄이 되면 구동값이 바뀔 때마다(수동조종이면 20Hz) 조향 칸도 함께 오므로,
//      그대로 두면 도달 판정이 영영 성립하지 않아 PD 가 목표 근처에서 계속 힘을 준다.
//      → 조향 칸이 직전과 같으면 다시 걸지 않는다. 단 직전에 건 지 0.9초가 지났으면 다시 건다 — 1016_B 가
//        1초 keepalive 마다 다시 걸던 것(떨어진 자리를 다시 잡는 효과)을 그대로 살린다.
//    ▶ ★무입력 워치독([0904-4], 3초)은 '완전한 줄 하나' 면 무엇이든 센다★(1016_A 와 같은 뜻 — PC→보드
//      방향이 살아 있는가). 워치독이 세우는 것은 인휠뿐이다(조향·제동은 1016_B 처럼 마지막 명령을 문다).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1017-3] 출력 규약 — ★"I," 한 줄★ 50ms 마다 (사용자 지시 2026-10-17) ★★
// ════════════════════════════════════════════════════════════════════════════
//      "I,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>,<브레이크단계>,<D5>,<실측조향각>,<상태>"
//         왼쪽·오른쪽펄스 : 1016_A S, 와 같다 (PID 가 실제로 본 값 — 3점 중앙값 뒤)
//         쓰로틀raw       : 1016_A S, 와 같다 (A15, 9점 중앙값)
//         브레이크단계    : 지금 지시된 단계 0/1/2 (brake_cmd_level — E-stop 이면 2, 수동이면 0)
//         D5              : 1 자율주행 / 0 수동조종 (1016_B P, 넷째 칸)
//         ★실측조향각★    : −40~+40 (1016_B P, 둘째 칸) — ★지시에 없어 덧붙였다★ /steer_angle_measured 의 출처
//         ★상태★          : 2 E-stop 효력 / 1 제동등 조건 / 0 (1016_B P, 셋째 칸) — ★지시에 없어 덧붙였다★
//                           /estop 과 상위의 쓰로틀 차단이 이것을 본다. 브레이크단계 2 만으로는
//                           '주행이 2단을 지시했다' 와 'E-stop' 이 구별되지 않는다.
//      "YES"     핸드셰이크 응답 (한 번)
//      "# ..."   캘리브레이션 결과 등 진단 — 상위는 버린다
//    ▶ 덧붙인 두 칸은 지시한 여섯 칸 ★뒤에★ 둔다 — 지시한 칸의 자리를 바꾸지 않는다.
//    ▶ 핸드셰이크 유예(2초) 중에는 내보내지 않는다 — ★단 E-stop 이면 낸다★(1016_B 규칙).
//    ▶ 접두어 "I," 가 있어 상위가 이 보드를 알아본다(옛 A 의 "S," · 옛 B 의 "P," 와 겹치지 않는다).
//      CH340 클론의 TX 에코(접두어 없는 4칸 줄)와도 섞이지 않는다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [1017-4] 구동 차단이 ★보드 안에서★ 일어난다 — 상태 2·1 이면 인휠 0 (FW_DRIVE_CUT) ★★
// ════════════════════════════════════════════════════════════════════════════
//    ▶ 1016 에서는 상위(arduino.py compose)가 B 상태 2(E-stop) 또는 1(제동등 조건)을 보고 A 에
//      ★쓰로틀 0★ 을 보냈다(모드 무관). 두 보드가 갈라져 있어 그 길밖에 없었다.
//      한 보드가 되었으니 ★같은 판정을 보드가 직접 한다★ — 상태 칸을 만드는 식과 같은 식이다:
//          차단 = estop_active (상태 2) || brakelight_on (상태 1)
//    ▶ ★거동은 1016 과 같다★ — 상위도 같은 때 0 을 보내므로 결과가 같고, 다른 것은 ★상위가 죽거나
//      늦어도 끊긴다★ 는 것뿐이다(1016 에서 E-stop 의 구동 차단은 PC 를 거쳐야만 성립했다 —
//      gold CLAUDE.md 8.5 R3. 이제 규정 <표 4> AS-Emergency 'Tractive unavailable' 을 보드가 보장한다).
//    ▶ 구현 — 상위의 구동 줄("A,...")은 cmd_drive 에 ★늘 받아 둔다★. 실제로 인휠에 거는 값은
//      차단 중이면 0, 아니면 cmd_drive 다. 차단이 들어가는 순간 PWM 을 그 자리에서 0 으로 쓰고,
//      풀리는 순간 그때의 cmd_drive 를 다시 건다 — 상위의 다음 keepalive(최대 1초)를 기다리지 않는다.
//      (상위도 차단 중에 0 을 보내므로 대개 풀리는 순간의 cmd_drive 는 0 이다 — 1016 과 같다.)
//      걸 때는 1016_A 의 applySide 를 그대로 탄다 — 0 은 펄스 모드 정지(코스트), 직접 PWM 도 해제된다.
//    ▶ 무입력 워치독은 cmd_drive 도 0 으로 지운다 — 링크가 돌아온 뒤 옛 명령이 되살아나지 않게.
//    ▶ ★되돌리려면 FW_DRIVE_CUT 을 false 로★ — 그러면 1016 과 똑같이 상위의 0 명령에만 기댄다.
//
//  ★ 켤 때 주의 ★ (1016_B 그대로) 수동조종으로 켜지면 곧바로 리니어를 0.5 s 들여보낸다(끝까지).
//    자율로 켜지면 움직이지 않는다 — 0단에 있다고 보고 상위 지시를 기다린다.
//  ★ E_STOP_TEST ★ 1016_B 그대로 true 다 — ★대회에는 false 로 굽는다★ (헤더 [1015-1] 은 1016_B).
// ============================================================

#include <EEPROM.h>

// ================= 통신 (A·B 공통) =================
const unsigned long BAUD = 115200;
const unsigned long TELE_MS = 50;              // 출력 주기 — S, 와 P, 를 같은 창에 낸다

// ================= 공통 제어주기 (A·B 공통 — 둘 다 20ms) =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ============================================================
//  ★★ 핀 ([1017-1]) ★★
// ============================================================
// --- A : 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 21;   // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 20;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- A : 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (펄스 21과 같은 컨트롤러)
const uint8_t PWM_PIN_R = 10;    // 오른쪽 모터 PWM (펄스 20과 같은 컨트롤러)

// --- A : 쓰로틀(가속) 페달 가변저항 — 이 보드는 이 값으로 제어하지 않는다. raw 를 보고만 한다 ---
const uint8_t THROTTLE_PIN = A15;

// --- B : 조향 / 리니어 ([1017-1] 옮김) ---
const uint8_t DC_DIR_PIN = 46;      // ★[1017-1]★ 1016_B : 7
const uint8_t DC_PWM_PIN = 44;      // ★[1017-1]★ 1016_B : 6
const uint8_t DC_POT_PIN = A10;     // ★[1017-1]★ 1016_B : A2 — 조향 가변저항

const uint8_t LINEAR_DIR_PIN = 47;  // ★[1017-1]★ 1016_B : 9
const uint8_t LINEAR_PWM_PIN = 45;  // ★[1017-1]★ 1016_B : 8

// --- B : E-stop ---
const uint8_t ESTOP_PIN = 12;
// ★차량의 소프트웨어 E-stop 을 판정하는 곳은 여기 하나다★ 여기까지 false 로 두면 D12 라인을 보는 곳이
//   하나도 남지 않는다.
const bool ESTOP_ENABLED = true;     // false 로 두면 핀 e-stop 비활성

// ★[1015-1] E-stop 시험 스위치★ (헤더는 1016_B)
//   true  : 수동조종에서도 E-stop 이 리니어 2단을 문다 (0909 동작)
//   false : 자율주행에서만 E-stop 이 효력을 갖는다 (1007 동작 = 대회 규정 AS-Off)
//   ★대회에는 false 로 굽는다★
const bool E_STOP_TEST = true;

const uint8_t MODE_PIN = 5;          // 주행모드 스위치 (D5 ── 스위치 ── GND)

// ★[1016-B] 제동등★ — 켜짐 = A5 스위치 닫힘(LOW) 또는 1·2단 지령
//   배선 A5 ── 스위치 ── GND, INPUT_PULLUP (D5·D12 와 같은 꼴)
const uint8_t BRAKE_SW_PIN   = A5;   // 제동등 스위치 (LOW = 닫힘 = 켬)
const uint8_t BRAKELIGHT_PIN = 11;   // 제동등 출력 (모터드라이버 PWM 선, digitalWrite 만)

// ★[1017-4] 상태 2·1 이면 보드가 스스로 인휠을 끊는다★ false = 1016 처럼 상위의 0 명령에만 기댄다
const bool FW_DRIVE_CUT = true;


// ============================================================
//  ===================  A : 인휠 (1016_A 그대로)  ===================
// ============================================================

// ================= [0730-1] 쓰로틀 필터 : 9샘플 중앙값 =================
const uint8_t THROTTLE_MEDIAN_N = 9;   // 반드시 홀수
int throttleRaw = 0;                   // 최근 중앙값 (0~1023, 텔레메트리용)


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(펄스21-PWM8), 오른쪽(펄스20-PWM10)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ================= ★ 값 해석 경계 ★ =================
const int TARGET_MAX = 15;      // 0~15 = 펄스 목표. 단일 값 입력은 이 범위만 유효
const int PWM_DIRECT_MAX = 255; // 16~255 = 직접 PWM (콤마 2값 형식에서만). 256 이상 = 정지

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= ★ [0730-2] 허수 펄스 판정 경계 (튜닝 지점) ★ =================
const int PULSE_SANITY_MAX = 40;
const int NOISE_HOLD_MAX_CYCLES = 50;   // 50주기 = 1초

// ================= ★ [0730-2] 기동 블랭킹 LAUNCH (튜닝 지점) ★ =================
const int LAUNCH_ENTRY_SPEED_MAX = 1;      // 이 펄스 이하일 때만 '정지 상태'로 보고 진입
const int LAUNCH_SETTLE_CYCLES   = 5;      // 정상 펄스가 이만큼 연속되면 노이즈 종료로 판정
const unsigned long LAUNCH_MAX_MS = 3000;  // 이 시간이 지나면 조건 불충족이어도 PID로 인계

// ================= ★ [0730-2] 주행 중 3점 중앙값 필터 ★ =================
const bool SPEED_MEDIAN_ON = true;
const uint8_t SPEED_MEDIAN_N = 3;   // 3 고정 (median3 함수가 3점 전용)


// ================= 좌/우 PID 상태 (완전 분리) =================
const uint8_t LEFT  = 0;   // 왼쪽  : 펄스 21 피드백 -> PWM 8
const uint8_t RIGHT = 1;   // 오른쪽 : 펄스 20 피드백 -> PWM 10

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터
int   noiseCnt[2]    = {0, 0};   // [0730-2] 연속 허수 펄스 주기 (블랭킹 중엔 세지 않음)

// [0730-2] 기동 블랭킹 상태
bool          launching[2] = {false, false};
unsigned long launch_t[2]  = {0, 0};   // 블랭킹 진입 시각 (타임아웃용)
int           settleCnt[2] = {0, 0};   // 정상 펄스 연속 주기 카운터

// [0730-2] 중앙값 필터용 링버퍼 (정상 판정된 값만 들어간다)
int     spdBuf[2][SPEED_MEDIAN_N] = {{0, 0, 0}, {0, 0, 0}};
uint8_t spdPos[2] = {0, 0};


// ================= 구동 모드 (좌/우 독립) =================
const uint8_t DRIVE_PULSE = 0;   // 펄스 목표 PID 주행 (0~15)
const uint8_t DRIVE_PWM   = 1;   // 직접 PWM 출력 (16~255, 무보호)

uint8_t driveMode[2] = {DRIVE_PULSE, DRIVE_PULSE};
int     pwmDirect[2] = {0, 0};   // 직접 PWM 모드에서 출력할 값


// ================= 인휠 상태 =================
volatile long encCountL = 0;    // 왼쪽 펄스 (21번 핀)
volatile long encCountR = 0;    // 오른쪽 펄스 (20번 핀)
int target[2] = {0, 0};         // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)
int rawSpeed[2] = {0, 0};       // [0730-2] 필터 전 원시 펄스 (진단용)
int useSpeed[2] = {0, 0};       // [0730-2] PID가 실제로 본 값 = 텔레메트리로 보내는 값
unsigned long wheel_t = 0;


// ================= ★[1017-4] 상위의 구동 명령 · 보드 안 차단★ =================
// cmd_drive : 상위가 마지막으로 보낸 구동값 — 입력 첫·둘째 칸 (1016_A 의 applySide 에 넣던 값 그대로 —
//             0~15 펄스 / 16~255 직접 PWM / 256 이상 정지).
// drive_cut : 지금 보드가 인휠을 끊고 있는가 (estop_active || brakelight_on, FW_DRIVE_CUT 일 때만)
long cmd_drive[2] = {0, 0};
bool drive_cut    = false;


// ================= ★[0904-4] 무입력 워치독 (통신 끊김 → 인휠 정지)★ =================
// 근거·설계는 1016_A 머리말과 updateRxWatchdog() 주석. ★3초 = 상위 keepalive 1초의 세 배★ —
//   상위 nxde/arduino.py 의 KEEPALIVE_S 를 늘리려면 이 값도 함께 늘린다(셋 배 이상 유지).
// ★[1017-2] 무엇을 '입력' 으로 세나 — 완전한 줄 하나★ A 줄·B 줄·핸드셰이크·형식 오류 줄 모두.
const unsigned long RX_TIMEOUT_MS = 3000;

unsigned long rx_t    = 0;       // 마지막으로 '완전한 줄'을 받은 시각
bool          rx_lost = false;   // 지금 무입력 정지 상태인가


// ============================================================
//  ===================  B : 조향·제동 (1016_B 그대로)  ===================
// ============================================================

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
// ⚠️ [1017-1] 조향 가변저항이 A10 으로 옮겨 갔다 — ★배선을 옮기면서 링키지를 만졌다면 'a' 캘리브를 다시 한다★
//   (이 보드는 B보드의 EEPROM 을 물려받지 않는다 — 새 보드면 코드값으로 시작한다).
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

// ★[1017-2] 조향 칸 다시 걸기★ 직전에 건 조향 칸과 같으면 STEER_REARM_MS 가 지나기 전에는 다시 걸지 않는다.
//   0.9초 = 상위 keepalive(1초)보다 조금 짧게 — 1016_B 가 keepalive 마다 다시 걸던 것을 놓치지 않게.
//   (줄 도착 간격이 keepalive 1.00 s 보다 몇 ms 짧게 찍혀도 다음 줄에서 걸린다.)
//   SETTLE_MS(500ms) 보다는 길어야 한다 — 그래야 다시 걸기 사이에 도달 판정이 성립한다.
const unsigned long STEER_REARM_MS = 900;
bool steer_seen         = false;   // 직전에 건 조향 칸이 있는가 (E-stop·캘리브레이션 뒤에는 지운다)
bool steer_seen_release = false;   // 그 칸이 'x' 였는가
int  steer_seen_angle   = 0;       // 그 칸의 각도 (클램프 뒤)
unsigned long steer_arm_t = 0;     // 그 칸을 건 시각


// ================= ★[1016] 브레이크 : 시간 제어 2단★ =================
const uint8_t BRAKE_NONE = 0;
const uint8_t BRAKE_SOFT = 1;
const uint8_t BRAKE_FULL = 2;
const uint8_t BRAKE_LEVEL_MAX = 2;
// 이동이 도중에 끊겨(E-stop·수동 전환·캘리브레이션) 리니어가 어디 있는지 모른다
const uint8_t BRAKE_UNKNOWN = 255;

// ★[1016] 단계 전이 시간 [ms] — 고치는 곳은 여기뿐★ (PWM 은 BRAKE_PWM 255 고정)
const unsigned long LIN_MS_0_1 = 200;   // 0 → 1단 : 나옴
const unsigned long LIN_MS_0_2 = 400;   // 0 → 2단 : 나옴
const unsigned long LIN_MS_1_2 = 300;   // 1 → 2단 : 나옴
const unsigned long LIN_MS_1_0 = 500;   // 1 → 0단 : 들어감
const unsigned long LIN_MS_2_0 = 500;   // 2 → 0단 : 들어감
//   2 → 1단 = 2 → 0단(LIN_MS_2_0 들어감) 뒤 0 → 1단(LIN_MS_0_1 나옴)

const int BRAKE_PWM = 255;              // ★체결·해제의 세기★ — 하나뿐이다

uint8_t brake_cmd_level = BRAKE_NONE;   // 상위에서 지시한 단계
uint8_t brake_level     = BRAKE_NONE;   // 지금 있다고 보는 단계 (BRAKE_UNKNOWN = 모름)
int     brake_output    = 0;            // 진단용 (지금 리니어에 내는 PWM)

//  LIN_MOVE 는 구간이 최대 둘이다(2 → 1단 : 들어감 → 나옴). 첫 구간이 끝나면 둘째로 잇는다.
enum LinState { LIN_IDLE, LIN_MOVE };
LinState lin_state = LIN_IDLE;
uint8_t  lin_tgt_level = BRAKE_NONE;
unsigned long lin_end_t = 0;      // 지금 구간이 끝나는 시각
uint8_t  lin_dir2 = HIGH;         // 둘째 구간 방향
unsigned long lin_ms2 = 0;        // 둘째 구간 시간 (0 = 없음)


// ================= E-stop ([0904-1] 확인시간 100ms, 발동·해제 대칭) =================
// 발동 : 100ms 연속 개방(HIGH) / 해제 : 100ms 연속 단락(LOW).
//   ★[1007-1] 둘로 갈랐다★
//     estop_line   : D12 확인 결과. 모드와 무관하게 updateEstop 이 늘 갱신한다.
//     estop_active : ★효력★ = estopEffective() — loop 가 매 루프 다시 계산한다.
bool estop_line   = false;
bool estop_active = false;
bool estop_latched = false;
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 100;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 100;
unsigned long estop_high_t = 0;
unsigned long estop_low_t  = 0;


// ================= 주행모드 스위치 =================
bool auto_mode = false;
uint8_t mode_pin_last = HIGH;
unsigned long mode_change_t = 0;
const unsigned long MODE_CONFIRM_MS = 50;

// ★[1015-1] E-stop 의 효력★ — 판정의 단일 소유자.
bool estopEffective() {
  return estop_line && (auto_mode || E_STOP_TEST);
}


// ================= ★[0813-2] 캘리브레이션★ =================
const unsigned long CAL_WATCH_MS = 1200;   // 이 시간 동안 지켜보며 최빈값을 잡는다
const int  CAL_BIN      = 8;               // 최빈값 히스토그램 칸 폭 (잡음 ±10 을 묶는다)
const int  CAL_BIN_LO   = 200;             // 히스토그램 범위 (조향 pot 의 현실적 구간)
const int  CAL_BIN_HI   = 840;
const uint8_t CAL_BINS  = (CAL_BIN_HI - CAL_BIN_LO) / CAL_BIN + 1;

bool cal_running = false;
unsigned long cal_end_t = 0;
uint8_t cal_hist[CAL_BINS];

// EEPROM 배치 : [매직 2B][좌 2B][우 2B][체크섬 1B]   (A 쪽은 EEPROM 을 쓰지 않는다 — 겹칠 곳이 없다)
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

// ★진단이 필요할 때만 true★ 출력 양식("S,"·"P,")을 지키기 위해 기본은 false.
//   '#' 로 시작하므로 파서는 무시한다(이 저장소의 관례).
const bool DEBUG_B = false;


// ============================================================
//  ===================  공통 : 시리얼 · 핸드셰이크  ===================
// ============================================================
unsigned long tele_t = 0;

// ================= 시리얼 입력 버퍼 =================
// 가장 긴 정상 줄은 "255,255,-40,2" (13자). 1016_A 의 48 을 그대로 둔다.
char rxBuf[48];
uint8_t rxLen = 0;

// ================= ★[0904-3] 연결확인 핸드셰이크 ('-' → "YES")★ =================
// 상위(nxde/arduino.py identify_port, nxde/check.py)가 ★PC→보드 방향이 실제로 살아 있는지★ 확인하는
// 유일한 수단이다. '-' 한 줄 → "YES" 한 줄을 즉시 찍고, 이 시간 동안 평상시 텔레메트리를 멈춘다
// (단 E-stop 이면 P, 는 낸다 — [1017-3]). ★millis() 로만 센다★ 유예 중에도 제어는 평소대로 돈다.
const unsigned long HANDSHAKE_HOLD_MS = 2000;
bool          handshake_hold = false;   // 지금 유예 중인가
unsigned long handshake_t    = 0;       // "YES" 를 찍은 시각


// ================= 선언 =================
int  readPotMedian();
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
void linAbort();
void updateBrakeLight();
bool isSignedNumber(const char* s);
bool isUnsignedNumber(const char* s);
bool isReleaseToken(const char* s);
bool isCalToken(const char* s);
bool isHandshakeToken(const char* s);
void handleLine(char* line, unsigned long now);
void handleCommandLine(char* line, unsigned long now);
void pollSerial(unsigned long now);
void updateMode(unsigned long now);
void retractForManual(unsigned long now);
void applyDriveCommand(unsigned long now);
void updateDriveCut(unsigned long now);
void sendOutput(unsigned long now);
int  readSteerAngle();
void startCal(unsigned long now);
void updateCal(unsigned long now);
bool loadCal();
void saveCal();


// ============================================================
//  ===================  A : 인휠 함수 (1016_A 그대로)  ===================
// ============================================================

// ================= ISR (21번=왼쪽 펄스, 20번=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


// ================= [0730-1] 쓰로틀 중앙값 필터 =================
int readThrottleMedian() {
  int s[THROTTLE_MEDIAN_N];
  for (uint8_t i = 0; i < THROTTLE_MEDIAN_N; i++) {
    int v = analogRead(THROTTLE_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[THROTTLE_MEDIAN_N / 2];
}


// ================= [0730-2] 주행펄스 3점 중앙값 (정렬 없이 비교 3회) =================
int median3(int a, int b, int c) {
  int lo = min(a, b);
  int hi = max(a, b);
  return max(lo, min(hi, c));
}

void pushSpeed(uint8_t idx, int v) {
  spdBuf[idx][spdPos[idx]] = v;
  spdPos[idx] = (spdPos[idx] + 1) % SPEED_MEDIAN_N;
}

void fillSpeed(uint8_t idx, int v) {
  for (uint8_t k = 0; k < SPEED_MEDIAN_N; k++) spdBuf[idx][k] = v;
  spdPos[idx] = 0;
}


// ================= 인휠 FF 보간 (목표펄스 -> PWM, 3점 라그랑주 2차보간) =================
float interpFF(float x) {
  if (x <= 0) return 0.0;
  if (x <= ffPulseTable[0]) {
    return ffPwmTable[0] * (x / ffPulseTable[0]);
  }
  if (x >= ffPulseTable[FF_TABLE_N - 1]) {
    return ffPwmTable[FF_TABLE_N - 1];
  }

  int i = 0;
  while (i < FF_TABLE_N - 2 && x > ffPulseTable[i + 1]) i++;

  int i0, i1, i2;
  if (i == 0) { i0 = 0; i1 = 1; i2 = 2; }
  else        { i0 = i - 1; i1 = i; i2 = i + 1; }

  float x0 = ffPulseTable[i0], x1 = ffPulseTable[i1], x2 = ffPulseTable[i2];
  float y0 = ffPwmTable[i0],   y1 = ffPwmTable[i1],   y2 = ffPwmTable[i2];

  float L0 = (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2));
  float L1 = (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2));
  float L2 = (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1));

  return y0 * L0 + y1 * L1 + y2 * L2;
}


// ================= [0730-2] 기동 블랭킹 진입/종료 =================
void enterLaunch(uint8_t idx, unsigned long now) {
  launching[idx] = true;
  launch_t[idx] = now;
  settleCnt[idx] = 0;
  noiseCnt[idx] = 0;
  pidI[idx] = 0;
  pidLastErr[idx] = 0;
  pidCoasting[idx] = false;   // 블랭킹 중엔 코스트/캐치 판정을 아예 하지 않는다
  runawayCnt[idx] = 0;
}

void exitLaunch(uint8_t idx, int speed) {
  launching[idx] = false;
  settleCnt[idx] = 0;
  noiseCnt[idx] = 0;
  pidI[idx] = 0;
  pidLastErr[idx] = target[idx] - speed;
}

// ================= [0730-2] 기동 블랭킹 중 개루프 출력 =================
int launchPwm(uint8_t idx) {
  int ffPwm = (int)interpFF((float)target[idx]);
  if (ffPwm > PWM_MAX) ffPwm = PWM_MAX;

  int pwm = pidLastPwm[idx] + PWM_SLEW_MAX;
  if (pwm > ffPwm) pwm = ffPwm;
  if (pwm < 0) pwm = 0;

  pidLastPwm[idx] = pwm;
  return pwm;
}


// ================= 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =================
int updatePid(uint8_t idx, int tgt, int speed) {
  int err = tgt - speed;
  int d = err - pidLastErr[idx];
  pidLastErr[idx] = err;

  float ff = interpFF((float)tgt);

  // 폭주 감지: 지속 과속이면 코스트 진입(PWM 0)
  if (err <= -RUNAWAY_ERR_OVER) {
    runawayCnt[idx]++;
    if (runawayCnt[idx] >= RUNAWAY_CONFIRM_CYCLES) {
      pidCoasting[idx] = true;
      pidI[idx] = 0;
      runawayCnt[idx] = 0;
    }
  } else {
    runawayCnt[idx] = 0;
  }

  // 코스트-캐치: 목표+마진까지 내려오면 PID 재개
  if (pidCoasting[idx] && speed <= tgt + CATCH_MARGIN) {
    pidCoasting[idx] = false;
  }

  int pwm;
  if (pidCoasting[idx]) {
    pwm = 0;
    pidI[idx] = 0;
  } else {
    float iTerm = ki[idx] * pidI[idx];
    pwm = ff + kp[idx] * err + iTerm + kd[idx] * d;

    // anti-windup: 출력 포화 시, 그리고 오차가 클 때(가속 중)는 적분 누적 중단
    if (pwm > 0 && pwm < PWM_MAX && abs(err) < I_ACCUM_ERR_MAX) {
      pidI[idx] += err;
    }

    // 적분 기여분을 ki 값과 무관하게 ±40 pwm로 고정 제한
    iTerm = constrain(ki[idx] * pidI[idx], -40, 40);
    pwm = ff + kp[idx] * err + iTerm + kd[idx] * d;
    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < 0) pwm = 0;
  }

  pwm = constrain(pwm, 0, PWM_MAX);

  // 슬루레이트 제한: pwm 급상승만 제한(관성 오버슈트 방지), 하강은 즉시 반영
  if (pwm > pidLastPwm[idx] + PWM_SLEW_MAX) pwm = pidLastPwm[idx] + PWM_SLEW_MAX;
  pidLastPwm[idx] = pwm;

  return pwm;
}


// ================= [0730-2] 한쪽 바퀴 갱신: 모드 분기 + 허수 필터 + 블랭킹/PID =================
int updateWheelSide(uint8_t idx, int raw, unsigned long now) {
  rawSpeed[idx] = raw;

  // 직접 PWM 모드는 무보호 경로: 받은 값을 그대로 출력 (피드백을 아예 쓰지 않음)
  if (driveMode[idx] == DRIVE_PWM) {
    if (raw <= PULSE_SANITY_MAX) pushSpeed(idx, raw);
    useSpeed[idx] = SPEED_MEDIAN_ON
                      ? median3(spdBuf[idx][0], spdBuf[idx][1], spdBuf[idx][2])
                      : raw;
    return pwmDirect[idx];
  }

  // ── 허수 판정 : 물리적으로 불가능한 값은 중앙값 버퍼에 넣지 않는다 ──
  bool sane = (raw <= PULSE_SANITY_MAX);
  if (sane) {
    pushSpeed(idx, raw);
    noiseCnt[idx] = 0;
    settleCnt[idx]++;
  } else {
    settleCnt[idx] = 0;
    if (!launching[idx]) noiseCnt[idx]++;   // 블랭킹 중엔 노이즈가 정상이라 세지 않음
  }

  useSpeed[idx] = SPEED_MEDIAN_ON
                    ? median3(spdBuf[idx][0], spdBuf[idx][1], spdBuf[idx][2])
                    : (sane ? raw : useSpeed[idx]);
  int speed = useSpeed[idx];

  // ── 기동 블랭킹 : 피드백 무시하고 FF까지 개루프 램프 ──
  if (launching[idx]) {
    int pwm = launchPwm(idx);

    int ffPwm = (int)interpFF((float)target[idx]);
    if (ffPwm > PWM_MAX) ffPwm = PWM_MAX;
    bool rampDone = (pwm >= ffPwm);
    bool settled  = (settleCnt[idx] >= LAUNCH_SETTLE_CYCLES);
    bool timeout  = (now - launch_t[idx] >= LAUNCH_MAX_MS);

    if ((rampDone && settled) || timeout) {
      exitLaunch(idx, speed);
    }
    return pwm;
  }

  // ── 센서 이상 실패안전 : 허수가 계속 들어오면 코스트로 내린다 ──
  if (NOISE_HOLD_MAX_CYCLES > 0 && noiseCnt[idx] >= NOISE_HOLD_MAX_CYCLES) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
    pidLastPwm[idx] = 0;
    noiseCnt[idx] = 0;
    return 0;
  }

  return updatePid(idx, target[idx], speed);
}


// ================= ★[0904-4] 구동 상태 전체 초기화 (공용)★ =================
// 무입력 워치독이 쓴다. ★복제하지 않는다★ — 초기화할 항목이 열 가지가 넘어서, 한쪽에만 항목을
// 추가하면 '해제 후 재개 시 잔재' 라는 같은 버그가 한쪽에서만 되살아난다.
// ★PWM 출력을 끊는 것은 부르는 쪽의 책임이다★ (이 함수는 상태만 본다).
void resetDriveState() {
  // 양쪽 구동 상태 초기화 (직접 PWM 모드도 강제 해제, 해제 후 재개 시 잔재 방지)
  for (uint8_t s = 0; s < 2; s++) {
    target[s] = 0;
    driveMode[s] = DRIVE_PULSE;
    pwmDirect[s] = 0;
    pidI[s] = 0;
    pidLastErr[s] = 0;
    pidCoasting[s] = false;
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 0부터 다시 램프업)
    runawayCnt[s] = 0;
    launching[s] = false;
    settleCnt[s] = 0;
    noiseCnt[s] = 0;
    cmd_drive[s] = 0;    // ★[1017-4]★ 링크가 돌아온 뒤 옛 명령이 차단 해제로 되살아나지 않게
    // ※ 중앙값 버퍼(spdBuf)는 일부러 건드리지 않는다. 0으로 채워버리면 '바퀴가 멈춤'으로 오인되어,
    //   아직 관성으로 굴러가는 상태에서 복구 직후 블랭킹(개루프 가속)에 진입할 수 있다.
  }
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
  wheel_t = millis();
}


// ================= ★[0904-4] 무입력 워치독 (매 루프 호출)★ =================
// pollSerial 바로 뒤에 불린다 — 이번 루프에 들어온 줄이 즉시 반영되도록.
// ★진입은 엣지에서 한 번만 처리한다★ 매 루프 초기화를 반복하면 관성으로 굴러가는 동안 encCount 를
//   계속 지워, 중앙값 버퍼가 '멈춤'으로 채워진다. 그러면 복구 직후 블랭킹에 잘못 진입한다.
// ★진입 뒤에는 updateWheel 을 그대로 돌린다★ — target 0 에서 PID 경로는 반드시 PWM 0 을 내고,
//   텔레메트리(계측)가 계속 흐르는 편이 원인을 찾기에 낫다.
// ★조향·제동은 건드리지 않는다★ (1016_B 에는 워치독이 없었다 — 마지막 명령을 문 채로 안전하다).
void updateRxWatchdog(unsigned long now) {
  if (now - rx_t <= RX_TIMEOUT_MS) {
    rx_lost = false;         // 정상 — 줄이 제때 오고 있다
    return;
  }
  if (rx_lost) return;       // 이미 처리한 단절이다 (엣지에서 한 번만)

  rx_lost = true;
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);
  resetDriveState();
}


// ================= 값 파싱 (A 구동값) =================
long parseValue(const char* s) {
  if (!isUnsignedNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}


// ================= 펄스 모드 목표 적용 =================
void setPulseTarget(uint8_t idx, int newTarget, unsigned long now) {
  if (driveMode[idx] == DRIVE_PWM) {
    driveMode[idx] = DRIVE_PULSE;
    pwmDirect[idx] = 0;
    pidI[idx] = 0;
    pidLastErr[idx] = 0;
    pidLastPwm[idx] = 0;      // 슬루 기준점 리셋: 캐치 후 0부터 다시 램프업
    runawayCnt[idx] = 0;
    noiseCnt[idx] = 0;
    target[idx] = newTarget;

    if (newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
      enterLaunch(idx, now);
    } else {
      launching[idx] = false;
      pidCoasting[idx] = true;
    }
    return;
  }

  // ★ [0730-2] 기동 블랭킹 진입 판정 : 멈춘 상태(0)에서 처음 양수 목표를 받을 때만 ★
  if (target[idx] == 0 && newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
    target[idx] = newTarget;
    enterLaunch(idx, now);
    return;
  }

  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제 (좌우 개별)
  if (newTarget < target[idx]) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
    if (newTarget == 0) launching[idx] = false;   // 정지 명령은 블랭킹보다 우선
  } else if (newTarget > target[idx]) {
    pidCoasting[idx] = false;
  }
  target[idx] = newTarget;
}


// ================= 값 -> 한쪽 바퀴 적용 =================
void applySide(uint8_t idx, long v, unsigned long now) {
  if (v <= TARGET_MAX) {
    setPulseTarget(idx, (int)v, now);
  } else if (v <= PWM_DIRECT_MAX) {
    driveMode[idx] = DRIVE_PWM;
    pwmDirect[idx] = (int)v;
    launching[idx] = false;   // 직접 PWM은 무보호 경로 — 블랭킹도 적용하지 않는다
  } else {
    setPulseTarget(idx, 0, now);
  }
}


// ================= ★[1017-4] 상위 명령 또는 차단(0)을 인휠에 건다★ =================
// 상위의 구동 줄이 올 때와 차단이 바뀔 때 부른다. 1016_A 는 줄이 올 때마다 applySide 를 불렀다 —
// 같은 값을 다시 거는 것은 1016_A 의 keepalive 와 같은 일이다(목표가 같으면 아무것도 안 바뀐다).
void applyDriveCommand(unsigned long now) {
  applySide(LEFT,  drive_cut ? 0 : cmd_drive[LEFT],  now);
  applySide(RIGHT, drive_cut ? 0 : cmd_drive[RIGHT], now);
}

// ★[1017-4] 차단 판정 — 상태 칸([1016-C])과 같은 식★ updateBrakeLight 뒤에 부른다(이번 루프의 지령이
//   반영된 brakelight_on 을 본다). 바뀐 순간에만 손댄다.
void updateDriveCut(unsigned long now) {
  extern bool brakelight_on;
  bool cut = FW_DRIVE_CUT && (estop_active || brakelight_on);
  if (cut == drive_cut) return;
  drive_cut = cut;
  if (cut) {
    // 다음 20ms 창을 기다리지 않고 그 자리에서 끊는다(그 뒤 창에서도 코스트라 0 이 나온다)
    analogWrite(PWM_PIN_L, 0);
    analogWrite(PWM_PIN_R, 0);
  }
  applyDriveCommand(now);
}


// ================= 인휠 제어 (좌우 독립: 펄스 PID 또는 직접 PWM) =================
void updateWheel(unsigned long now) {
  if (now - wheel_t < CONTROL_WINDOW_MS) return;
  wheel_t += CONTROL_WINDOW_MS;

  noInterrupts();
  long cL = encCountL;
  long cR = encCountR;
  encCountL = 0;
  encCountR = 0;
  interrupts();

  // [0730-1] 쓰로틀 페달 계측 (제어에는 쓰지 않고 텔레메트리로만 보고)
  throttleRaw = readThrottleMedian();

  int pwmL = updateWheelSide(LEFT,  (int)cL, now);   // 왼쪽 : 펄스 21 -> PWM 8
  int pwmR = updateWheelSide(RIGHT, (int)cR, now);   // 오른쪽 : 펄스 20 -> PWM 10

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ============================================================
//  ===================  B : 조향·제동 함수 (1016_B 그대로)  ===================
// ============================================================

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
  int s[9];                       // POT_MEDIAN_N 이하
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


// ================= ★[1016] 브레이크 : 시간 제어 2단★ =================
// 단계 이동 시작. ★위치를 보지 않는다★ — 지금 단계(brake_level)와 목표 단계로 시간표를 고른다.
//   지금 단계를 모르면(BRAKE_UNKNOWN) : 0단 = 끝까지 들어감 / 2단 = 0→2 시간만큼 나옴(더 밀려도
//   2단 쪽 끝에 닿을 뿐이다) / 1단 = 먼저 끝까지 들어간 뒤 0→1.
void startLinMove(uint8_t lv, unsigned long now) {
  uint8_t from = brake_level;
  unsigned long home_ms = (LIN_MS_2_0 > LIN_MS_1_0) ? LIN_MS_2_0 : LIN_MS_1_0;

  lin_tgt_level = lv;
  lin_ms2 = 0;
  if (from == lv) {                  // 이미 그 단계 — 구동할 것이 없다
    linearStop();
    lin_state = LIN_IDLE;
    return;
  }

  uint8_t dir;
  unsigned long ms;
  if (lv == BRAKE_NONE) {
    dir = LINEAR_REV;
    ms  = (from == BRAKE_SOFT) ? LIN_MS_1_0 : (from == BRAKE_FULL) ? LIN_MS_2_0 : home_ms;
  } else if (lv == BRAKE_FULL) {
    dir = LINEAR_FWD;
    ms  = (from == BRAKE_SOFT) ? LIN_MS_1_2 : LIN_MS_0_2;
  } else if (from == BRAKE_NONE) {   // 0 → 1단
    dir = LINEAR_FWD;
    ms  = LIN_MS_0_1;
  } else {                           // 2 → 1단 (또는 모름 → 1단) : 끝까지 들어갔다가 0 → 1단
    dir = LINEAR_REV;
    ms  = (from == BRAKE_FULL) ? LIN_MS_2_0 : home_ms;
    lin_dir2 = LINEAR_FWD;
    lin_ms2  = LIN_MS_0_1;
  }

  lin_end_t = now + ms;
  linearDrive(dir, BRAKE_PWM);
  lin_state = LIN_MOVE;
}

// 구동 중 종료 판정. ★시간 하나뿐이다★ 첫 구간이 끝나면 둘째 구간이 있으면 잇는다.
void updateLinear(unsigned long now) {
  if (lin_state != LIN_MOVE) return;
  if ((long)(now - lin_end_t) < 0) return;

  if (lin_ms2 > 0) {
    lin_end_t = now + lin_ms2;
    linearDrive(lin_dir2, BRAKE_PWM);
    lin_ms2 = 0;
    return;
  }
  linearStop();
  lin_state = LIN_IDLE;
  brake_level = lin_tgt_level;
}

// 진행 중인 이동을 끊는다. ★끊긴 자리는 모른다★ → BRAKE_UNKNOWN (다음 이동이 그에 맞게 고른다)
void linAbort() {
  if (lin_state == LIN_IDLE) return;
  linearStop();
  lin_state = LIN_IDLE;
  lin_ms2 = 0;
  brake_level = BRAKE_UNKNOWN;
}

// 지시 단계와 지금 단계가 다르면 이동을 건다(구동 중에는 끝난 뒤에).
void updateBrake(unsigned long now) {
  if (lin_state != LIN_IDLE) return;
  if (brake_cmd_level != brake_level) startLinMove(brake_cmd_level, now);
}

// ★[1016-B] 제동등 : A5 스위치 닫힘(LOW) 또는 1·2단 지령★ — 어떤 분기에도 걸지 않고 매 루프 부른다.
bool brakelight_on = false;   // [1016-C] 텔레메트리 상태 1 의 출처 · ★[1017-4] 구동 차단의 출처★
void updateBrakeLight() {
  bool on = (digitalRead(BRAKE_SW_PIN) == LOW)
         || brake_cmd_level == BRAKE_SOFT || brake_cmd_level == BRAKE_FULL;
  brakelight_on = on;
  digitalWrite(BRAKELIGHT_PIN, on ? HIGH : LOW);
}


// ================= E-stop =================
// ★[1007-1] 이 함수는 D12 라인만 판정한다(estop_line)★ 모드는 보지 않는다.
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_line   = false;
    estop_high_t = 0;
    estop_low_t  = 0;
    return;
  }

  bool open_now = (digitalRead(ESTOP_PIN) == HIGH);   // NC 개방 = e-stop 요청

  if (!estop_line) {
    estop_low_t = 0;
    if (!open_now) {
      estop_high_t = 0;
    } else if (estop_high_t == 0) {
      estop_high_t = now;
    } else if (now - estop_high_t >= ESTOP_TRIGGER_CONFIRM_MS) {
      estop_line   = true;
      estop_high_t = 0;
    }
  } else {
    estop_high_t = 0;
    if (open_now) {
      estop_low_t = 0;
    } else if (estop_low_t == 0) {
      estop_low_t = now;
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_line  = false;
      estop_low_t = 0;
    }
  }
}

// ★E-stop = 2단 체결★ 조향은 힘빼기, 브레이크는 최대. (인휠 차단은 [1017-4] updateDriveCut)
void applyEstop(unsigned long now) {
  dcStop();
  steer_angle_cmd = 0;

  if (!estop_latched) {
    estop_latched = true;
    cal_running = false;              // 캘리브레이션 중이었다면 취소
    linAbort();                       // 진행 중이던 이동을 끊고 다시 건다 ([1016] 자리는 모름)
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
  linAbort();                         // [1016] 이동 중이었다면 끊는다 — 끝난 뒤 updateBrake 가 다시 건다
  steer_state = ST_SETTLED;           // 지켜보는 동안 조향은 정지
  settleTimerRunning = false;
  // ★항상 찍는다★ 사람이 손으로 하는 일회성 조작이라 확인이 필요하다. '#' 줄은 상위가 버린다.
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
  steer_seen = false;                 // ★[1017-2]★ 1016_B 는 끝난 뒤 다음 B 줄(≤1초)에 다시 걸었다
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


// ================= 주행모드 스위치 =================
// ★[1007-1]★ auto_mode 가 E-stop 의 효력과 브레이크 지시 수용을 정한다. 자율→수동으로 확정되는
//   순간 리니어를 0단으로 되돌린다([1007-2]).
void updateMode(unsigned long now) {
  uint8_t lv = digitalRead(MODE_PIN);

  if (lv != mode_pin_last) {
    mode_pin_last = lv;
    mode_change_t = now;
  } else if (mode_change_t != 0 && now - mode_change_t >= MODE_CONFIRM_MS) {
    bool was_auto = auto_mode;
    auto_mode = (lv == LOW);
    mode_change_t = 0;
    // ★[1015-1] E-stop 이 효력을 갖는 동안에는 빼지 않는다★ (E_STOP_TEST = true 에서만 생긴다)
    if (was_auto && !auto_mode && !estopEffective()) retractForManual(now);
  }
}

// ★[1007-2] 수동조종으로 들어가면 리니어를 끝까지 넣어 둔다★ (자율→수동 엣지 · 수동 부팅)
void retractForManual(unsigned long now) {
  linAbort();
  brake_cmd_level = BRAKE_NONE;
  brake_level     = BRAKE_UNKNOWN;    // [1016] 어디 있든 끝까지 넣는다 (들어감 LIN_MS_2_0·1_0 중 긴 쪽)
  startLinMove(BRAKE_NONE, now);
  if (DEBUG_B) Serial.println(F("# MANUAL — 리니어 0단 복귀"));
}


// ============================================================
//  ===================  공통 : 입력 ([1017-2])  ===================
// ============================================================

// ================= 입력 형식 검사 =================
// B 조향각용 — 부호 허용 (1016_B isValidNumber)
bool isSignedNumber(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}

// A 구동값용 — 부호 없는 정수만 (1016_A isValidNumber)
bool isUnsignedNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}

bool isReleaseToken(const char* s) {
  return s && (s[0] == 'x' || s[0] == 'X') && s[1] == '\0';
}

bool isCalToken(const char* s) {
  return s && (s[0] == 'a' || s[0] == 'A') && s[1] == '\0';
}

// ★[0904-2/3] 핸드셰이크 토큰★ '-' ★한 글자로만★ 이루어진 줄에서만 성립한다.
bool isHandshakeToken(const char* s) {
  return s && s[0] == '-' && s[1] == '\0';
}


// ================= ★[1017-2] 명령 줄 "<왼쪽값>,<오른쪽값>,<명령조향각>,<브레이크단수>"★ =================
// 앞 두 칸 = 1016_A 콤마 2값 파서, 뒤 두 칸 = 1016_B 파서. ★네 칸 형식이 다 맞아야★ 어느 쪽이든 건다.
void handleCommandLine(char* line, unsigned long now) {
  if (strchr(line, ' ')) return;   // 공백 포함 줄은 형식 오류 (1016_A 와 같다)

  // ── 정확히 네 칸으로 자른다 (strtok 은 빈 칸 "1,,2" 를 건너뛰어 칸을 밀어 버리므로 쓰지 않는다) ──
  char* tok[4];
  uint8_t n = 0;
  tok[n++] = line;
  for (char* p = line; *p != '\0'; p++) {
    if (*p == ',') {
      if (n >= 4) return;          // 다섯 칸 이상 → 형식 오류
      *p = '\0';
      tok[n++] = p + 1;
    }
  }
  if (n != 4) return;              // 네 칸이 아니다 (옛 A 의 "7" · "3,3" · 옛 B 의 "-12,0" 도 여기서 버린다)

  // ── 형식 검사를 먼저 다 하고, 하나라도 틀리면 아무것도 걸지 않는다 ──
  long vL = parseValue(tok[0]);    // 부호 없는 정수만 (1016_A)
  long vR = parseValue(tok[1]);
  if (vL < 0 || vR < 0) return;

  bool release = isReleaseToken(tok[2]);
  if (!release && !isSignedNumber(tok[2])) return;
  if (!isSignedNumber(tok[3])) return;

  // ── 구동 (1016_A) — ★[1017-4] cmd_drive 에 받아 두고 applyDriveCommand 로 건다★ ──
  //   차단 중이면 0 이 걸리고, 풀리면 이 값이 걸린다.
  cmd_drive[LEFT]  = vL;
  cmd_drive[RIGHT] = vR;
  applyDriveCommand(now);

  // ── 조향·브레이크 (1016_B) ──
  // e-stop 중에는 조향/브레이크 미적용 → 리니어 재구동 방지 (estop_active = 효력)
  // 캘리브레이션 중에도 받지 않는다 — 핸들을 사람이 잡고 있는 상황이다
  //   ★받지 않은 동안은 조향 기억을 지운다★ — 풀린 뒤 첫 줄은 같은 값이어도 곧바로 건다.
  if (estop_active || cal_running) {
    steer_seen = false;
    return;
  }

  int angle = release ? 0 : constrain(atoi(tok[2]), STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  bool same = steer_seen && (release == steer_seen_release) && (release || angle == steer_seen_angle);
  if (!same || (now - steer_arm_t) >= STEER_REARM_MS) {
    if (release) {
      releaseSteer();
    } else {
      steer_angle_cmd = angle;
      target_pos = angleToPot(steer_angle_cmd);
      settleTimerRunning = false;
      steer_state = ST_ACTIVE;
    }
    steer_seen         = true;
    steer_seen_release = release;
    steer_seen_angle   = angle;
    steer_arm_t        = now;
  }

  int brake = atoi(tok[3]);
  // ★[1007-2] 수동조종에서는 브레이크 지시를 받지 않는다 — 리니어는 들어간 채로 둔다★
  if (!auto_mode) brake = BRAKE_NONE;

  if (brake >= BRAKE_NONE && brake <= BRAKE_LEVEL_MAX) {
    brake_cmd_level = (uint8_t)brake;
  }
}


// ================= 한 줄 처리 ([1017-2]) =================
void handleLine(char* line, unsigned long now) {
  // ★"-" 한 줄 = 연결확인 핸드셰이크★ 구동·조향·제동에는 아무 영향이 없다.
  //   ★e-stop 중에도, 캘리브레이션 중에도 받는다★ (1016_A·B 와 같다)
  if (isHandshakeToken(line)) {
    Serial.println("YES");
    handshake_hold = true;
    handshake_t = now;
    return;
  }

  // ★"a" 한 줄 = 조향 영점 캘리브레이션★ (E-stop 중에는 받지 않는다 — 1016_B 와 같다)
  if (isCalToken(line)) {
    if (!estop_active) startCal(now);
    return;
  }

  handleCommandLine(line, now);
}

void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) {
        // ★[0904-4] 워치독 리셋은 handleLine 앞에 둔다★ 형식 오류 줄도 '링크가 살아 있다'는
        //   증거로는 유효하다. 뒤에 두면 형식 오류가 이어질 때 워치독이 오발동한다.
        rx_t = now;
        handleLine(rxBuf, now);
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
}


// ============================================================
//  ===================  공통 : 출력 ([1017-3])  ===================
// ============================================================
// "I,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>,<브레이크단계>,<D5>,<실측조향각>,<상태>" — 50ms 마다 한 줄.
//   상태 : 2 = E-stop 효력 / 1 = 제동등 조건 / 0 = 그 밖. 2 가 이긴다([1016-C]). D5 칸은 실제 스위치.
// ★핸드셰이크 유예 중에는 내보내지 않는다★ — 단 E-stop 이면 낸다(1016_B 규칙).
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (handshake_hold) {
    if (now - handshake_t < HANDSHAKE_HOLD_MS) {
      if (!estop_active) return;   // E-stop 은 유예 중에도 알린다
    } else {
      handshake_hold = false;      // 유예 종료 — 다음 줄부터 원래대로
    }
  }

  int state = estop_active ? 2 : (brakelight_on ? 1 : 0);

  Serial.print("I,");
  Serial.print(useSpeed[LEFT]);
  Serial.print(',');
  Serial.print(useSpeed[RIGHT]);
  Serial.print(',');
  Serial.print(throttleRaw);
  Serial.print(',');
  Serial.print(brake_cmd_level);          // 지시된 단계 0/1/2
  Serial.print(',');
  Serial.print(auto_mode ? 1 : 0);        // D5
  Serial.print(',');
  Serial.print(readSteerAngle());         // ★덧붙인 칸★ 실측 조향각
  Serial.print(',');
  Serial.println(state);                  // ★덧붙인 칸★ 2 E-stop / 1 제동등 / 0
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  // ── A : 인휠 ──
  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), encISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), encISR_R, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // [0730-1] 쓰로틀 페달 (analogRead는 pinMode 없어도 되지만 명시)
  pinMode(THROTTLE_PIN, INPUT);
  throttleRaw = readThrottleMedian();   // 첫 텔레메트리부터 실제값이 나가도록 프라이밍

  // [0730-2] 부팅 시엔 바퀴가 멈춰 있으므로 중앙값 버퍼를 0으로 채워둔다
  fillSpeed(LEFT, 0);
  fillSpeed(RIGHT, 0);

  // ── B : 조향·리니어 ──
  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  pinMode(DC_POT_PIN, INPUT);
  digitalWrite(DC_DIR_PIN, DIR_CW);
  dcStop();

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_line   = false;
  estop_active = false;
  estop_high_t = 0;
  estop_low_t  = 0;

  pinMode(MODE_PIN, INPUT_PULLUP);

  // ★[1016-B] 제동등★ A5 는 스위치 입력(내부 풀업 — 스위치가 GND 로 당긴다), D11 은 출력
  pinMode(BRAKE_SW_PIN, INPUT_PULLUP);
  pinMode(BRAKELIGHT_PIN, OUTPUT);
  digitalWrite(BRAKELIGHT_PIN, LOW);
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
  wheel_t = tele_t = now;

  // ★[0904-4] 워치독 프라이밍★ 부팅 직후를 '무입력' 으로 세지 않는다(1016_A 그대로).
  rx_t = now;
  rx_lost = false;

  // 브레이크 : 부팅 상태는 '단계 미상'이 아니라 0단으로 두고 대기한다.
  brake_level     = BRAKE_NONE;
  brake_cmd_level = BRAKE_NONE;
  lin_state       = LIN_IDLE;

  // ★[1007-2] 수동조종으로 켜지면 리니어를 끝까지 넣고 시작한다★ ★자율로 켜진 경우는 그대로 둔다★
  //   (2단을 물고 서 있던 차가 리셋됐다면 상위가 곧 다시 2단을 지시한다 — 먼저 빼면 그 사이에 풀린다).
  if (!auto_mode) retractForManual(now);

  updateBrakeLight();        // [1016-B] 부팅 순간의 A5 로 바로 판정한다

  // 조향 PD : 시작 시 현재 위치를 목표로 유지 (대기 상태, 급조향 방지)
  int rawInit = readPotMedian();
  lastPotMedian    = rawInit;
  steerAdcFiltered = rawInit;
  target_pos = rawInit;
  prev_pos   = rawInit;
  steer_win_t = now;

  // ★여기서 안내를 출력하지 않는다★ 출력 양식("S,"·"P,")을 그대로 지키기 위함.
}


// ================= loop =================
// 순서 — B 의 판정(E-stop·모드)을 명령보다 앞에(1016_B), 워치독은 pollSerial 바로 뒤(1016_A),
//   제동등·구동 차단은 이번 루프의 지령이 다 반영된 뒤, 인휠은 그 차단을 본 뒤.
void loop() {
  unsigned long now = millis();

  // 진행 중인 리니어 구동의 종료 판정 — e-stop 여부와 무관하게 매 루프
  updateLinear(now);

  // ★E-stop 판정은 pollSerial 보다 앞★ 발동 직후 들어온 줄이 한 번 먹는 틈을 없앤다
  updateEstop(now);          // [1007-1] D12 라인(estop_line) — 모드와 무관

  // ★[1007-1] 주행모드도 pollSerial 보다 앞★ — 자율→수동으로 확정되면 여기서 리니어 0단 복귀
  updateMode(now);

  // ★[1007-1] E-stop 의 효력 — 엣지가 아니라 매 루프 레벨★ ([1015-1] estopEffective)
  estop_active = estopEffective();

  pollSerial(now);

  // ★[0904-4] 무입력 워치독★ pollSerial 뒤 — 이번 루프에 들어온 줄을 먼저 센다. 인휠만 세운다.
  updateRxWatchdog(now);

  if (estop_active) {
    applyEstop(now);
  } else {
    if (estop_latched) {
      estop_latched = false;
      // ★해제는 0단 복귀와 같은 작용★ 송신측이 0을 보내주기를 기다리지 않는다.
      linAbort();                     // 2단 이동이 아직 끝나지 않았으면 끊는다
      brake_cmd_level = BRAKE_NONE;
      brake_level     = BRAKE_FULL;   // 2단 쪽에 있다고 보고 0단으로 내린다 (2→0 들어감)

      // 조향 PD 목표를 해제 시점의 현재 위치로 재동기화
      int cur = readPotMedian();
      lastPotMedian    = cur;
      steerAdcFiltered = cur;
      target_pos = cur;
      prev_pos   = cur;
      steer_state = ST_SETTLED;
      settleTimerRunning = false;
      steer_seen = false;             // ★[1017-2]★ 해제 뒤 첫 줄은 같은 조향값이어도 곧바로 건다
    }

    if (cal_running) {
      updateCal(now);          // 캘리브레이션 중에는 조향·제동을 건드리지 않는다
    } else {
      updateBrake(now);
      updateSteer(now);
    }
  }

  // ★[1016-B] 제동등 : 어떤 분기에도 걸지 않는다★ 이번 루프의 지령이 반영된 뒤에 판정한다.
  updateBrakeLight();

  // ★[1017-4] 상태 2·1 = 인휠 차단★ 제동등 판정 바로 뒤 — 같은 루프의 brakelight_on 을 본다.
  updateDriveCut(now);

  // 인휠 20ms 창 (1016_A 는 E-stop 과 무관하게 늘 돌았다 — 차단은 목표 0 으로 들어가 있다)
  updateWheel(now);

  sendOutput(now);
}
