// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID + 쓰로틀 페달 계측 (Arduino Mega 2560) - 0730 버전
//  kasa_0727_A_nodiff.ino 기반. 0730 변경점은 아래 2가지.
//
//  [0730-1] ★ 쓰로틀 페달 입력(A0) 추가 + 텔레메트리 3필드로 확장 ★
//      가속페달에 달린 가변저항을 A0으로 읽어 눌린 정도를 0~1023 raw로 그대로 보고한다.
//      이 보드는 이 값으로 아무 제어도 하지 않는다 (계측·보고 전용).
//      ROS2(kasa_ws)가 이 값을 보고 '수동조종 모드'에서 주행 목표펄스를 만들어
//      다시 이 보드로 "<좌펄스>,<우펄스>"로 내려보내는 구조.
//        출력 : "S,<왼쪽펄스>,<오른쪽펄스>"  ->  "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>"
//        예)   "S,5,5"                      ->  "S,5,5,446"
//      ※ 필드가 하나 늘었으므로 kasa_ws의 A보드 파서를 반드시 같이 고쳐야 한다.
//      ※ 스파이크 제거를 위해 9샘플 중앙값만 적용(지연 ~1ms). 지수평활은 넣지 않았다 —
//        쓰로틀은 사람이 실시간으로 밟는 입력이라 지연을 최소화하는 편이 낫고,
//        추가 평활이 필요하면 ROS2 쪽에서 하는 것이 튜닝하기 쉬움.
//
//  [0730-2] ★★ 기동 노이즈 대책 : 기동 블랭킹(LAUNCH) + 3점 중앙값 필터 ★★
//      (iw_0730_pid.ino에서 검증한 로직을 그대로 이식. 상세 배경은 그 파일 헤더 참고)
//
//      [문제] "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간에서 홀신호에 수백~1500
//      단위 노이즈가 쏟아진다. 이 모터는 무부하 최대 1320RPM x 64펄스/회전(직결구동)이라
//      20ms 창당 최대 ≈28.2펄스가 물리적 한계이므로 그 이상은 전부 허수다.
//        PWM 60 직접 인가 실측 : 433/720 → 789/1222 → 563/925 → 181/217 → 30/21 →
//                                124/17 → 709/15 → 752/21 → 118/9 → 4/6 → 7/7 → 5/5 → …
//        노이즈 버스트는 약 12~15주기(240~300ms) 지속되고 바퀴가 구르면 말끔히 사라진다.
//      기존 PID는 이 허수를 오차로 먹어(target 15에 speed 666이면 err=-651) pwm을 0으로
//      클램프했고, 하강엔 슬루 제한이 없어(감속 즉시반영은 의도된 안전설계) 쌓아온
//      램프업이 통째로 리셋 → 재램프 → 또 노이즈의 무한루프에 빠졌다.
//
//      [대책 1 : 기동 블랭킹] 정지 상태에서 목표가 0 → 양수로 바뀌면 LAUNCH 상태로
//      들어가 그 구간 동안 피드백을 아예 보지 않는다(개루프).
//        - pwm = FF(target)까지 PWM_SLEW_MAX(+4/cycle)로 램프. PID/코스트/폭주감지 정지
//        - 허수 펄스가 얼마가 들어와도 램프가 멈추거나 되돌아가지 않는다
//        - 이 구간은 애초에 피드백이 필요없다(바퀴가 안 도는 상태에서 할 일은
//          FF 테이블이 실측으로 갖고 있는 정적 PWM까지 밀어붙이기 하나뿐)
//      종료 조건은 아래 둘을 '모두' 만족할 때 (또는 LAUNCH_MAX_MS 타임아웃):
//        (a) pwm이 FF(target)에 도달 (램프 완료)
//        (b) 정상 범위 펄스가 LAUNCH_SETTLE_CYCLES 주기 연속 관측
//      ★ (a)가 반드시 필요하다 ★ (b)만 쓰면 램프 초반(pwm 4·8·12)에는 코일 힘이 약해
//        노이즈가 시작되지도 않아 speed가 "0 0 0 0 0"으로 읽히고, 노이즈가 오기도 전에
//        블랭킹이 조기 종료된다. (a)로 "노이즈가 나올 만큼 힘을 준 뒤"를 보장한다.
//      종료 시 pidLastPwm/pidLastErr를 현재값으로 이어받아(pidI=0) PID로 무단절 인계.
//
//      [대책 2 : 3점 중앙값] 바퀴가 구른 뒤에도 단발성 스파이크(1,2 사이에 갑자기 17)가
//      남는다. 3점 중앙값은 이를 완전히 제거하면서 지연이 1주기(20ms)뿐이라 차량 관성
//      시정수 대비 무시할 수준이다. 허수 판정(> PULSE_SANITY_MAX)된 값은 버퍼에 아예
//      넣지 않으므로, 노이즈 주기에는 '직전 정상값이 자동 대입'되는 효과가 난다
//      (그래서 블랭킹이 끝난 뒤에도 pwm이 얼거나 0으로 리셋되지 않는다).
//
//      ※ 텔레메트리 펄스 필드는 '필터 후 값'(PID가 실제로 본 값)을 보낸다. 필드 구성은
//        그대로이므로 kasa_ws 수정은 필요 없다. 원시 허수값을 ROS2 화면에 그대로
//        띄우지 않기 위한 선택이며, 이 보드의 공식 속도 계측값이 필터 후 값이기 때문.
//
//  --- 이하 구조는 0727 nodiff와 동일 ---
//  좌우 차동은 이 보드에 없다. ROS2(kasa_ws)가 담당한다:
//     B보드 텔레메트리 "P,<조향각>,<모드>"  ->  kasa_ws가 좌/우 펄스 계산
//     ->  A보드로 "<좌펄스>,<우펄스>" 콤마 2값 전송  ->  A보드는 그대로 추종만
//  따라서 이 보드는 조향각 가변저항 분기 입력이 필요 없다 (A0은 쓰로틀 전용).
//
//  ★ 배선 (2026-07-30 실차 확인 완료) ★
//     왼쪽  모터 : 펄스 2  -> PID -> PWM 8
//     오른쪽 모터 : 펄스 21 -> PID -> PWM 9
//   좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//   ※ 과거 버전과 홀센서 핀이 다르다 : kasa_0717_A/kasa_0727_A는 21(왼쪽)/20(오른쪽),
//     kasa_0727_A_nodiff는 2/3을 썼다. 현재 확정 배선은 2(왼쪽)/21(오른쪽)이며
//     iw_0730_check.ino(후보 6핀 전수 계측)로 검증되었다.
//
//  입력 : "<값>" 또는 "<왼쪽값>,<오른쪽값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값: 펄스 전용(0~15), 범위 밖/숫자 아님은 무시 (좌우 동일값)
//         - 콤마 2값: 좌/우 독립 — 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지
//           ★ kasa_ws가 차동 주행 시 쓰는 경로가 이것. 0~15를 넘기면 직접 PWM으로
//             오해석되므로 kasa_ws 쪽(setting.PULSE_MAX)에서 반드시 클램프할 것.
//  출력 : "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" (평상시) / "STOP" (e-stop 중)
//  제어주기 : 20ms, 출력주기 : 50ms
//    - FF 보간 테이블(펄스->PWM, 라그랑주 2차보간) + PID(0.4/0.03/0.2, 좌우 게인 별도 관리)
//    - PWM 슬루레이트 제한(+4/cycle) + 조건부 적분(|오차|<4, 기여 ±40 클램프)
//    - 하강 코스트-캐치, 폭주 감지(목표+2펄스 과속 1초 연속 시 코스트)
//    - [0730-2] 기동 블랭킹(개루프 FF 램프) + 허수 펄스 배제 + 3점 중앙값
//      ※ 위 안전장치들은 '펄스 모드'에만 적용. 직접 PWM 모드는 무보호(테스트 전용).
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    ★ 현재 ESTOP_ENABLED = false (2026-07-30, B보드와 동일) ★
//      NC 방식이라 스위치 미배선 상태에서는 13번이 HIGH(개방)로 읽혀 상시 발동해 버린다.
//      실제 배선 후 estop_0713.ino로 확인하고 true로 되돌릴 것. 아래 로직은 그대로 살아 있다.
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) -> e-stop
//    - e-stop 발동 시 직접 PWM 모드도 즉시 해제(펄스 0으로 복귀)
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// ★ 배선 (2026-07-30 실차 확인 완료) ★
//     왼쪽  모터 : 펄스 2  -> PID -> PWM 8
//     오른쪽 모터 : 펄스 21 -> PID -> PWM 9
//   즉 좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//   ※ 좌우가 뒤바뀐 것으로 보이면 이 4줄만 고치면 된다.
//     아래 파서/PID/출력은 전부 LEFT/RIGHT 인덱스로만 동작하므로 자동으로 따라간다.

// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 2;    // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 21;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (펄스 2와 같은 컨트롤러)
const uint8_t PWM_PIN_R = 9;     // 오른쪽 모터 PWM (펄스 21과 같은 컨트롤러)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
// ★ 현재 false (2026-07-30, B보드와 동일) ★ NC 방식이라 스위치가 배선되지 않은 상태에서는
//   INPUT_PULLUP으로 13번이 HIGH(개방)로 읽혀 500ms 후 e-stop이 상시 발동해 버린다.
//   스위치를 실제로 배선한 뒤 estop_0713.ino로 단락/개방을 확인하고 true로 되돌릴 것.
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// --- [0730-1] 쓰로틀(가속) 페달 가변저항 ---
// 이 보드는 이 값으로 제어하지 않는다. 0~1023 raw를 텔레메트리로 보고만 한다.
const uint8_t THROTTLE_PIN = A1;

// ※ 조향각 가변저항 분기 입력은 nodiff 버전에서 제거됨(이 파일도 없음).
//   좌우 차동은 ROS2(kasa_ws)가 B보드 텔레메트리 "P,<각도>,<모드>"를 보고 계산해
//   "<좌펄스>,<우펄스>" 콤마 2값으로 내려보낸다. A보드는 그 값을 그대로 PID로 추종만 한다.


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= [0730-1] 쓰로틀 필터 : 9샘플 중앙값 =================
// ADC 스파이크(모터 노이즈 유입 등)를 '평균에 섞지 않고' 통째로 버리는 것이 목적.
// Mega의 analogRead는 약 104us이므로 9회 ≈ 0.94ms (제어주기 20ms의 5% 수준).
// 지수평활은 일부러 넣지 않음 — 사람이 밟는 입력이라 지연 최소화가 유리하고,
// 추가 평활이 필요하면 ROS2 쪽에서 하는 편이 튜닝하기 쉽다.
const uint8_t THROTTLE_MEDIAN_N = 9;   // 반드시 홀수
int throttleRaw = 0;                   // 최근 중앙값 (0~1023, 텔레메트리용)


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(펄스2-PWM8), 오른쪽(펄스21-PWM9)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ================= ★ 값 해석 경계 ★ =================
const int TARGET_MAX = 15;      // 0~15 = 펄스 목표. 단일 값 입력은 이 범위만 유효
const int PWM_DIRECT_MAX = 255; // 16~255 = 직접 PWM (콤마 2값 형식에서만). 256 이상 = 정지

// ================= ★ 인휠 코스트-캐치 (튜닝 지점) ★ =================
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ================= ★ 인휠 PWM 상한 (튜닝 지점) ★ =================
// ※ 펄스(PID) 모드에만 적용. 직접 PWM 모드(16~255)는 이 상한을 무시하고 그대로 출력.
const int PWM_MAX = 170;

// ================= ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =================
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영). 직접 PWM 모드에는 미적용.
// [0730-2] 기동 블랭킹 중 FF까지 올라가는 개루프 램프도 이 값을 쓴다.
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
// 한쪽 컨트롤러의 과속 특성 대비 안전망: 목표보다 RUNAWAY_ERR_OVER 펄스 이상 과속이
// RUNAWAY_CONFIRM_CYCLES 주기(20ms) 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= ★ [0730-2] 허수 펄스 판정 경계 (튜닝 지점) ★ =================
// 무부하 최대 1320RPM x 64펄스/회전(직결구동) 기준 이론 최대 ≈ 28.2펄스/20ms창.
// 실측 노이즈는 85~1500 수준이라 넉넉히 40으로 잡아도 전부 걸러진다 — 정상값을
// 잘못 버리는 쪽이 더 위험하므로 이론 최대보다 여유를 두었다.
const int PULSE_SANITY_MAX = 40;

// 허수로 판정된 주기가 이만큼 연속되면(블랭킹 종료 후 기준) 홀센서 이상으로 보고
// 코스트(PWM 0) 진입. 0으로 두면 상한 없음. 블랭킹 중에는 노이즈가 정상이라 세지 않는다.
const int NOISE_HOLD_MAX_CYCLES = 50;   // 50주기 = 1초

// ================= ★ [0730-2] 기동 블랭킹 LAUNCH (튜닝 지점) ★ =================
// 목표가 0 -> 양수로 바뀌고 바퀴가 사실상 멈춰 있을 때 진입. 상세는 파일 상단 [0730-2] 참고.
const int LAUNCH_ENTRY_SPEED_MAX = 1;      // 이 펄스 이하일 때만 '정지 상태'로 보고 진입
const int LAUNCH_SETTLE_CYCLES   = 5;      // 정상 펄스가 이만큼 연속되면 노이즈 종료로 판정
const unsigned long LAUNCH_MAX_MS = 3000;  // 이 시간이 지나면 조건 불충족이어도 PID로 인계

// ================= ★ [0730-2] 주행 중 3점 중앙값 필터 ★ =================
// true면 PID가 보는 speed(및 텔레메트리 펄스)를 최근 정상값 3개의 중앙값으로 대체.
// false면 원시값 직결 — 반응성 비교용으로 껐다 켜볼 수 있게 남겨둔 스위치.
const bool SPEED_MEDIAN_ON = true;
const uint8_t SPEED_MEDIAN_N = 3;   // 3 고정 (median3 함수가 3점 전용)


// ================= 좌/우 PID 상태 (완전 분리) =================
// 주의: Arduino IDE는 함수 프로토타입을 파일 맨 위(커스텀 타입 정의보다 앞)에 자동 삽입한다.
// struct로 상태를 묶으면 그 프로토타입이 struct 정의보다 앞에 삽입되어 컴파일 에러가 남.
// 그래서 기본 타입(int/float/bool) 배열 + 좌(0)/우(1) 인덱스로 상태를 분리한다.
const uint8_t LEFT  = 0;   // 왼쪽  : 펄스 2  피드백 -> PWM 8
const uint8_t RIGHT = 1;   // 오른쪽 : 펄스 21 피드백 -> PWM 9

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
volatile long encCountL = 0;    // 왼쪽 펄스 (2번 핀)
volatile long encCountR = 0;    // 오른쪽 펄스 (21번 핀)
int target[2] = {0, 0};         // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)
int rawSpeed[2] = {0, 0};       // [0730-2] 필터 전 원시 펄스 (진단용)
int useSpeed[2] = {0, 0};       // [0730-2] PID가 실제로 본 값 = 텔레메트리로 보내는 값
unsigned long wheel_t = 0;

// ================= E-stop 상태 =================
bool estop_active = false;

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


// ================= ISR (2번=왼쪽 펄스, 21번=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


// ================= [0730-1] 쓰로틀 중앙값 필터 =================
// 읽으면서 바로 삽입정렬로 정렬해두고 가운데 값을 반환 (9개 기준 약 20회 비교, 부하 무시 가능).
// analogRead 9회 ≈ 0.94ms 소요. 제어주기(20ms) 안에서만 호출할 것.
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

// PID로 인계: pwm(pidLastPwm)은 램프가 올려놓은 값을 그대로 이어받고,
// 미분 킥이 생기지 않게 pidLastErr를 현재 오차로 맞춰둔다.
void exitLaunch(uint8_t idx, int speed) {
  launching[idx] = false;
  settleCnt[idx] = 0;
  noiseCnt[idx] = 0;
  pidI[idx] = 0;
  pidLastErr[idx] = target[idx] - speed;
}

// ================= [0730-2] 기동 블랭킹 중 개루프 출력 =================
// 피드백을 일절 보지 않고 FF(target)까지 PWM_SLEW_MAX로 램프만 한다.
// 허수 펄스가 얼마가 들어와도 램프가 멈추거나 되돌아가지 않는 것이 핵심.
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
// idx(LEFT/RIGHT)로 자기 상태/게인 배열만 참조해 완전히 독립적으로 동작.
int updatePid(uint8_t idx, int tgt, int speed) {
  int err = tgt - speed;
  int d = err - pidLastErr[idx];
  pidLastErr[idx] = err;

  float ff = interpFF((float)tgt);

  // 폭주 감지: 지속 과속이면 코스트 진입(PWM 0) — 순간 오버슈트는 CONFIRM 주기로 걸러냄
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

  // 코스트-캐치: 목표+마진까지 내려오면 PID 재개 (PWM은 FF값에서 시작)
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

  // 직접 PWM 모드는 무보호 경로: 받은 값을 그대로 출력 (피드백을 아예 쓰지 않음).
  // 펄스 계측/보고는 계속하되, 중앙값 버퍼도 갱신해 둔다 (펄스 모드 복귀 시 기준값용).
  if (driveMode[idx] == DRIVE_PWM) {
    if (raw <= PULSE_SANITY_MAX) pushSpeed(idx, raw);
    useSpeed[idx] = SPEED_MEDIAN_ON
                      ? median3(spdBuf[idx][0], spdBuf[idx][1], spdBuf[idx][2])
                      : raw;
    return pwmDirect[idx];
  }

  // ── 허수 판정 : 물리적으로 불가능한 값은 중앙값 버퍼에 넣지 않는다 ──
  //   버퍼가 갱신되지 않으므로 '직전 정상값이 자동 대입'되는 효과가 나고,
  //   그 덕에 노이즈 주기에도 PID/램프가 얼어붙지 않고 계속 진행된다.
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

    // (a) 램프 완료 && (b) 정상값 연속 → 노이즈 구간 종료로 보고 PID 인계.
    // 둘 중 하나라도 미충족이면 타임아웃까지 개루프 유지.
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


// ================= E-stop 안전 동작 (e-stop 상태에서 매 루프 호출) =================
void applyEstop() {
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // 양쪽 구동 상태 초기화 (직접 PWM 모드도 강제 해제, 해제 후 재개 시 잔재 방지)
  for (uint8_t s = 0; s < 2; s++) {
    target[s] = 0;
    driveMode[s] = DRIVE_PULSE;
    pwmDirect[s] = 0;
    pidI[s] = 0;
    pidLastErr[s] = 0;
    pidCoasting[s] = false;
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 재개 시 0부터 다시 램프업)
    runawayCnt[s] = 0;
    // [0730-2] 블랭킹도 즉시 해제 — e-stop 중에 개루프 램프가 살아 있으면 안 된다
    launching[s] = false;
    settleCnt[s] = 0;
    noiseCnt[s] = 0;
    // ※ 중앙값 버퍼(spdBuf)는 일부러 건드리지 않는다. e-stop 중에는 계측이 멈추므로
    //   0으로 채워버리면 '바퀴가 멈춤'으로 오인되어, 아직 관성으로 굴러가는 상태에서
    //   해제 직후 블랭킹(개루프 가속)에 진입할 수 있다. 직전 실측값을 남겨두는 편이
    //   보수적이다(굴러가는 중이면 블랭킹 없이 코스트-캐치로 넘어간다).
  }
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
  wheel_t = millis();
}


// ================= 입력 형식 검사 (부호 없는 정수만) =================
// 음수/소수/그 외 문자는 여기서 걸러짐
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), encISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), encISR_R, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  // E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  pinMode(ESTOP_PIN, INPUT_PULLUP);

  // [0730-1] 쓰로틀 페달 (analogRead는 pinMode 없어도 되지만 명시)
  pinMode(THROTTLE_PIN, INPUT);
  throttleRaw = readThrottleMedian();   // 첫 텔레메트리부터 실제값이 나가도록 프라이밍

  // [0730-2] 부팅 시엔 바퀴가 멈춰 있으므로 중앙값 버퍼를 0으로 채워둔다
  fillSpeed(LEFT, 0);
  fillSpeed(RIGHT, 0);

  unsigned long now = millis();
  wheel_t = tele_t = now;
}


// ================= 값 파싱 =================
// 숫자 토큰 -> 값. 숫자가 아니면 -1(줄 전체 무시용).
// 4자리 이상은 어차피 255 초과 = 정지이므로 atoi 오버플로 없이 256으로 통일.
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}


// ================= 펄스 모드 목표 적용 =================
// 직접 PWM 모드에서 복귀할 때는 PID 상태를 리셋하고 코스트로 진입
// (과속 상태면 무동력 감속, 이미 느리면 다음 주기에 바로 캐치되어 PID 재개)
// [0730-2] 멈춘 상태에서 처음 양수 목표를 받으면 기동 블랭킹으로 진입한다.
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

    // 직접 PWM 중 바퀴가 이미 멈춰 있었다면 기동 노이즈가 똑같이 발생하므로 블랭킹.
    // 돌고 있었으면 기존대로 코스트로 넘긴다.
    if (newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
      enterLaunch(idx, now);
    } else {
      launching[idx] = false;
      pidCoasting[idx] = true;
    }
    return;
  }

  // ★ [0730-2] 기동 블랭킹 진입 판정 : 멈춘 상태(0)에서 처음 양수 목표를 받을 때만 ★
  //   이미 굴러가는 중(코스트 등)에 목표가 바뀌는 경우는 노이즈가 없으므로 그대로 PID.
  //   판정 기준은 필터 후 speed(useSpeed) — 허수값에 속아 진입 판정이 뒤틀리지 않게.
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


// ================= 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =================
// 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지(펄스 0)
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


// ================= 입력 파서 =================
// "<값>" 단일 = 펄스 전용(0~15, 좌우 공통), "<좌값>,<우값>" = 좌/우 독립(펄스/PWM/정지).
// 형식이 안 맞으면 무시.
void handleLine(char* line, unsigned long now) {
  if (strchr(line, ' ')) return;   // 공백 포함 줄은 형식 오류 (기존과 동일하게 무시)

  char* comma = strchr(line, ',');
  if (comma) {
    *comma = '\0';
    char* tokR = comma + 1;
    if (strchr(tokR, ',')) return;   // 콤마 2개 이상 → 무시

    long vL = parseValue(line);
    long vR = parseValue(tokR);
    if (vL < 0 || vR < 0) return;

    // e-stop 중에는 구동 명령 미적용
    if (estop_active) return;
    applySide(LEFT, vL, now);
    applySide(RIGHT, vR, now);
  } else {
    // 단일 값은 무조건 펄스 모드: 0~15만 받고 그 외는 무시
    // (직접 PWM은 콤마 2값 형식으로만 가능 — 일반 주행 경로에서의 오발동 방지)
    long v = parseValue(line);
    if (v < 0 || v > TARGET_MAX) return;

    if (estop_active) return;
    setPulseTarget(LEFT, (int)v, now);
    setPulseTarget(RIGHT, (int)v, now);
  }
}

void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen > 0) handleLine(rxBuf, now);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
    }
  }
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

  // [0730-1] 쓰로틀 페달 계측 (제어에는 쓰지 않고 텔레메트리로만 보고).
  // 텔레메트리(50ms)가 이 값을 재사용하므로 sendOutput에서 ADC를 또 돌지 않는다.
  throttleRaw = readThrottleMedian();

  // ※ 좌우 차동 계산은 이 보드에서 제거됨 (ROS2 kasa_ws가 담당).
  //   kasa_ws가 B보드 조향각을 보고 이미 좌/우로 나눈 값을 "<좌>,<우>"로 내려주므로,
  //   여기서는 target[LEFT]/target[RIGHT]를 그대로 추종하기만 하면 된다.

  // [0730-2] 모드 분기(펄스 PID / 직접 PWM)와 허수 필터·기동 블랭킹은
  // updateWheelSide가 전담한다. rawSpeed/useSpeed도 그 안에서 갱신된다.
  int pwmL = updateWheelSide(LEFT,  (int)cL, now);   // 왼쪽 : 펄스 2 -> PWM 8
  int pwmR = updateWheelSide(RIGHT, (int)cR, now);   // 오른쪽 : 펄스 3 -> PWM 9

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= 출력 =================
// [0730-1] 3필드로 확장: "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>"
//   입력 "<왼쪽>,<오른쪽>"과 앞 두 필드 순서가 동일하다 (왼쪽이 앞, 오른쪽이 뒤).
//   쓰로틀raw는 0~1023 (A0 가변저항 중앙값, 이 보드는 제어에 쓰지 않음).
//   직접 PWM 모드에서도 펄스 계측/출력은 계속됨 (PWM-펄스 특성 계측용)
// [0730-2] 펄스 필드는 '필터 후 값'(useSpeed = PID가 실제로 본 값)을 보낸다.
//   필드 구성은 그대로이므로 kasa_ws 수정은 필요 없다. 기동 구간의 허수 펄스
//   (수백~1500)를 ROS2 화면에 그대로 띄우지 않기 위한 선택.
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("S,");
  Serial.print(useSpeed[LEFT]);
  Serial.print(',');
  Serial.print(useSpeed[RIGHT]);
  Serial.print(',');
  Serial.println(throttleRaw);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial(now);

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
    applyEstop();
  } else {
    updateWheel(now);
  }

  sendOutput(now);
}
