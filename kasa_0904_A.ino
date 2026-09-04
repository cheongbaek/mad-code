// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID + 쓰로틀 페달 계측 (Arduino Mega 2560) - 0904 버전
//  kasa_0804_A.ino 기반. 제어 로직(PID·FF·블랭킹·중앙값)은 한 줄도 바꾸지 않았다.
//  바뀐 것은 셋이다 : ①E-stop 확인시간 500ms→100ms, ②이 보드의 E-stop 비활성화,
//                     ③연결확인 핸드셰이크('-' → "YES") 추가.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0904-1] E-stop 확인시간 500ms → ★100ms★ (발동·해제 대칭은 그대로) ★★
// ════════════════════════════════════════════════════════════════════════════
//      발동 : 100ms 연속 개방(HIGH) / 해제 : 100ms 연속 단락(LOW).
//      B보드(kasa_0904_B.ino)도 같은 값으로 함께 바뀌었다 — 두 보드의 규약은 늘 같다.
//
//      [왜 줄이는가] 0804 의 500ms 는 '주행 중 오발동이 더 위험하다'는 판단으로 둔
//        값이었다. 그 대가가 ★비상정지가 최대 0.5초 늦는 것★ 인데, 실제로 밟히기까지
//        차가 가는 거리가 저속에서도 1m 를 넘었다. 접점 채터링은 100ms 로도 충분히
//        걸러지므로(채터링은 수 ms 단위다), 확인 시간을 노이즈를 막는 최소치까지
//        낮추고 정지 지연을 그만큼 되돌린다.
//
//      ★ 늦는 거리 (100ms 기준) ★
//            2.65 m/s ( 9.5km/h, 3펄스) → 약 0.27 m
//            4.42 m/s (15.9km/h, 5펄스) → 약 0.44 m
//           13.26 m/s (47.7km/h,15펄스) → 약 1.33 m
//        ※ 물리적 최후 수단은 여전히 별개로 있어야 한다 — 이 핀은 '소프트웨어 비상정지'다.
//
//      [구현] 판정은 loop 의 digitalRead 폴링(updateEstop)만으로 한다. 0731 에 있던
//        핀체인지 인터럽트(PCINT)는 0804 에서 제거했고 되살리지 않았다 — 확인 시간을
//        두는 이상 '개방을 본 즉시 끊는' ISR 은 요구와 모순되고, 남겨두면 확인시간
//        미만의 노이즈에서 인휠 PWM 만 순간 끊겨(램프업 리셋) 주행이 거칠어진다.
//        loop 는 수 kHz 로 돌아 100ms 판정에도 해상도가 충분히 남는다.
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0904-2] 이 보드의 E-stop 은 ★꺼져 있다★ (ESTOP_ENABLED = false) ★★
// ════════════════════════════════════════════════════════════════════════════
//      ★차량의 E-stop 이 없어진 것이 아니다★ D12 NC 라인은 A·B 두 보드에 병렬로
//      물려 있고, ★B보드(kasa_0904_B.ino)가 ESTOP_ENABLED = true 로 그대로 감지한다★.
//      B보드가 발동하면 조향 힘빼기 + 리니어 2단(풀브레이킹)이 걸리고, 상위
//      (nxde/arduino.py update_estop)가 B보드의 "STOP" 한 줄을 보고 A보드로 정지값을
//      보낸다. 즉 제동과 구동차단은 그대로 살아 있다.
//
//      [왜 A보드만 끄는가]
//        ① 같은 한 라인을 두 보드가 각자 판정하면 ★같은 사건에 두 개의 판정 시각★ 이
//           생긴다. 한쪽만 발동한 동안 텔레메트리가 갈려(S, 는 나오는데 STOP 도 나온다)
//           상위 로그에서 원인을 읽기 어려웠다.
//        ② A보드가 STOP 만 내보내는 동안에는 ★쓰로틀 raw(A0)도 함께 끊긴다★.
//           그 값은 수동조종의 유일한 구동 입력이라, E-stop 을 푼 직후에도 페달이
//           죽은 것처럼 보이는 구간이 생겼다.
//        ③ 보드 식별에도 걸린다 — E-stop 중에는 'S,' 가 안 나와 arduino.py 의
//           identify_port 가 A보드를 A 로 알아보지 못했다(DETECT_ESTOP_HOLD_S 참고).
//           이 보드가 E-stop 을 보지 않으면 그 동안에도 계속 'S,' 를 내보내므로
//           ★E-stop 이 걸린 채로도 A보드는 정상 식별된다★.
//      ★되돌리려면 아래 ESTOP_ENABLED 를 true 로만 바꾸면 된다★ (판정 코드는 그대로다).
//
// ════════════════════════════════════════════════════════════════════════════
//  ★★ [0904-3] 연결확인 핸드셰이크 : '-' 한 줄을 받으면 "YES" 한 줄 ★★
// ════════════════════════════════════════════════════════════════════════════
//      ▶ 왜 : 텔레메트리('S,...')가 나온다는 것은 ★보드→PC 한 방향★ 만 증명한다.
//        PC→보드 방향은 아무것도 증명하지 못한다(TX 선이 빠져도, 포트를 다른
//        프로세스가 물고 있어 write 가 먹히지 않아도 출력은 멀쩡히 보인다).
//        그래서 상위(nxde/arduino.py identify_port, nxde/check.py)가 '-' 를 보내고
//        "YES" 를 받아야 비로소 ★양방향★ 연결로 인정한다. 못 받으면 포트를 닫고
//        다시 열어 처음부터 다시 확인한다.
//      ▶ 동작 : '-' 한 줄(공백·다른 문자 없이)을 받으면 즉시 "YES" 를 한 번 찍고,
//        그 뒤 HANDSHAKE_HOLD_MS(2000ms) 동안 ★평상시 텔레메트리를 내보내지 않는다★.
//        상위가 "YES" 를 텔레메트리에 섞이지 않은 채로 확인할 수 있게 하기 위함이다.
//        유예가 끝나면 아무 상태도 남기지 않고 원래대로 돌아간다.
//      ▶ ★millis() 로만 센다 — delay() 를 쓰지 않는다★ 유예 중에도 제어주기(20ms)와
//        E-stop 판정은 평소대로 돈다. 멈추는 것은 출력 한 줄뿐이다.
//      ▶ ★E-stop 의 "STOP" 은 유예 중에도 나간다★ 상위가 그 문자열로 비상정지를
//        판정하므로, 여기서 가리면 2초 동안 비상정지가 보이지 않는다.
//        (이 보드는 [0904-2] 로 E-stop 을 끄고 있어 실제로는 나오지 않지만,
//         ESTOP_ENABLED 를 true 로 되돌렸을 때를 위해 순서를 그렇게 두었다.)
//      ▶ 구동에는 아무 영향이 없다 — 목표펄스도, 직접 PWM 도 건드리지 않는다.
//
//  --- 이하 구조는 0804(=0730) 과 동일 ---
//  [0730-1] 쓰로틀 페달 입력(A0) + 텔레메트리 4필드 "S,<좌펄스>,<우펄스>,<쓰로틀raw>"
//      가속페달 가변저항을 A0으로 읽어 0~1023 raw로 보고만 한다(이 보드는 제어에 쓰지 않음).
//      9샘플 중앙값만 적용(지연 ~1ms). 지수평활은 넣지 않았다.
//
//  [0730-2] ★★ 기동 노이즈 대책 : 기동 블랭킹(LAUNCH) + 3점 중앙값 필터 ★★
//      (iw_0730_pid.ino에서 검증한 로직을 그대로 이식. 상세 배경은 그 파일 헤더 참고)
//
//      [문제] "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간에서 홀신호에 수백~1500
//      단위 노이즈가 쏟아진다. 이 모터는 무부하 최대 1320RPM x 64펄스/회전(직결구동)이라
//      20ms 창당 최대 ≈28.2펄스가 물리적 한계이므로 그 이상은 전부 허수다.
//      기존 PID는 이 허수를 오차로 먹어 pwm을 0으로 클램프했고, 램프업이 통째로 리셋 →
//      재램프 → 또 노이즈의 무한루프에 빠졌다.
//
//      [대책 1 : 기동 블랭킹] 정지 상태에서 목표가 0 → 양수로 바뀌면 LAUNCH 상태로
//      들어가 그 구간 동안 피드백을 아예 보지 않는다(개루프).
//        - pwm = FF(target)까지 PWM_SLEW_MAX(+4/cycle)로 램프. PID/코스트/폭주감지 정지
//        - 종료 조건 : (a) 램프 완료 && (b) 정상 펄스 LAUNCH_SETTLE_CYCLES 연속,
//          또는 LAUNCH_MAX_MS 타임아웃.  ★(a)가 없으면 노이즈가 오기도 전에 조기 종료★
//      [대책 2 : 3점 중앙값] 주행 중 남는 단발성 스파이크 제거. 허수 판정된 값은 버퍼에
//      아예 넣지 않아, 노이즈 주기에 '직전 정상값이 자동 대입'되는 효과가 난다.
//
//      ※ 텔레메트리 펄스 필드는 '필터 후 값'(PID가 실제로 본 값)을 보낸다.
//
//  --- 이하 구조는 0727 nodiff와 동일 ---
//  좌우 차동은 이 보드에 없다. ROS2(gold_ws/src/nxde)가 담당한다:
//     B보드 텔레메트리 "P,<조향각>,<A5원본>,<모드>"  ->  arduino.py 가 좌/우 펄스 계산
//     ->  A보드로 "<좌펄스>,<우펄스>" 콤마 2값 전송  ->  A보드는 그대로 추종만
//
//  ★ 배선 (2026-08-13 실차 재확인 — 0730 이후 세 번 바뀌었다) ★
//     왼쪽  모터 : 펄스 21 -> PID -> PWM 8
//     오른쪽 모터 : 펄스 20 -> PID -> PWM 10
//   좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//     ※ 0730 헤더에는 "왼쪽 펄스 2 / 오른쪽 펄스 21, PWM 8/9" 로 적혀 있었는데, 그 뒤
//       ①왼쪽 홀센서 2→20, ②우측 PWM 9→10, ③홀센서 L/R 교체(20↔21) 세 번의 배선
//       변경이 코드에만 반영되고 주석에는 남지 않았다. 여기서 실제 핀에 맞춘다.
//
//  입력 : "<값>" 또는 "<왼쪽값>,<오른쪽값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값: 펄스 전용(0~15), 범위 밖/숫자 아님은 무시 (좌우 동일값)
//         - 콤마 2값: 좌/우 독립 — 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지
//         - ★"-" 한 줄 : 연결확인 핸드셰이크 ([0904-3]) — "YES" 를 한 번 돌려준다★
//  출력 : "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" (평상시)
//         "STOP" (e-stop 중 — 이 보드는 [0904-2] 로 꺼 두었으므로 실제로는 안 나온다)
//         "YES"  (핸드셰이크 응답, [0904-3])
//  제어주기 : 20ms, 출력주기 : 50ms
//  E-stop 스위치: ★12번 핀★, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) -> e-stop
//    - ★[0904-1] 발동: 100ms 연속 개방(HIGH) / 해제: 100ms 연속 단락(LOW) — 대칭★
//    - ★[0904-2] 단, 이 보드는 ESTOP_ENABLED = false 라 판정 자체를 하지 않는다★
//    - e-stop 발동 시 직접 PWM 모드도 즉시 해제(펄스 0으로 복귀)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 21;   // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 20;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (펄스 21과 같은 컨트롤러)
const uint8_t PWM_PIN_R = 10;    // 오른쪽 모터 PWM (펄스 20과 같은 컨트롤러)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
// 폴링으로만 판정한다 (0804에서 PCINT 제거 — 확인 시간을 두는 이상 즉시 감지가
//   요구와 모순된다). D12 는 PCINT 를 걸 수 없는 핀이기도 하다.
const uint8_t ESTOP_PIN = 12;
// ★[0904-2] 이 보드는 E-stop 을 보지 않는다★ 차량 E-stop 은 B보드(kasa_0904_B.ino,
//   ESTOP_ENABLED = true)가 같은 D12 라인에서 판정한다. 이유는 헤더 [0904-2] 참고.
//   되살리려면 이 한 줄을 true 로 바꾸면 된다 — 판정 코드는 그대로 남겨 두었다.
const bool ESTOP_ENABLED = false;

// --- [0730-1] 쓰로틀(가속) 페달 가변저항 ---
// 이 보드는 이 값으로 제어하지 않는다. 0~1023 raw를 텔레메트리로 보고만 한다.
const uint8_t THROTTLE_PIN = A0;


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


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


// ================= [0904-1] E-stop 상태 =================
bool estop_active = false;

// ★ 발동·해제 모두 이 시간 동안 레벨이 '연속으로' 유지되어야 인정한다 (대칭) ★
//   발동 : ESTOP_TRIGGER_CONFIRM_MS 동안 계속 개방(HIGH)
//   해제 : ESTOP_RELEASE_CONFIRM_MS 동안 계속 단락(LOW)
//   중간에 반대 레벨이 한 번이라도 관측되면 해당 타이머는 0으로 리셋된다.
//
// ⚠️ 발동을 늦추면 그만큼 차가 더 간다(헤더 [0904-1] 거리 표 참고).
//   100ms 는 '접점 채터링(수 ms)은 거르고 정지 지연은 최소로' 를 노린 값이다.
//   0 으로 두면 첫 관측에서 바로 발동한다(0731의 폴링 즉시발동과 동일. 단 ISR이 없으므로
//   차단 시점은 loop 한 바퀴 뒤다 — 수십 us 수준).
//   ※ 이 보드는 ESTOP_ENABLED = false 라 아래 두 값이 실제로 쓰이지 않는다([0904-2]).
//     B보드(kasa_0904_B.ino)의 같은 상수와 ★항상 같은 값으로 유지할 것★.
const unsigned long ESTOP_TRIGGER_CONFIRM_MS = 100;
const unsigned long ESTOP_RELEASE_CONFIRM_MS = 100;

// 각 레벨이 '처음' 관측된 시각. 0 = 아직 그 레벨을 관측하지 않음(또는 리셋됨).
//   ※ millis()가 0을 돌려주는 부팅 첫 1ms 는 이 규약에서 '미관측'으로 취급되지만,
//     다음 루프(수 us 뒤)에 다시 세워지므로 실질 영향이 없다.
unsigned long estop_high_t = 0;   // HIGH(개방)가 처음 관측된 시각 -> 발동 타이머
unsigned long estop_low_t  = 0;   // LOW(단락)가 처음 관측된 시각 -> 해제 타이머


// ================= 출력용 =================
unsigned long tele_t = 0;
const unsigned long TELE_MS = 50;


// ================= ★[0904-3] 연결확인 핸드셰이크 ('-' → "YES")★ =================
// 상위(nxde/arduino.py identify_port, nxde/check.py)가 ★PC→보드 방향이 실제로
// 살아 있는지★ 확인하는 유일한 수단이다. 텔레메트리는 보드→PC 한 방향만 증명한다.
//   '-' 한 줄  → "YES" 한 줄을 즉시 찍고, 이 시간 동안 평상시 텔레메트리를 멈춘다.
//   멈추는 이유는 상위가 "YES" 를 'S,...' 에 섞이지 않은 채로 읽게 하기 위함이다.
//   ★millis() 로만 센다 — delay() 를 쓰지 않는다★ 유예 중에도 제어주기와 E-stop
//   판정은 평소대로 돈다. 멈추는 것은 출력 한 줄뿐이고, 끝나면 아무 상태도 남지 않는다.
const unsigned long HANDSHAKE_HOLD_MS = 2000;
bool          handshake_hold = false;   // 지금 유예 중인가
unsigned long handshake_t    = 0;       // "YES" 를 찍은 시각


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= ISR (21번=왼쪽 펄스, 20번=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


// ================= E-stop 핀체인지 인터럽트는 없다 (0804에서 제거) =================
// 0731에는 ISR(PCINT0_vect) + setupEstopPcint()가 있었고, 개방(HIGH)을 본 즉시 좌우 인휠
// PWM을 0으로 떨어뜨렸다. 발동에 확인 시간을 요구하는 순간 그 즉시성은 요구와 모순되고,
// 남겨두면 확인시간 미만의 노이즈에서 PWM만 순간 끊겨(램프업 리셋) 주행이 오히려 거칠어진다.
// 판정은 loop의 digitalRead 폴링(updateEstop)만으로 한다.
//   ※ 되살리려면 kasa_0731_A.ino 의 ISR(PCINT0_vect) / setupEstopPcint() /
//     estop_pin_hit / estop_edge_seen 4개를 그대로 가져오면 된다. 단 그 시절 핀은
//     D13(PB7/PCINT7)이었고 지금은 D12 라, PCINT 그룹부터 다시 잡아야 한다.


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


// ================= E-stop 안전 동작 (e-stop 상태에서 매 루프 호출) =================
// ★ 출력 차단은 여기가 유일한 경로다 ★ 0731에는 ISR이 먼저 PWM을 끊어 주었지만
//   PCINT를 제거했다(헤더 참고). 발동이 확정되는 루프에서 곧바로 이 함수가
//   불리므로(loop 참고) 실질 지연은 루프 한 바퀴(수십 us)다.
// ※ 이 보드는 [0904-2] 로 E-stop 을 끄고 있어, 지금 구성에서는 호출되지 않는다.
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
    pidLastPwm[s] = 0;   // 슬루레이트 제한 기준점도 리셋 (해제 후 0부터 다시 램프업)
    runawayCnt[s] = 0;
    launching[s] = false;
    settleCnt[s] = 0;
    noiseCnt[s] = 0;
    // ※ 중앙값 버퍼(spdBuf)는 일부러 건드리지 않는다. e-stop 중에는 계측이 멈추므로
    //   0으로 채워버리면 '바퀴가 멈춤'으로 오인되어, 아직 관성으로 굴러가는 상태에서
    //   해제 직후 블랭킹(개루프 가속)에 진입할 수 있다.
  }
  noInterrupts();
  encCountL = 0;
  encCountR = 0;
  interrupts();
  wheel_t = millis();
}


// ================= ★[0904-3] 핸드셰이크 토큰★ ('-' 한 글자로만 이루어진 줄) =================
// 구동 명령과 겹칠 여지가 없다 — 이 보드의 입력은 전부 ★부호 없는★ 정수라서
// '-' 로 시작하는 줄은 어차피 isValidNumber 에서 걸러진다.
bool isHandshakeToken(const char* s) {
  return s && s[0] == '-' && s[1] == '\0';
}


// ================= 입력 형식 검사 (부호 없는 정수만) =================
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

  // [0904-1] E-stop (NC: INPUT_PULLUP, 평상시 스위치가 GND로 눌러 LOW)
  //   폴링만 쓴다(PCINT 없음). 타이머는 0에서 시작하므로, 이미 개방된
  //   채로 켜져도 첫 루프부터 발동 타이머가 돌아 100ms 뒤에 발동한다. 부팅 직후에는
  //   모터가 정지 상태이고 시리얼 명령도 아직 없어 그 100ms 는 위험하지 않다.
  //   ★[0904-2] 단, 이 보드는 ESTOP_ENABLED = false 라 판정 자체가 돌지 않는다★
  //     핀 설정은 그대로 둔다 — true 로 되돌리는 순간 바로 동작해야 하기 때문이다.
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_active = false;
  estop_high_t = 0;
  estop_low_t  = 0;

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
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
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


// ================= 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =================
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
void handleLine(char* line, unsigned long now) {
  // ★[0904-3] "-" 한 줄 = 연결확인 핸드셰이크★ 구동에는 아무 영향이 없다.
  //   ★e-stop 중에도 받는다★ 이것은 구동 명령이 아니라 '통신이 살아 있는가' 의 답이고,
  //   E-stop 이 걸린 채로 연결을 확인해야 하는 상황이 실제로 있다(런치 전 점검).
  if (isHandshakeToken(line)) {
    Serial.println("YES");
    handshake_hold = true;
    handshake_t = now;
    return;
  }

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

  // [0730-1] 쓰로틀 페달 계측 (제어에는 쓰지 않고 텔레메트리로만 보고)
  throttleRaw = readThrottleMedian();

  int pwmL = updateWheelSide(LEFT,  (int)cL, now);   // 왼쪽 : 펄스 21 -> PWM 8
  int pwmR = updateWheelSide(RIGHT, (int)cR, now);   // 오른쪽 : 펄스 20 -> PWM 10

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= [0904-1] E-stop 판정 (발동·해제 모두 100ms 연속 유지) =================
// 현재 핀 레벨만 본다(ISR 없음). 매 루프 호출되므로 폴링 해상도는 100ms 판정에 충분하다.
// ★[0904-2] ESTOP_ENABLED = false 인 지금은 첫 분기에서 곧바로 빠져나온다★
//   - 발동 : ESTOP_TRIGGER_CONFIRM_MS 동안 계속 개방(HIGH)
//   - 해제 : ESTOP_RELEASE_CONFIRM_MS 동안 계속 단락(LOW)
//   - 반대 레벨이 한 번이라도 보이면 그쪽 타이머가 0으로 리셋된다 → '연속' 유지가 조건
// ★ 지금 상태에 필요한 타이머만 돌린다 ★ 발동 중에는 해제 타이머만, 정상 중에는 발동
//   타이머만 의미가 있다. 그래서 각 분기에서 반대쪽 타이머를 0으로 눕혀 둔다 —
//   그래야 상태가 바뀐 직후에 낡은 타이머 값이 남아 즉시 재전환되는 일이 없다.
void updateEstop(unsigned long now) {
  if (!ESTOP_ENABLED) {
    estop_active = false;
    estop_high_t = 0;
    estop_low_t  = 0;
    return;
  }

  bool open_now = (digitalRead(ESTOP_PIN) == HIGH);   // NC 개방 = e-stop 요청

  if (!estop_active) {
    // ── 정상 상태 : 개방이 연속 유지되면 발동 ──
    estop_low_t = 0;
    if (!open_now) {
      estop_high_t = 0;                 // 단락을 봤다 → 발동 타이머 리셋
    } else if (estop_high_t == 0) {
      estop_high_t = now;               // 개방 관측 시작
    } else if (now - estop_high_t >= ESTOP_TRIGGER_CONFIRM_MS) {
      estop_active = true;              // ★ 100ms 연속 개방 확인 → 발동 ★
      estop_high_t = 0;
    }
  } else {
    // ── 발동 상태 : 단락이 연속 유지되면 해제 ──
    estop_high_t = 0;
    if (open_now) {
      estop_low_t = 0;                  // 개방을 다시 봤다 → 해제 타이머 리셋
    } else if (estop_low_t == 0) {
      estop_low_t = now;                // 단락 관측 시작
    } else if (now - estop_low_t >= ESTOP_RELEASE_CONFIRM_MS) {
      estop_active = false;             // 100ms 연속 단락 확인 → 해제
      estop_low_t = 0;
    }
  }
}


// ================= 출력 =================
// "S,<왼쪽펄스>,<오른쪽펄스>,<쓰로틀raw>" / e-stop 중에는 "STOP"만
// ★[0904-3] 핸드셰이크 유예 중에는 평상시 줄을 내보내지 않는다★
//   ★단 "STOP" 은 가리지 않는다★ 상위가 그 문자열로 비상정지를 판정하므로, 여기서
//   막으면 2초 동안 비상정지가 상위에 보이지 않는다. 그래서 순서가 estop → 유예다.
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  if (handshake_hold) {
    if (now - handshake_t < HANDSHAKE_HOLD_MS) return;
    handshake_hold = false;      // 유예 종료 — 다음 줄부터 원래대로
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

  // ★ [0904-1] 최우선 : E-stop 판정 (발동·해제 모두 100ms 연속 유지 확인) ★
  //   PCINT가 없으므로 출력 차단도 아래 applyEstop()이 한다. pollSerial 보다 앞에
  //   두어, 발동이 확정된 루프에서 들어온 명령이 한 번 적용되는 틈을 없앤다.
  //   ★[0904-2] ESTOP_ENABLED = false 인 지금은 estop_active 가 항상 false 다★
  updateEstop(now);

  pollSerial(now);

  if (estop_active) {
    applyEstop();
  } else {
    updateWheel(now);
  }

  sendOutput(now);
}
