// ============================================================
//  linear_0804.ino : 리니어(브레이크) 3단계 위치 이동 테스트 (Arduino Mega 2560)
//
//  ★ 목적 ★
//    linear_0803_pot.ino로 '행정 크기'를 재고 나서, 그 다음 단계인 **위치 명령 구동**을
//    처음 붙여보는 테스트 코드다. 시리얼로 0/1/2 한 글자를 받아 정해진 위치까지 리니어를
//    움직이고, 도달하면 그 자리를 새 영점으로 잡는다.
//
//  ★ 입력 (시리얼 모니터, 115200, ★개행 전송★) — 정확히 "0"/"1"/"2" 한 줄만 인정 ★
//      0 = 기본위치  : ★엔코더를 보지 않고★ 들어가는 방향으로 HOME_DRIVE_MS(0.5초) 구동
//                      -> 끝(하드스톱)까지 밀어 기준을 회복하는 '재영점' 동작
//      1 = 약한 위치 : 기본위치 대비 변화량 POS_FULL의 ★1/3★ (기본값 83)
//      2 = 최대위치  : 기본위치 대비 변화량 POS_FULL       (기본값 250)
//    그 외 입력(빈 줄, "10", "0 " 등)은 전부 무시한다.
//    ※ 위치값은 POS_FULL 하나만 고치면 중간위치가 따라온다(아래 '단계별 위치' 절 참고).
//
//  ★ 출력 : REPORT_MS(5ms)마다 '지금 영점 대비 엔코더 변화량' 정수 하나 ★
//    값이 변하지 않아도 매 주기 찍는다(등간격 표본). 숫자 하나뿐이라 시리얼 플로터로도
//    볼 수 있고, 상태 변화·경고만 '#'로 시작하는 줄로 따로 나간다(플로터는 무시).
//
//  ★★ 위치 모델 : '영점은 항상 현재 단계의 기준점' ★★
//    증분 엔코더는 절대위치를 모르므로(linear_0803_pot.ino 헤더 참고), 이 코드는
//    "지금 몇 단계에 있는지(cur_level)"를 따로 기억하고, 엔코더는 **그 단계의 기준점
//    대비 변화량**만 센다. 구동이 끝나면 그 자리를 다시 0으로 잡는다.
//      목표 상대변화량 = LEVEL_POS[목표단계] - LEVEL_POS[현재단계]
//      남은 오차       = 목표 상대변화량 - 현재 엔코더값   -> ★이 오차를 시간으로 환산한다★
//    이 두 줄이 요구사항을 전부 만족한다 (아래 ms는 실측표 보간 결과):
//      - 기본위치(0)에서 2 입력          -> 목표 250, enc 0   -> 250만큼(198ms) 밟는다
//      - 기본위치(0)에서 1 입력          -> 목표  83, enc 0   -> 83만큼(57ms) 밟는다
//      - 최대(2)에서 1 입력              -> 목표 -167, enc 0  -> 167만큼(147ms) 들어간다
//      - 기본위치에서 발로 150 밟은 뒤 2 -> 목표 250, enc 150 -> ★100만★(65ms) 밟는다
//      - 기본위치에서 발로 200 밟은 뒤 1 -> 목표  83, enc 200 -> ★117 들어간다★(96ms)
//    ※ 대기(ST_IDLE) 중에 사람이 페달을 밟아 엔코더가 변해도 아무 보정도 하지 않는다.
//      그 값이 곧 '지금 얼마나 밟혀 있는가'이고, 다음 명령이 그것을 그대로 반영한다.
//
//  ★★★ [0804-2] 제어 방식 : 시간이 주(主), 엔코더는 보험 ★★★
//    [폐루프였던 이전 판이 실패한 기록] 처음에는 엔코더를 매 루프 보며 '목표에 닿는 순간'
//      PWM을 끊었다. 그런데 실측이 이렇게 나왔다:
//        # MOVE L1 rel=150 cur=0 err=150   ->   # DONE L1 OVER 73 -> ZERO
//      목표 150인데 217에서 멈췄다(오버슛 73 = 49%). 원인은 판정 주기가 아니다 —
//      PWM 255에서 약 1카운트/ms로 움직이고, 끊은 뒤에도 관성으로 26~75카운트 더 간다.
//      즉 ★'닿는 순간 끊기'로는 원리적으로 못 맞춘다★ (POS_TOLERANCE를 0으로 해도 남는다).
//      감속하려면 PD가 필요한데, 브레이크는 반응성이 최우선이고 러프해도 되는 대상이다.
//
//    [지금 방식] linear_0803_speed.ino로 "몇 ms 밀면 몇 카운트 가는가"를 실측해 표로 넣고,
//      명령이 오면 ★필요한 카운트를 그 표로 시간으로 환산해 그 시간만큼만 민다★.
//      엔코더는 제어에서 빠지고 ★보험★으로만 남는다:
//        - 목표 카운트에 먼저 닿으면 시간이 남았어도 끊는다("by=ENC")
//        - ENC_HARD_MAX를 넘으면 무조건 끊는다("by=HARDMAX")
//      정상 동작에서는 보험이 걸리지 않는다 — 표의 카운트는 '정착 후' 값이라, 예를 들어
//      250카운트용 198ms를 주면 끊는 순간의 값은 214쯤이고 250은 관성이 채운다.
//      보험이 걸렸다면 그것은 '표가 실제보다 느리게 잡혀 있다'는 신호이므로 로그를 볼 것.
//
//    ※ 부족(undershoot)은 보정하지 않는다. 시간이 끝나면 그대로 그 단계로 확정한다
//      (kasa_0731_B.ino의 열린루프 전이와 같은 태도). 얼마나 어긋났는지는 "# DONE"의
//      want/got으로 매번 나오므로, 계통적으로 치우치면 표나 TIME_PCT를 고치면 된다.
//
//    ※ PWM은 항상 DRIVE_PWM(255)이므로 제어주기(20ms 창)를 두지 않는다. 상수를 주기마다
//      다시 쓰는 것은 아무 일도 하지 않으면서 정지 판정만 늦춘다. 구동 개시/정지는 사건이
//      생긴 순간에 한 번씩 처리하고, 종료 판정(시간·보험)은 매 루프 확인한다.
//
//  ★★ 되돌아오지 않는다 (채터링 방지) ★★
//    방향은 구동을 시작할 때 한 번만 정한다. 시간이 끝났거나 보험이 걸려 멈춘 뒤에는,
//    목표를 지나쳤더라도 되돌아오지 않는다. 브레이크에서 왕복은 승차감·기구 모두에
//    해로우므로 의도한 설계다.
//
//  ★ 영점을 잡는 시점은 두 군데다 ★
//    (1) 구동 직후 : PWM을 끊고 SETTLE_MS(100ms) 관성 정착을 기다린 뒤 그 자리를 0으로.
//        '끊은 직후'가 아니라 '실제로 멈춘 뒤'여야 그 자리가 진짜 기준점이 된다. 이 대기
//        중에 더 밀려간 양은 "coast="로 보고한다(실측 26~75카운트. 표가 이미 이 관성을
//        포함한 '정착 후' 값이라, 여기서 기다리지 않으면 기준점이 매번 앞당겨진다).
//    (2) ★정지 3초 : 대기(ST_IDLE) 중 엔코더가 IDLE_ZERO_MS(3초) 동안 움직이지 않으면
//        그 지점을 지금 단계의 새 기준점으로 삼는다("# REZERO n")★
//        - ★단계(cur_level)는 바꾸지 않는다★ 마지막으로 1단계에 뒀다면 그 자리가 1단계의
//          기준점이 되고, 이후 2를 넣으면 거기서 +(POS_LEVEL2 - POS_LEVEL1) = +167을 간다.
//        - 목적은 증분 엔코더의 누적 드리프트 흡수다(사이클당 -55카운트 실측, 0803_pot 헤더).
//          사람이 발로 밟았다 뗀 뒤 남는 기준점 이동도 여기서 함께 흡수된다.
//        - ★주의★ 발을 페달에 올린 채 3초를 넘기면 '밟힌 그 위치'가 기준점이 된다. 발을
//          뗀 뒤 다시 3초가 조용하면 그때 또 보정되므로 결국 제자리로 수렴하지만, 명령을
//          넣는 순간의 기준점이 무엇인지는 "# REZERO" 로그로 확인하는 습관을 들일 것.
//        - '움직이지 않았다'의 판정은 IDLE_QUIET_BAND(±2카운트) 이내다(진동·양자화 여유).
//
//  ★ 배선 ★
//    - 리니어(브레이크) 모터드라이버 MD20A : DIR = D8, PWM = D9   (kasa_0731_B.ino와 동일)
//        ★ DIR = LOW(LINEAR_FWD)  = 로드가 나오는 방향 = 브레이크를 밟는 방향
//        ★ DIR = HIGH(LINEAR_REV) = 로드가 들어가는 방향 = 브레이크를 놓는 방향
//        리니어 MB - 빨간색, MA - 검은색
//    - 로터리 엔코더 E40S8-2048-3-T-5 : OUT A -> D2(INT4), OUT B -> D3(INT5), C(Z) 미사용, 5V/GND
//        ※ kasa_0731_B.ino가 D2에 쓰던 리미트 스위치는 이 코드에 없다. 그 자리를 엔코더가 쓴다.
//        x4 디코딩 -> 1회전 8192카운트, 1카운트 = 0.044도
//    ※ 밟는 방향으로 움직였을 때 엔코더 값이 '음수'로 가면 INVERT_DIR을 true로 바꾼다.
//      (이 코드는 "밟는 방향 = 카운트 증가"를 전제로 방향을 고른다)
//
//  ★★ 이 파일에는 E-stop도 리미트 스위치도 없다 ★★
//    단독 테스트 코드다(linear_0803 계보). 대신 아래 안전장치만 둔다:
//      - 구동 시간의 상한 MAX_DRIVE_MS(300ms). 표 계산이 어떻게 나오든 이보다 길게 밀지 않는다
//        (그래서 별도 타임아웃이 필요 없다 — 모든 구동은 정해진 시간에 반드시 끝난다)
//      - 엔코더 보험 2단 : 목표 카운트 도달(by=ENC) / 절대 상한 ENC_HARD_MAX(by=HARDMAX)
//      - MIN_MOVE_COUNTS 미만의 소량 이동은 아예 하지 않는다(시간으로 제어 불가 구간)
//      - 구동 중 새 명령이 오면 PWM을 먼저 0으로 떨어뜨리고 REVERSE_DEADTIME_MS 뒤에 시작
//        (VNH2SP30 계열은 회전 중 방향 반전에서 전류 스파이크가 크다)
//
//  ★ 켤 때 주의 ★
//    부팅 순간의 위치를 '기본위치(0단계)'로 간주한다. 브레이크가 밟혀 있는 상태로 켜면
//    단계 모델이 그만큼 어긋나므로, 확실하지 않으면 켠 직후 "0"을 한 번 넣어 복귀시킨다.
//
//  동작 : delay() 미사용, millis() 기준 논블로킹 (카운트는 인터럽트로만 갱신)
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// --- 리니어(브레이크) 모터 : MD20A ---
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// --- 엔코더 (D2/D3 고정) ---
// Mega 2560의 외부 인터럽트 핀은 D2, D3, D18~D21뿐이고, 이 코드는 그중 D2/D3의
// INT4/INT5 벡터에 직접 붙는다. readAB()의 포트 읽기와 ISR 이름 둘 다 D2/D3 전용이라
// 핀을 바꾸려면 두 곳을 함께 고쳐야 한다(이유는 아래 ISR 주석 참고).
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;


// ================= 리니어 방향 규약 =================
// 정방향(체결, 로드가 나옴) = LOW / 역방향(해제, 로드가 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= 단계별 위치 (엔코더 카운트, 여기서 조절) =================
// 기본위치를 0으로 두고 '밟는 방향으로 몇 카운트인가'로 적는다.
//
// ★ 고치는 곳은 POS_FULL 하나뿐이다 ★
//   1단계 위치는 최대위치에서 비율로 자동 계산되므로, 행정을 다시 재서 최대값만 바꾸면
//   1단계도 따라온다(두 숫자를 따로 적어 두면 한쪽만 고치는 실수가 반드시 생긴다).
//   비율은 POS_MID_NUM/POS_MID_DEN으로 정한다 — ★현재 1/3★ (약한 브레이킹을 더 약하게).
//   ※ 정수 나눗셈이라 나머지는 내림된다(250 x 1/3 = 83.3 -> 83). 카운트 1개는 0.044도라
//     무해하다.
//
// linear_0803_pot.ino 실측(2026-08-04)에서 전체 행정이 약 400카운트(=17.6도)였다.
// ★[0804-2] 최대치를 300 -> 250으로 줄였다★ (행정의 약 63%. 표의 신뢰구간 안쪽이고
//   보험이 걸릴 여지도 남는다. 더 밟아야 하면 이 값만 올리면 된다)
const long POS_FULL    = 250;   // ★ 최대위치(2단계)의 변화량 — 여기만 고친다 ★
const long POS_MID_NUM = 1;     // 1단계 = POS_FULL x (NUM/DEN)  ★현재 1/3★
const long POS_MID_DEN = 3;

const long POS_LEVEL0 = 0;                                   // 0 : 기본위치 (브레이크 놓음)
const long POS_LEVEL1 = POS_FULL * POS_MID_NUM / POS_MID_DEN;  // 1 : 약한 브레이킹 (1/3 = 83)
const long POS_LEVEL2 = POS_FULL;                            // 2 : 최대위치 (250)
const long LEVEL_POS[3] = { POS_LEVEL0, POS_LEVEL1, POS_LEVEL2 };
const uint8_t LEVEL_MAX = 2;


// ================= 구동 PWM =================
// ★ 항상 이 값으로 구동한다 ★ 실측표(아래)가 이 PWM에서 잰 값이므로, 이 값을 바꾸면
//   표 전체가 무효가 된다 — linear_0803_speed.ino를 같은 PWM으로 다시 돌려야 한다.
const int DRIVE_PWM = 255;


// ================= [0804-2] 실측표 : 구동시간(ms) -> 이동량(카운트) =================
// ★ 출처 : linear_0803_speed.ino, 2026-08-04, PWM 255, '정착 후' 값 ★
//   10~40ms 구간은 값이 4~56으로 튀어 신뢰할 수 없어 표에서 뺐다(기동 마찰 구간이라
//   그 시간에는 움직이다 말다 한다). 그래서 표는 50ms부터 시작한다.
//
//   ★ 표를 보면 선형이 아니다 ★ 50~140ms는 약 0.9카운트/ms인데 150ms부터 1.8카운트/ms로
//     빨라진다(기동 마찰을 벗어나며 가속). 그래서 단일 기울기(예: 카운트 = ms x 1.3)로
//     계산하지 않고 ★표를 구간 선형보간★ 한다. 이 구부러짐이 이 방식의 핵심 근거다.
//
//   재측정하면 두 배열만 갈아끼우면 된다(길이는 SPD_N으로 함께 고칠 것).
//   ※ 80/90ms(113,111), 180/190ms(238,238)처럼 값이 뒤집히거나 같은 구간이 있다.
//     역보간이 분모 0을 만나면 그 구간의 '짧은 쪽 시간'을 쓴다 — 짧게 미는 것이 안전 방향.
const uint8_t  SPD_N = 19;
const uint16_t SPD_MS[SPD_N]  = {  50,  60,  70,  80,  90, 100, 110, 120, 130, 140,
                                  150, 160, 170, 180, 190, 200, 210, 220, 230 };
const int16_t  SPD_CNT[SPD_N] = {  64,  91, 108, 113, 111, 120, 122, 135, 136, 147,
                                  175, 197, 218, 238, 238, 253, 266, 293, 317 };

// 표의 첫 점(50ms=64카운트)보다 작은 이동은 시간으로 제어할 수 없다(위 10~40ms 참고).
//   - 이 값 미만이면 아예 구동하지 않고 단계만 확정한다.
//   - 이 값 ~ 64카운트 구간은 첫 점까지 비례로 시도하되 로그에 '저신뢰'를 붙인다.
const long MIN_MOVE_COUNTS = 30;

// 표 밖(317카운트 초과)은 마지막 두 점 기울기로 외삽한다. 그 결과의 상한.
// ★ 안전 상한이다 ★ 어떤 계산 결과도 이 시간보다 길게 밀지 않는다.
const unsigned long MAX_DRIVE_MS = 300;

// ★ 역방향(들어가는 방향) 시간 보정 [%] ★
//   실측은 FWD(밟는 방향)만 했다. REV는 브레이크 스프링이 복귀를 도와 더 빠를 수 있다.
//   100 = 표 그대로. 실차에서 REV가 매번 목표를 넘으면(보험 by=ENC가 자주 걸리면) 이 값을
//   80, 70 …으로 줄인다. 정확히 하려면 linear_0803_speed.ino의 방향을 바꿔 재측정할 것.
const long REV_TIME_PCT = 100;

// "0"(기본위치) 명령의 열린루프 복귀 시간. 하드스톱까지 확실히 밀 만큼 넉넉하게.
// ★ 이것만은 표를 쓰지 않는다 ★ 목표가 '카운트'가 아니라 '끝까지'이기 때문이다.
const unsigned long HOME_DRIVE_MS = 500;


// ================= [0804-2] 엔코더 = 보험 (제어에서 빠졌다) =================
// 시간이 끝나기 전에 목표 카운트에 닿으면 끊는다. 정상 동작에서는 걸리지 않는다 —
// 표의 값이 '정착 후'라서, 시간이 끝나는 순간의 카운트는 목표보다 30~70 작기 때문이다.
// 그래서 이것이 걸렸다면 '표가 실제보다 느리게 잡혀 있다'는 신호다(로그 by=ENC).
const long POS_TOLERANCE = 5;    // 보험 판정 여유 (목표선을 이만큼 앞두면 도달로 본다)

// 영점 대비 이 카운트를 넘으면 즉시 정지 (최종 보험).
// 전 행정이 약 400이므로 그 안쪽에 둔다. 발로 밟은 상태에서 명령이 겹칠 때를 대비한 것.
// ★ 밟는 방향 구동에만 적용한다 ★ 해제 방향에 걸면 깊이 밟힌 상태에서 브레이크가 풀리지
//   않는 사고가 난다(updateState의 (1) 주석 참고).
const long ENC_HARD_MAX = 330;

// PWM을 끊은 뒤 관성이 멈추기를 기다리는 시간. 이 뒤에 영점을 잡는다.
const unsigned long SETTLE_MS = 100;

// 구동 중에 새 명령이 왔을 때 PWM 0으로 두는 시간 (방향 반전 보호)
const unsigned long REVERSE_DEADTIME_MS = 30;

// ★ 대기 중 자동 재영점 ★ 엔코더가 이 시간 동안 IDLE_QUIET_BAND 이내로 머물면
//   그 지점을 지금 단계의 새 기준점으로 잡는다 (단계는 그대로 유지).
const unsigned long IDLE_ZERO_MS   = 3000;
const long          IDLE_QUIET_BAND = 2;   // ±이 범위 안이면 '안 움직였다'로 본다

// 출력(관찰) 주기. linear_0803_pot.ino와 같은 5ms로 둬서 두 코드의 로그를 그대로 비교할
// 수 있게 했다. 5ms = 200줄/초 x 약 7바이트 = 1.4KB/s 로 115200baud(11.5KB/s)에 여유가 있다.
// ★ 이 값은 '얼마나 자세히 보는가'만 정한다 — 제어·정지판정과는 무관하다 ★
const unsigned long REPORT_MS = 5;

const unsigned long BAUD = 115200;


// ================= 엔코더 스펙 / 옵션 =================
const long PPR            = 2048;      // A상 1회전당 펄스 수
const long COUNTS_PER_REV = PPR * 4;   // x4 -> 8192 (참고용)

// 토템폴(T) 출력이라 풀업 없이 INPUT이 맞다. 실물이 오픈컬렉터(N)면 true로.
const bool USE_PULLUP = false;

// ★ 밟는 방향으로 움직였는데 값이 음수로 나오면 true로 ★ (A/B를 바꿔 꽂는 것과 같은 효과)
//   이 코드는 "밟는 방향 = 카운트 증가"를 전제로 DIR을 고르므로, 부호가 뒤집힌 채로 두면
//   반대 방향으로 달려간다. 여기만 고치면 전부 맞는다.
const bool INVERT_DIR = false;

// 유실 엣지 보정 (linear_0803_pot.ino와 동일한 이유)
//   A와 B가 '동시에' 바뀐 것으로 보이면 중간 상태를 하나 건너뛴 것이므로 실제로는 정확히
//   2카운트가 지났다. 방향만 알 수 없어 직전 진행 방향을 이어 붙인다. 버리면 빠른 쪽
//   방향만 덜 세어져 기준점이 한쪽으로 밀린다.
const bool RECOVER_LOST_EDGES = true;


// ================= x4 쿼드러처 전이 표 =================
// 상태 = (A<<1)|B, 인덱스 = (이전<<2)|현재.
// +1/-1 = 정상 전이(부호=방향), 0 = 변화 없음 또는 두 비트 동시 변화(엣지 유실).
const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};


// ================= ISR이 갱신하는 값 =================
// long은 4바이트라 읽기가 원자적이지 않다 -> loop에서 읽을 때 noInterrupts()로 감싼다.
volatile long     enc_count = 0;   // ★ 지금 영점 대비 변화량 ★
volatile uint8_t  enc_state = 0;   // 직전 (A<<1)|B
volatile uint16_t enc_err   = 0;   // 엣지 유실 횟수
volatile int8_t   enc_dir   = 0;   // 마지막으로 확정된 진행 방향 (유실 보정용)


// ================= 상태머신 =================
//   ST_IDLE     : 무출력 대기. 사람이 페달을 밟아 엔코더가 변해도 그냥 둔다(표시만)
//   ST_MOVE     : ★표에서 뽑은 시간만큼★ 단방향 구동 (엔코더는 보험으로만 본다)
//   ST_HOME     : "0" 명령. 들어가는 방향으로 시간만큼 구동 (엔코더 무시, 열린루프)
//   ST_SETTLE   : PWM 0으로 관성 정착 대기 -> 끝나면 영점 리셋 + 단계 확정
//   ST_DEADTIME : 구동 중 새 명령을 받아 PWM을 끊고 기다리는 중 (방향 반전 보호)
enum MoveState { ST_IDLE, ST_MOVE, ST_HOME, ST_SETTLE, ST_DEADTIME };
MoveState state = ST_IDLE;

uint8_t cur_level = 0;         // 지금 물려 있다고 보는 단계 (부팅 위치 = 0단계)
uint8_t tgt_level = 0;         // 진행 중인 구동이 끝나면 확정될 단계
long    move_target   = 0;     // 지금 영점 대비 목표 변화량 (보험 판정선)
int8_t  move_dir_sign = 0;     // +1 = 밟는 방향, -1 = 들어가는 방향 (시작 시 고정)
unsigned long drive_start_t = 0;   // MOVE/HOME 구동 시작 시각
unsigned long settle_start_t = 0;
unsigned long deadtime_t = 0;
long stop_count = 0;           // PWM을 끊은 순간의 카운트 (관성분 진단용)

// [0804-2] 시간 기반 구동 상태
unsigned long planned_ms = 0;      // ★표에서 뽑은, 이번에 밀 시간★ (주 종료조건)
long move_need  = 0;               // 이번 구동에서 움직여야 하는 양 (부호 포함. HOME은 0)
long move_start_count = 0;         // 구동 시작 시점의 엔코더 값 (실제 이동량 계산용)
const char* stop_why = "-";        // 무엇이 구동을 끝냈나 : TIME / ENC / HARDMAX / HOME / SKIP

int8_t pending_cmd = -1;       // 대기 중인 명령 (-1 = 없음, 0/1/2)

int  linear_output = 0;        // 지금 리니어에 주고 있는 PWM (디버그용)
unsigned long report_t = 0;    // 출력주기 타이머

// 대기 중 자동 재영점 감시 (ST_IDLE 전용)
long          quiet_ref = 0;   // '안 움직였다'를 재는 기준 카운트
unsigned long quiet_t   = 0;   // 그 기준으로 조용해진 시각


// ================= 함수 선언 =================
static inline uint8_t readAB();
long encRead();
void zeroHere(bool quiet);
void linearStop();
void linearDrive(uint8_t dir, int p);
void onCommand(uint8_t lv, unsigned long now);
void beginCommand(uint8_t lv, unsigned long now);
unsigned long msForCounts(long counts);
void startMove(uint8_t lv, unsigned long now);
void startHome(unsigned long now);
void enterSettle(unsigned long now, long at);
void finishAt(uint8_t lv, unsigned long now);
void updateState(unsigned long now);
void armIdleZero(unsigned long now);
void updateIdleZero(unsigned long now);
void report(unsigned long now);
void pollSerial(unsigned long now);


// ================= A/B 레벨 읽기 =================
// ★ Mega 2560 : D2 = PE4, D3 = PE5 ★ (Uno와 포트가 다르다)
//   포트를 한 번에 읽으므로 빠르고(약 0.2us), A/B가 서로 어긋나 읽힐 일도 없다.
static inline uint8_t readAB() {
  uint8_t p = PINE;
  uint8_t a = (p >> 4) & 0x01;   // PE4 = D2
  uint8_t b = (p >> 5) & 0x01;   // PE5 = D3
  return (uint8_t)((a << 1) | b);
}


// ================= 엔코더 1엣지 처리 (두 ISR이 공유) =================
// always_inline이라 아래 두 벡터 안에 코드가 그대로 박힌다(함수 호출 오버헤드 0).
// 시리얼 출력은 절대 하지 않는다(느리고 위험).
static inline void encStep() __attribute__((always_inline));
static inline void encStep() {
  uint8_t cur  = readAB();
  uint8_t prev = enc_state;
  if (cur == prev) return;                    // 같은 상태로의 인터럽트(글리치) — 무시
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
  enc_dir = step;                    // 정상 전이에서만 방향을 갱신한다
  enc_count += INVERT_DIR ? -step : step;
}


// ================= ISR 벡터 직결 (INT4=D2, INT5=D3) =================
// ★ attachInterrupt를 쓰지 않는 이유 ★
//   attachInterrupt는 공용 핸들러에서 함수 포인터 테이블을 거쳐 우리 함수를 '호출'하며,
//   레지스터를 대량 저장/복원해 1회 약 5~6us가 든다. 8192카운트/회전이라 엣지 발생률이
//   약 136.5 x RPM 이므로 그 오버헤드가 곧 누락 한계를 정한다(약 1,200RPM에서 CPU 100%).
//   벡터에 직접 붙이면 약 2~3us로 줄어 여유가 두 배가 된다(약 2,500RPM).
ISR(INT4_vect) { encStep(); }
ISR(INT5_vect) { encStep(); }


// ================= 엔코더 값 읽기 (loop 전용) =================
// 4바이트 읽기는 원자적이지 않으므로 ISR과 겹치지 않게 감싼다.
long encRead() {
  long c;
  noInterrupts();
  c = enc_count;
  interrupts();
  return c;
}


// ================= 영점 재설정 =================
// 지금 위치를 0으로 만든다.
// ★ enc_state는 건드리지 않는다 ★ 그것은 진행 중인 쿼드러처 전이의 판정 기준이라,
//   여기서 다시 읽어 넣으면 마침 대기 중이던 엣지를 한 카운트 잃을 수 있다.
void zeroHere(bool quiet) {
  noInterrupts();
  enc_count = 0;
  enc_err   = 0;   // 오차 원인을 새로 세기 위해 유실 카운터도 함께 초기화
  enc_dir   = 0;   // 직전 구간의 방향 추정을 새 구간에 물려 쓰지 않도록
  interrupts();
  if (!quiet) Serial.println("# ZERO");   // '#'로 시작해 시리얼 플로터가 무시한다
}


// ================= 모터 출력 =================
void linearStop() {
  analogWrite(LINEAR_PWM_PIN, 0);
  linear_output = 0;
}

// 방향은 LINEAR_FWD(정, 체결) / LINEAR_REV(역, 해제) 둘 중 하나.
// ★ DIR을 먼저 쓰고 PWM을 나중에 쓴다 ★ 순서가 반대면 전환 순간 잘못된 방향으로 한 번
//   힘이 들어간다.
void linearDrive(uint8_t dir, int p) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  linear_output = constrain(p, 0, 255);
  analogWrite(LINEAR_PWM_PIN, linear_output);
}


// ================= 명령 접수 =================
// 구동 중에 명령이 오면 곧바로 시작하지 않는다.
//   - MOVE/HOME 중 : PWM을 0으로 떨어뜨리고 REVERSE_DEADTIME_MS 뒤에 시작한다.
//     방향이 같을 때도 예외를 두지 않는다 — 명령 변경은 드물고 30ms면 전 행정의 몇 %도
//     못 움직이는데, 예외를 두면 '회전 중 방향 반전'이라는 위험한 경로가 하나 생긴다.
//   - SETTLE 중 : 정착을 먼저 끝내야 영점·단계가 확정된다. 그 전에 새 구동을 시작하면
//     위치 모델이 어긋나므로 pending으로 미룬다(대기는 SETTLE_MS 이내라 짧다).
//   - DEADTIME 중 : 마지막 명령으로 덮어쓴다.
void onCommand(uint8_t lv, unsigned long now) {
  if (state == ST_MOVE || state == ST_HOME) {
    linearStop();
    pending_cmd = (int8_t)lv;
    state = ST_DEADTIME;
    deadtime_t = now;
    return;
  }
  if (state == ST_SETTLE || state == ST_DEADTIME) {
    pending_cmd = (int8_t)lv;
    return;
  }
  beginCommand(lv, now);
}

void beginCommand(uint8_t lv, unsigned long now) {
  if (lv == 0) startHome(now);
  else         startMove(lv, now);
}


// ================= [0804-2] 이동량(카운트) -> 구동시간(ms) : 실측표 역보간 =================
// ★ 이 함수가 이 코드의 심장이다 ★ 엔코더로 '언제 멈출까'를 판단하는 대신, 미리 '얼마나
//   밀까'를 여기서 결정한다. 표가 선형이 아니므로(50~140ms 0.9카운트/ms, 150ms↑ 1.8) 단일
//   기울기를 쓰지 않고 구간마다 보간한다.
//   입력은 항상 양수(이동량의 크기). 방향 보정(REV_TIME_PCT)은 호출측에서 한다.
unsigned long msForCounts(long counts) {
  if (counts <= 0) return 0;

  // ── 표 첫 점(50ms=64카운트) 미달 : 원점과 첫 점을 이어 비례 ──
  //   10~40ms 실측이 4~56으로 튀었던 구간이라 신뢰도가 낮다. 호출측이 로그에 표시한다.
  if (counts <= (long)SPD_CNT[0]) {
    return (unsigned long)((long)SPD_MS[0] * counts / (long)SPD_CNT[0]);
  }

  // ── 표 안 : counts를 처음 넘어서는 구간에서 선형보간 ──
  for (uint8_t i = 1; i < SPD_N; i++) {
    if (counts <= (long)SPD_CNT[i]) {
      long d = (long)SPD_CNT[i] - (long)SPD_CNT[i - 1];
      if (d <= 0) return SPD_MS[i - 1];   // 뒤집힌/평탄 구간 -> 짧은 쪽 시간(안전 방향)
      return (unsigned long)((long)SPD_MS[i - 1]
             + (counts - (long)SPD_CNT[i - 1])
               * ((long)SPD_MS[i] - (long)SPD_MS[i - 1]) / d);
    }
  }

  // ── 표 밖(317 초과) : 마지막 두 점 기울기로 외삽하고 MAX_DRIVE_MS로 자른다 ──
  long d  = (long)SPD_CNT[SPD_N - 1] - (long)SPD_CNT[SPD_N - 2];
  long ms = (long)SPD_MS[SPD_N - 1];
  if (d > 0) {
    ms += (counts - (long)SPD_CNT[SPD_N - 1])
          * ((long)SPD_MS[SPD_N - 1] - (long)SPD_MS[SPD_N - 2]) / d;
  }
  if (ms > (long)MAX_DRIVE_MS) ms = (long)MAX_DRIVE_MS;
  if (ms < 0) ms = 0;
  return (unsigned long)ms;
}


// ================= "1"/"2" : 표에서 뽑은 시간만큼 구동 (엔코더는 보험) =================
void startMove(uint8_t lv, unsigned long now) {
  long rel = LEVEL_POS[lv] - LEVEL_POS[cur_level];   // 현재 단계 기준 목표 변화량
  long cur = encRead();
  long err = rel - cur;                              // 실제로 움직여야 하는 양
  long mag = labs(err);

  // ★ 시간으로 제어할 수 없는 소량은 아예 움직이지 않는다 ★
  //   실측 10~40ms 구간이 4~56카운트로 튀었으므로, 이보다 작은 이동은 '조금 밀기'가 아니라
  //   '얼마나 갈지 모르는 밀기'가 된다. 움직이지 않고 단계만 확정하는 쪽이 안전하다.
  if (mag < MIN_MOVE_COUNTS) {
    linearStop();
    stop_count       = cur;
    move_start_count = cur;
    move_need        = 0;
    tgt_level        = lv;
    stop_why         = "SKIP";
    finishAt(lv, now);
    return;
  }

  unsigned long ms = msForCounts(mag);
  if (err < 0) ms = ms * (unsigned long)REV_TIME_PCT / 100UL;   // 역방향 보정(미실측)
  if (ms > MAX_DRIVE_MS) ms = MAX_DRIVE_MS;                     // 최종 상한

  move_target      = rel;                // 보험 판정선 (영점 대비)
  move_need        = err;
  move_start_count = cur;
  move_dir_sign    = (err > 0) ? +1 : -1;   // ★ 방향은 여기서 한 번만 정한다 ★
  planned_ms       = ms;
  tgt_level        = lv;
  drive_start_t    = now;
  state            = ST_MOVE;

  linearDrive(move_dir_sign > 0 ? LINEAR_FWD : LINEAR_REV, DRIVE_PWM);

  Serial.print("# MOVE L");
  Serial.print(lv);
  Serial.print(" need=");
  Serial.print(err);
  Serial.print(" t=");
  Serial.print(ms);
  Serial.print("ms");
  if (mag <= (long)SPD_CNT[0]) Serial.print("  ★저신뢰(표 첫점 미달)★");
  Serial.println();
}


// ================= "0" : 들어가는 방향으로 시간 구동 (열린루프 재영점) =================
// ★ 엔코더를 보지 않는다 ★ 이 동작의 목적은 '목표 카운트에 맞추기'가 아니라 하드스톱까지
//   밀어붙여 기준점을 되찾는 것이다(증분 엔코더의 누적 드리프트 대책 — linear_0803_pot 헤더).
//   그래서 이전에 무엇을 입력했든, 지금 위치가 어디든 항상 같은 시간만큼 역방향 구동한다.
void startHome(unsigned long now) {
  move_dir_sign    = -1;
  move_target      = 0;        // 쓰지 않는다 (열린루프)
  move_need        = 0;        // 0 = 로그에서 want/got을 생략한다는 표시
  move_start_count = encRead();
  planned_ms       = HOME_DRIVE_MS;
  tgt_level        = 0;
  drive_start_t    = now;
  state            = ST_HOME;

  linearDrive(LINEAR_REV, DRIVE_PWM);

  Serial.print("# HOME ");
  Serial.print(HOME_DRIVE_MS);
  Serial.println("ms (엔코더 무시, 역방향)");
}


// ================= 정지 -> 관성 정착 대기 =================
void enterSettle(unsigned long now, long at) {
  linearStop();
  stop_count     = at;
  settle_start_t = now;
  state          = ST_SETTLE;
}


// ================= 단계 확정 + 영점 리셋 =================
// ★ 이 함수를 지나야 '현재 단계 = lv, 영점 = 지금 위치'가 성립한다 ★
// ★[0804-2] 로그가 표 보정의 근거다★ want(움직여야 했던 양)과 got(실제 움직인 양)을 함께
//   찍는다. got이 계통적으로 작으면 표의 시간이 짧은 것이고, 크면 긴 것이다. by=ENC가 자주
//   나오면 표가 실제보다 느리게 잡혀 있다는 뜻(REV라면 REV_TIME_PCT를 줄인다).
void finishAt(uint8_t lv, unsigned long now) {
  long settled = encRead();
  long coast   = settled - stop_count;         // PWM을 끊은 뒤 관성으로 더 밀려간 양
  long got     = settled - move_start_count;   // 이번 구동의 실제 이동량

  cur_level = lv;
  zeroHere(true);                       // "# ZERO"는 생략하고 아래 한 줄로 합쳐 보고
  state = ST_IDLE;
  armIdleZero(now);                     // 여기서부터 3초 정지 감시 시작

  Serial.print("# DONE L");
  Serial.print(lv);
  Serial.print(" by=");
  Serial.print(stop_why);
  if (move_need != 0) {
    Serial.print(" want=");
    Serial.print(move_need);
    Serial.print(" got=");
    Serial.print(got);
    Serial.print(" (");
    if (got - move_need > 0) Serial.print('+');
    Serial.print(got - move_need);
    Serial.print(")");
  }
  Serial.print(" coast=");
  Serial.print(coast);
  Serial.println(" -> ZERO");
}


// ================= 대기 중 자동 재영점 : 감시 시작(재무장) =================
// 모터가 멈춘 시점 / 엔코더가 마지막으로 움직인 시점부터 IDLE_ZERO_MS를 센다.
void armIdleZero(unsigned long now) {
  quiet_ref = encRead();
  quiet_t   = now;
}


// ================= 대기 중 자동 재영점 (ST_IDLE에서만, 매 루프) =================
// 3초 동안 엔코더가 IDLE_QUIET_BAND 이내로 머물면 그 지점을 '지금 단계의 기준점'으로 잡는다.
//   ★ 단계(cur_level)는 건드리지 않는다 ★ 그래서 1단계에서 재영점된 뒤 2를 넣으면
//     그 자리에서 +(POS_LEVEL2 - POS_LEVEL1)만큼만 간다.
//   구동 중(MOVE/HOME)이나 정착 대기(SETTLE)·데드타임에는 감시하지 않는다 — 그 구간의
//   위치 변화는 우리가 일부러 만든 것이고, 영점은 finishAt이 잡는다.
void updateIdleZero(unsigned long now) {
  if (state != ST_IDLE) return;

  long cur = encRead();

  // 움직였다 -> 기준을 지금 값으로 옮기고 타이머 재시작
  if (labs(cur - quiet_ref) > IDLE_QUIET_BAND) {
    quiet_ref = cur;
    quiet_t   = now;
    return;
  }

  if (now - quiet_t < IDLE_ZERO_MS) return;

  // 3초 조용함 확정. 타이머는 항상 재시작한다(다음 3초를 새로 센다).
  quiet_t = now;

  // 이미 기준점 위에 있으면 할 일이 없다. zeroHere를 부르면 enc_err/enc_dir까지
  // 초기화되어 진단 정보만 잃으므로 조용히 넘어간다(로그도 내지 않는다).
  if (labs(cur) <= IDLE_QUIET_BAND) {
    quiet_ref = cur;
    return;
  }

  zeroHere(true);
  quiet_ref = 0;

  Serial.print("# REZERO ");
  Serial.print(cur);        // 이만큼 밀린 지점을
  Serial.print(" -> L");
  Serial.print(cur_level);  // 이 단계의 새 기준점으로 삼았다
  Serial.println(" 기준");
}


// ================= 상태 갱신 (★매 루프★ 호출) =================
// ★ 어떤 주기에도 걸지 않는다 ★ 주기를 두면 그만큼 정지가 늦어지고, PWM 255에서는 그 지연이
// 곧 카운트 오차가 된다(약 1카운트/ms). 시간 경계를 ms 단위로 지키려면 매 루프 봐야 한다.
void updateState(unsigned long now) {
  switch (state) {

    // ★[0804-2] 종료조건 3개 — 우선순위 순서대로 본다 ★
    //   먼저 오는 것이 이긴다. 정상 동작에서는 항상 (3) TIME이 이긴다.
    case ST_MOVE: {
      long cur = encRead();

      // (1) 최종 보험 : 너무 많이 밟혔다. ★밟는 방향(+) 구동에만 적용한다★
      //     들어가는 방향에는 걸지 않는다 — 걸면 이미 깊이 밟힌 상태에서 '해제'조차 막혀
      //     브레이크가 풀리지 않는다(절댓값으로 보면 그 사고가 난다). 해제 쪽 과주행은
      //     기본위치 하드스톱이 받아주고 시간 상한도 있어 위험하지 않다.
      if (move_dir_sign > 0 && cur >= ENC_HARD_MAX) {
        stop_why = "HARDMAX";
        enterSettle(now, cur);
        return;
      }

      // (2) 보험 : 목표선에 먼저 닿았다. '그 방향으로 넘었는가'로 보므로 지나쳐도 잡힌다.
      //     여기서 끊겼다면 표가 실제보다 느리게 잡혀 있다는 신호다(로그 by=ENC).
      bool reached = (move_dir_sign > 0) ? (cur >= move_target - POS_TOLERANCE)
                                         : (cur <= move_target + POS_TOLERANCE);
      if (reached) {
        stop_why = "ENC";
        enterSettle(now, cur);
        return;
      }

      // (3) 주 종료조건 : 표에서 뽑은 시간이 지났다.
      //     ★타임아웃을 따로 두지 않는다★ planned_ms 자체가 상한이라 반드시 끝난다.
      if (now - drive_start_t >= planned_ms) {
        stop_why = "TIME";
        enterSettle(now, cur);
      }
      return;
    }

    case ST_HOME:
      if (now - drive_start_t >= HOME_DRIVE_MS) {
        stop_why = "HOME";
        enterSettle(now, encRead());
      }
      return;

    case ST_SETTLE:
      if (now - settle_start_t >= SETTLE_MS) {
        finishAt(tgt_level, now);
        // 정착을 기다리는 동안 들어온 명령이 있으면 지금 시작한다
        if (pending_cmd >= 0) {
          uint8_t lv = (uint8_t)pending_cmd;
          pending_cmd = -1;
          beginCommand(lv, now);
        }
      }
      return;

    case ST_DEADTIME:
      if (now - deadtime_t >= REVERSE_DEADTIME_MS) {
        int8_t lv = pending_cmd;
        pending_cmd = -1;
        state = ST_IDLE;
        armIdleZero(now);
        if (lv >= 0) beginCommand((uint8_t)lv, now);
      }
      return;

    case ST_IDLE:
    default:
      return;
  }
}


// ★ 주기적으로 출력을 다시 인가하는 함수는 두지 않았다 ★
//   PWM이 DRIVE_PWM(255) 상수이므로, 구동 개시는 startMove()/startHome()에서 한 번,
//   정지는 enterSettle()에서 한 번이면 충분하다. 같은 값을 주기마다 다시 쓰는 코드는
//   아무 일도 하지 않으면서 종료 판정만 늦춘다.
//   ※ [0804-2] 이제 정밀도는 '더 자주 판정하기'가 아니라 ★실측표의 정확도★에서 온다.
//     한 카운트 더 맞추고 싶으면 코드를 고치는 대신 linear_0803_speed.ino를 다시 돌려
//     SPD_MS/SPD_CNT를 갱신하는 것이 맞다(구간을 촘촘히 재면 그만큼 정확해진다).


// ================= 출력 (REPORT_MS 주기) =================
// 요구사항대로 '현재 엔코더 변화량' 정수 하나만 찍는다. 값이 안 변해도 매 주기 찍어
// 등간격 표본이 되게 한다(시리얼 플로터 호환).
void report(unsigned long now) {
  if (now - report_t < REPORT_MS) return;
  report_t = now;

  uint16_t err;
  noInterrupts();
  err = enc_err;
  interrupts();

  Serial.println(encRead());

  // ★ 엣지 유실이 생긴 순간에만 한 줄 더 ★ 이 줄이 없는데도 같은 위치에서 값이 틀어지면
  //   전기적 누락이 아니라 기계적 유격이다(1카운트 = 0.044도) — linear_0803_pot 헤더 참고.
  //   ※ 영점 리셋 때 enc_err도 0으로 초기화되므로 '줄어든 경우'는 보고하지 않는다
  //     (그것은 유실이 아니라 우리가 카운터를 지운 것이다).
  static uint16_t last_err = 0;
  if (err != last_err) {
    bool grew = (err > last_err);
    last_err = err;
    if (grew) {
      Serial.print("# ERR ");
      Serial.println(err);
    }
  }
}


// ================= 시리얼 입력 (줄 단위, "0"/"1"/"2"만 인정) =================
// 문자 단위로 받지 않는 이유 : 나중에 목표값을 여러 자리로 받게 되면 입력 도중의 한 글자에
// 반응해 엉뚱하게 움직인다. 줄 단위로 두면 그 위험이 없다.
char rxBuf[8];
uint8_t rxLen = 0;

void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen == 1 && rxBuf[0] >= '0' && rxBuf[0] <= ('0' + LEVEL_MAX)) {
        onCommand((uint8_t)(rxBuf[0] - '0'), now);
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;   // 너무 긴 줄은 버린다
    }
  }
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, LINEAR_FWD);
  linearStop();

  const uint8_t mode = USE_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_A_PIN, mode);
  pinMode(ENC_B_PIN, mode);

  // 현재 레벨을 시작 상태로 채택한 뒤 인터럽트를 건다.
  // (먼저 걸면 enc_state가 0이라 첫 전이가 엉뚱하게 판정될 수 있다)
  enc_state = readAB();
  enc_count = 0;                 // ★ 켜진 순간의 위치가 영점 = 기본위치(0단계) ★
  enc_err   = 0;
  enc_dir   = 0;

  // ── 외부 인터럽트 INT4(D2)·INT5(D3)를 양쪽 엣지(CHANGE)로 직접 설정 ──
  //   INT4~INT7은 EICRB가 담당하고 감지 방식은 2비트다: 00=LOW, 01=양쪽엣지, 10=하강, 11=상승.
  EICRB = (EICRB & ~((1 << ISC41) | (1 << ISC40) | (1 << ISC51) | (1 << ISC50)))
                 | (1 << ISC40) | (1 << ISC50);
  EIFR  = (1 << INTF4) | (1 << INTF5);    // 설정 중에 쌓인 대기 플래그를 지운 뒤
  EIMSK |= (1 << INT4) | (1 << INT5);     // 두 인터럽트를 활성화

  cur_level   = 0;
  tgt_level   = 0;
  state       = ST_IDLE;
  pending_cmd = -1;

  unsigned long now = millis();
  report_t = now;
  armIdleZero(now);   // 켜진 직후부터 3초 정지 감시 (부팅 위치 = 0단계 기준점)

  Serial.println();
  Serial.println("# linear_0804 : 리니어 3단계 위치 이동 테스트");
  Serial.print  ("# 입력 0=기본(역방향 ");
  Serial.print(HOME_DRIVE_MS);
  Serial.print  ("ms) / 1=약1/3(");
  Serial.print(POS_LEVEL1);
  Serial.print  (") / 2=최대(");
  Serial.print(POS_LEVEL2);
  Serial.println(")  ★개행 전송★");
  Serial.print  ("# PWM 고정 ");
  Serial.print(DRIVE_PWM);
  Serial.print  (", 출력 ");
  Serial.print(REPORT_MS);
  Serial.println("ms - 영점 대비 카운트 하나");

  // ★ 표가 제대로 들어갔는지 켤 때 바로 확인 ★ (실측표 기준 83=약 57ms, 250=약 198ms)
  Serial.print  ("# 시간 환산(표) : ");
  Serial.print(POS_LEVEL1);
  Serial.print  ("카운트=");
  Serial.print(msForCounts(POS_LEVEL1));
  Serial.print  ("ms / ");
  Serial.print(POS_LEVEL2);
  Serial.print  ("카운트=");
  Serial.print(msForCounts(POS_LEVEL2));
  Serial.print  ("ms  (역방향 x");
  Serial.print(REV_TIME_PCT);
  Serial.println("%)");
  Serial.println("# 시간이 주(主), 엔코더는 보험 - by=TIME이 정상 / by=ENC는 표가 느린 신호");
  Serial.println("# 지금 위치를 '기본위치(0단계)'로 본다. 밟힌 채 켰다면 먼저 0을 넣을 것");
  Serial.println("# 구동이 끝나면 그 자리를 새 영점으로 (# DONE : want/got/coast)");
  Serial.print  ("# 대기 중 ");
  Serial.print(IDLE_ZERO_MS);
  Serial.println("ms 동안 안 움직이면 그 지점을 지금 단계의 기준점으로 (# REZERO)");
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // ★ 입력을 가장 먼저 본다 ★ 뒤에 두면 주기 판정에 걸려 입력이 늦게 반영된다.
  pollSerial(now);

  // ★ 종료 판정(시간·보험)과 정착 판정은 매 루프 ★
  //   PWM은 상수라 주기적으로 다시 인가할 것이 없다 — 구동 개시/정지는 사건 발생 시 한 번.
  updateState(now);

  // 대기 중 3초 정지 -> 그 지점을 지금 단계의 기준점으로 (updateState가 IDLE로 바꾼
  // 직후에 부르므로, 정착이 끝난 그 순간부터 3초를 센다)
  updateIdleZero(now);

  report(now);
}
