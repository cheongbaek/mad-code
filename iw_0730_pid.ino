// ============================================================
//  인휠모터 좌우 독립 PID 테스트 (Arduino Mega 2560) - 0730 버전
//  kasa_0730_A.ino의 인휠 부분(입력 파서 + FF 보간 + PID + 코스트-캐치 + 폭주감지 +
//  직접 PWM 모드)을 그대로 떼어낸 단독 튜닝용 코드.
//  A보드 코드에서 REF 계산/쓰로틀 계측(A1)/E-stop(13번)/"S," 텔레메트리를 제거하고,
//  대신 튜닝용 평문 출력(20ms마다 pwm·target·speed)을 넣었다.
//
//  iw_0716_pid.ino 대비 변경점
//  ------------------------------------------------------------
//  [1] 핀 배정 갱신 (2026-07-30 실차 확인 완료)
//      왼쪽 펄스 2 / 오른쪽 펄스 21 (PWM 8/9는 기존과 동일)
//      iw_0716_pid.ino는 21(왼쪽)/20(오른쪽)이었다. 현재 확정 배선은 2(왼쪽)/21(오른쪽)이며
//      iw_0730_check.ino(후보 6핀 전수 계측)로 검증되었고 kasa_0730_A.ino와 동일하다.
//
//  [2] 좌우 독립 입력 + 직접 PWM 모드 이식 (iw_0716_pid.ino에는 없던 부분)
//      iw_0716_pid.ino는 좌우 공통 목표펄스 1개만 받았으나, 이 파일은 A보드와
//      동일하게 "<좌>,<우>" 콤마 2값으로 좌/우를 따로 지정할 수 있고 값 범위에
//      따라 모드가 갈린다 (0~15 펄스 / 16~255 직접 PWM / 256↑ 정지).
//      ★ 아래 [3][4] 를 포함한 안전장치(PWM 상한·슬루·폭주감지·기동 블랭킹·중앙값)는
//        A보드와 마찬가지로 '펄스 모드'에만 걸린다. 직접 PWM은 무보호 경로다.
//
//  [3] ★★ 기동 블랭킹 (LAUNCH) — 이번 파일의 핵심 ★★
//      [문제] 실차 로그 확인 결과, "코일에 힘은 들어갔는데 바퀴가 아직 안 도는" 구간에서
//      홀신호에 수백~1500 단위 노이즈가 쏟아진다. 이 모터는 무부하 최대 1320RPM x
//      64펄스/회전(직결구동)이라 20ms 창당 최대 ≈28.2펄스가 물리적 한계이므로 전부 허수다.
//        PWM 60 직접 인가 실측 : 433/720 → 789/1222 → 563/925 → 181/217 → 30/21 →
//                                124/17 → 709/15 → 752/21 → 118/9 → 4/6 → 7/7 → 5/5 → …
//        즉 노이즈 버스트는 약 12~15주기(240~300ms) 지속되고, 바퀴가 구르기 시작하면
//        말끔히 사라진다. (구른 뒤의 잔진동은 [4] 중앙값 필터가 담당)
//
//      [초기 대응이 실패한 이유 — 기록용] 처음에는 "허수 펄스가 들어온 주기는 PID를
//      건너뛰고 직전 pwm을 유지"하는 방식을 썼다. 리셋은 막았지만 램프업 '진행'까지
//      같이 멈춰버려, 램프 도중 노이즈가 시작되면 pwm이 그 지점에 얼어붙었다.
//      target 2의 FF는 70 PWM이고 모터 기동점은 약 60인데 pwm이 40에서 얼면 바퀴는
//      영원히 못 구르고 → 노이즈도 안 그치고 → 실패안전 타이머가 코스트로 내림 →
//      0부터 재램프 → 또 40에서 얼음. 실측 로그에 이 무한루프가 그대로 찍혔다:
//          40 40 2 2 1027 681 …  (pwm이 40에 고정, speed는 계속 허수)
//
//      [현재 방식] 정지 상태에서 목표가 0 → 양수로 바뀌면 LAUNCH 상태로 들어가
//      그 구간 동안 피드백을 아예 보지 않는다(개루프):
//        - pwm = FF(target)까지 PWM_SLEW_MAX(+4/cycle)로 램프. PID/코스트/폭주감지 전부 정지
//        - 따라서 허수 펄스가 얼마가 들어와도 램프가 멈추거나 되돌아가지 않는다
//      LAUNCH 종료 조건은 아래 둘을 '모두' 만족할 때 (또는 LAUNCH_MAX_MS 타임아웃):
//        (a) pwm이 FF(target)에 도달 (램프 완료)
//        (b) 정상 범위 펄스가 LAUNCH_SETTLE_CYCLES 주기 연속 관측
//      ★ (a)가 반드시 필요하다 ★ (b)만 쓰면 램프 초반(pwm 4·8·12)에는 코일 힘이 약해
//        노이즈가 시작되지도 않아 speed가 "0 0 0 0 0"으로 읽히고, 노이즈가 오기도 전에
//        블랭킹이 조기 종료된다. (a)로 "노이즈가 나올 만큼 힘을 준 뒤"를 보장한다.
//      종료 시 pidLastPwm/pidLastErr를 현재값으로 이어받아(pidI=0) PID로 무단절 인계.
//      실측 기준 예상 소요: target 2에서 램프 18주기 + 노이즈 15주기 + 정착 5주기 ≈ 0.8초.
//
//  [4] ★ 주행 중 잔노이즈용 3점 중앙값 필터 (SPEED_MEDIAN_ON) ★
//      바퀴가 구른 뒤에도 단발성으로 튀는 값(예: 1,2 사이에서 갑자기 17)이 남는다.
//      3점 중앙값은 이런 '단발 스파이크'를 완전히 제거하면서 지연은 1주기(20ms)뿐이라
//      차량 관성 시정수에 비하면 무시할 수준이다. (5점은 40ms 지연이라 굳이 쓰지 않음)
//      허수 판정(> PULSE_SANITY_MAX)된 값은 중앙값 버퍼에 아예 넣지 않는다. 그래서
//      노이즈 주기에는 버퍼가 갱신되지 않아 '직전 정상값이 자동으로 대입'되는 효과가
//      나고, [3]의 얼어붙음 없이 PID가 계속 정상 진행한다.
//      SPEED_MEDIAN_ON=false로 두면 원시값이 그대로 PID에 들어간다(반응성 비교용).
//  ------------------------------------------------------------
//  입력 : "<값>" 또는 "<좌값>,<우값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값  : 펄스 전용(0~15), 좌우 동일 목표. 범위 밖/숫자 아님은 무시
//         - 콤마 2값 : 좌/우 독립 — 0~15 펄스(PID) / 16~255 직접 PWM(무보호) / 256↑ 정지
//  출력 : "<pwmL> <pwmR> <tgtL> <tgtR> <rawL> <rawR> <spdL> <spdR> <launch> 0 25" (20ms)
//         raw = 필터 전 원시 펄스, spd = PID가 실제로 본 값(중앙값 필터 후)
//         launch = 0(둘 다 PID) / 1(왼쪽 블랭킹) / 2(오른쪽) / 3(양쪽)
//         마지막 "0 25"는 시리얼 플로터 축 고정용 상수 (iw_0716_pid.ino와 동일 관례)
// ============================================================

// ===== ★ 피드포워드 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =====
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// 목표펄스 -> PWM 보간 (3점 라그랑주 2차보간)
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

// ===== ★ PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =====
float kp[2] = {0.4,  0.4};    // {왼쪽(펄스2-PWM8), 오른쪽(펄스21-PWM9)}
float ki[2] = {0.03, 0.03};
float kd[2] = {0.2,  0.2};

// ===== ★ 값 해석 경계 (A보드와 동일) ★ =====
const int TARGET_MAX = 15;      // 0~15 = 펄스 목표. 단일 값 입력은 이 범위만 유효
const int PWM_DIRECT_MAX = 255; // 16~255 = 직접 PWM (콤마 2값 형식에서만). 256 이상 = 정지

// ===== ★ 코스트-캐치 (튜닝 지점) ★ =====
const int CATCH_MARGIN = 1;   // 목표+이 값(펄스)에서 캐치. 언더슈트 크면 늘리고, 목표 위에 오래 머물면 0

// ===== ★ PWM 상한 (튜닝 지점) ★ =====
// ※ 펄스(PID) 모드에만 적용. 직접 PWM 모드(16~255)는 이 상한을 무시하고 그대로 출력.
const int PWM_MAX = 170;

// ===== ★ PWM 슬루레이트 제한 (튜닝 지점) ★ =====
// 사이클(20ms)당 pwm 상승폭을 제한해 급가속으로 인한 관성 오버슈트를 방지.
// 하강은 제한하지 않음(안전: 감속/정지는 항상 즉시 반영). 직접 PWM 모드에는 미적용.
// 기동 블랭킹(LAUNCH) 중 FF까지 올라가는 램프도 이 값을 쓴다.
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ===== ★ 폭주 감지 (튜닝 지점) ★ =====
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초

// ===== ★ [0730] 허수 펄스 판정 경계 (튜닝 지점) ★ =====
// 무부하 최대 1320RPM x 64펄스/회전(직결구동) 기준 이론 최대 ≈ 28.2펄스/20ms창.
// 실측 노이즈는 85~1500 수준이라 넉넉히 40으로 잡아도 전부 걸러진다 — 정상값을
// 잘못 버리는 쪽이 더 위험하므로 이론 최대보다 여유를 두었다.
const int PULSE_SANITY_MAX = 40;

// 허수로 판정된 주기가 이만큼 연속되면(LAUNCH 종료 후 기준) 홀센서 이상으로 보고
// 코스트(PWM 0) 진입. 0으로 두면 상한 없음. LAUNCH 중에는 노이즈가 정상이라 세지 않는다.
const int NOISE_HOLD_MAX_CYCLES = 50;   // 50주기 = 1초

// ===== ★ [0730] 기동 블랭킹 LAUNCH (튜닝 지점) ★ =====
// 목표가 0 -> 양수로 바뀌고 바퀴가 사실상 멈춰 있을 때 진입. 상세는 파일 상단 [3] 참고.
const int LAUNCH_ENTRY_SPEED_MAX = 1;      // 이 펄스 이하일 때만 '정지 상태'로 보고 진입
const int LAUNCH_SETTLE_CYCLES   = 5;      // 정상 펄스가 이만큼 연속되면 노이즈 종료로 판정
const unsigned long LAUNCH_MAX_MS = 3000;  // 이 시간이 지나면 조건 불충족이어도 PID로 인계

// ===== ★ [0730] 주행 중 3점 중앙값 필터 ★ =====
// true면 PID가 보는 speed를 최근 정상값 3개의 중앙값으로 대체(단발 스파이크 제거, 지연 20ms).
// false면 원시값 직결 — 반응성 비교용으로 껐다 켜볼 수 있게 남겨둔 스위치.
const bool SPEED_MEDIAN_ON = true;
const uint8_t SPEED_MEDIAN_N = 3;   // 3 고정 (median3 함수가 3점 전용)

// ===== 좌/우 PID 상태 (완전 분리) =====
// 주의: Arduino IDE는 함수 프로토타입을 파일 맨 위(커스텀 타입 정의보다 앞)에 자동 삽입한다.
// struct로 상태를 묶으면 그 프로토타입이 struct 정의보다 앞에 삽입되어 컴파일 에러가 남.
// 그래서 기본 타입(int/float/bool) 배열 + 좌(0)/우(1) 인덱스로 상태를 분리한다.
const uint8_t LEFT  = 0;   // 2번 펄스 피드백 -> 8번 PWM (왼쪽)
const uint8_t RIGHT = 1;   // 21번 펄스 피드백 -> 9번 PWM (오른쪽)

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터
int   noiseCnt[2]    = {0, 0};   // 연속 허수 펄스 주기 카운터 (LAUNCH 중엔 세지 않음)

// [0730] 기동 블랭킹 상태
bool          launching[2]  = {false, false};
unsigned long launch_t[2]   = {0, 0};   // LAUNCH 진입 시각 (타임아웃용)
int           settleCnt[2]  = {0, 0};   // 정상 펄스 연속 주기 카운터

// [0730] 중앙값 필터용 링버퍼 (정상 판정된 값만 들어간다)
int     spdBuf[2][SPEED_MEDIAN_N] = {{0, 0, 0}, {0, 0, 0}};
uint8_t spdPos[2] = {0, 0};

// ===== 구동 모드 (좌/우 독립, A보드와 동일) =====
const uint8_t DRIVE_PULSE = 0;   // 펄스 목표 PID 주행 (0~15)
const uint8_t DRIVE_PWM   = 1;   // 직접 PWM 출력 (16~255, 무보호)

uint8_t driveMode[2] = {DRIVE_PULSE, DRIVE_PULSE};
int     pwmDirect[2] = {0, 0};   // 직접 PWM 모드에서 출력할 값

// ===== 핀 (kasa_0730_A.ino와 동일, 2026-07-30 실차 확인 완료) =====
const uint8_t PWM_PIN_L  = 8;
const uint8_t PWM_PIN_R  = 9;
const uint8_t HALL_PIN_L = 2;    // 왼쪽 펄스
const uint8_t HALL_PIN_R = 21;   // 오른쪽 펄스

// ===== 제어주기 =====
const unsigned long CONTROL_WINDOW_MS = 20;

// ===== 변수선언 =====
volatile long encCountL = 0;   // 왼쪽 펄스
volatile long encCountR = 0;   // 오른쪽 펄스
unsigned long wheel_t = 0;
int target[2] = {0, 0};        // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)

// 텔레메트리용 (PID가 실제로 본 값 / 필터 전 원시값)
int rawSpeed[2] = {0, 0};
int useSpeed[2] = {0, 0};

// ===== 시리얼 입력 버퍼 =====
char rxBuf[48];
uint8_t rxLen = 0;

// 부호 없는 정수(숫자만)인지 검사 — 음수/소수/그 외 문자는 여기서 걸러짐
bool isValidNumber(const char* s) {
  if (!s || *s == '\0') return false;
  for (uint8_t k = 0; s[k] != '\0'; k++) {
    if (!isdigit((unsigned char)s[k])) return false;
  }
  return true;
}

// 숫자 토큰 -> 값. 숫자가 아니면 -1(줄 전체 무시용).
// 4자리 이상은 어차피 255 초과 = 정지이므로 atoi 오버플로 없이 256으로 통일.
long parseValue(const char* s) {
  if (!isValidNumber(s)) return -1;
  if (strlen(s) > 3) return 256;
  return atoi(s);
}

// ===== [0730] 3점 중앙값 (정렬 없이 비교 3회) =====
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

// ===== [0730] 기동 블랭킹 진입/종료 =====
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

// ===== 펄스 모드 목표 적용 (A보드 setPulseTarget + 블랭킹 진입 판정) =====
// 직접 PWM 모드에서 복귀할 때는 PID 상태를 리셋하고 코스트로 진입
// (과속 상태면 무동력 감속, 이미 느리면 다음 주기에 바로 캐치되어 PID 재개)
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

    // 직접 PWM 테스트 중 바퀴가 이미 멈춰 있었다면(예: "0,0" 후 펄스 목표) 여기서도
    // 기동 노이즈가 똑같이 발생하므로 블랭킹을 걸어준다. 돌고 있었으면 코스트로 넘긴다.
    if (newTarget > 0 && useSpeed[idx] <= LAUNCH_ENTRY_SPEED_MAX) {
      enterLaunch(idx, now);
    } else {
      launching[idx] = false;
      pidCoasting[idx] = true;
    }
    return;
  }

  // ★ 기동 블랭킹 진입 판정 : 멈춘 상태(0)에서 처음 양수 목표를 받을 때만 ★
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

// ===== 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =====
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

// ===== 입력 파서 (A보드 handleLine과 동일) =====
// "<값>" 단일 = 펄스 전용(0~15, 좌우 공통), "<좌값>,<우값>" = 좌/우 독립(펄스/PWM/정지).
// 형식이 안 맞으면 무시(직전 명령 유지).
void handleLine(char* line, unsigned long now) {
  if (strchr(line, ' ')) return;   // 공백 포함 줄은 형식 오류

  char* comma = strchr(line, ',');
  if (comma) {
    *comma = '\0';
    char* tokR = comma + 1;
    if (strchr(tokR, ',')) return;   // 콤마 2개 이상 → 무시

    long vL = parseValue(line);
    long vR = parseValue(tokR);
    if (vL < 0 || vR < 0) return;

    applySide(LEFT, vL, now);
    applySide(RIGHT, vR, now);
  } else {
    // 단일 값은 무조건 펄스 모드: 0~15만 받고 그 외는 무시
    // (직접 PWM은 콤마 2값 형식으로만 가능 — 일반 주행 경로에서의 오발동 방지)
    long v = parseValue(line);
    if (v < 0 || v > TARGET_MAX) return;

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

// ===== 인휠 PID (FF 보간 + PID + 코스트-캐치 + 폭주 감지) =====
// kasa_0730_A.ino의 PID 로직을 그대로 이식. idx(LEFT/RIGHT)로 자기 상태/게인 배열만
// 참조해 완전히 독립적으로 동작.
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

// ===== [0730] 기동 블랭킹 중 개루프 출력 =====
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

// ===== 한쪽 바퀴 갱신: 모드 분기 + 허수 필터 + 블랭킹/PID =====
int updateWheelSide(uint8_t idx, int raw, unsigned long now) {
  rawSpeed[idx] = raw;

  // 직접 PWM 모드는 무보호 경로: 받은 값을 그대로 출력 (피드백을 아예 쓰지 않음).
  // 펄스 계측/표시는 계속하되, 중앙값 버퍼는 갱신해 둔다 (펄스 모드 복귀 시 기준값용).
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

void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }

void setup() {
  Serial.begin(115200);

  pinMode(HALL_PIN_L, INPUT_PULLUP);
  pinMode(HALL_PIN_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_L), encISR_L, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN_R), encISR_R, CHANGE);

  pinMode(PWM_PIN_L, OUTPUT);
  pinMode(PWM_PIN_R, OUTPUT);
  analogWrite(PWM_PIN_L, 0);
  analogWrite(PWM_PIN_R, 0);

  fillSpeed(LEFT, 0);    // 부팅 시엔 바퀴가 멈춰 있으므로 0으로 채워둔다
  fillSpeed(RIGHT, 0);

  wheel_t = millis();
}

void loop() {
  unsigned long now = millis();
  pollSerial(now);

  // ===== 제어주기 20ms =====
  if (now - wheel_t >= CONTROL_WINDOW_MS) {
    wheel_t += CONTROL_WINDOW_MS;

    noInterrupts();
    long rawL = encCountL;
    long rawR = encCountR;
    encCountL = 0;
    encCountR = 0;
    interrupts();

    int pwmL = updateWheelSide(LEFT,  (int)rawL, now);   // 2번 펄스 -> 8번 PWM
    int pwmR = updateWheelSide(RIGHT, (int)rawR, now);   // 21번 펄스 -> 9번 PWM

    analogWrite(PWM_PIN_L, pwmL);
    analogWrite(PWM_PIN_R, pwmR);

    uint8_t launchFlag = (launching[LEFT] ? 1 : 0) | (launching[RIGHT] ? 2 : 0);

    Serial.print(pwmL);
    Serial.print(" ");
    Serial.print(pwmR);
    Serial.print(" ");
    Serial.print(target[LEFT]);
    Serial.print(" ");
    Serial.print(target[RIGHT]);
    Serial.print(" ");
    Serial.print(rawSpeed[LEFT]);
    Serial.print(" ");
    Serial.print(rawSpeed[RIGHT]);
    Serial.print(" ");
    Serial.print(useSpeed[LEFT]);
    Serial.print(" ");
    Serial.print(useSpeed[RIGHT]);
    Serial.print(" ");
    Serial.print(launchFlag);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(25);
  }
}
