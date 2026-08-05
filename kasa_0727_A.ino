// ============================================================
//  A보드 : 인휠모터 좌우 독립 PID + 조향각 기반 좌우 차동 (Arduino Mega 2560) - 0727 버전
//  kasa_0717_A.ino(+ A1 조향각 차동보정) 기반. 이번 0727 변경점은 아래 6가지.
//
//  [0727-1] ★ 좌/우 핀 배정 실측 재확인 ★
//      개발 중간본(kasa_0727_A_nodiff.ino)에서는 홀센서 핀을 플레이스홀더로 D2/D3에 두고,
//      "펄스-PWM은 같은 모터컨트롤러 소속이어야 PID가 자기 바퀴 속도로 자기 바퀴를
//      제어한다"는 원칙에 따라 PWM_PIN_L/R을 9/8로 맞춰둔 상태였음. 이후 홀센서를
//      kasa_0717_A.ino와 동일한 실제 배선 핀(21=왼쪽/20=오른쪽)으로 되돌리면서 실차로
//      재확인한 결과, PWM_PIN_L=8 / PWM_PIN_R=9(0717과 동일) 조합이 맞는 배선으로 확정됨
//      (PWM은 스왑하지 않음):
//        왼쪽  모터 : 펄스 21 -> PID -> PWM 8
//        오른쪽 모터 : 펄스 20 -> PID -> PWM 9
//      입력("<왼쪽>,<오른쪽>")과 출력("S,<왼쪽>,<오른쪽>")은 항상 이 순서를 유지한다.
//
//  [0727-6] ★ 좌우 차동 방향 반전 수정 (실차 텔레메트리 기반) ★
//      증상 : 오른쪽으로 조향(양수각) 시 왼쪽(바깥쪽)이 느려지고 오른쪽(안쪽)이 빨라짐,
//             왼쪽으로 조향(음수각) 시에도 왼쪽이 계속 더 빠름 — 둘 다 물리적으로 반대.
//        예) 오른쪽 30도 : "S,13,16" "S,13,14" "S,12,15" 등 -> 왼쪽이 계속 더 느림
//            왼쪽  30도 : "S,15,13" "S,16,13" "S,17,13" 등 -> 왼쪽이 계속 더 빠름
//      핀 배정(0727-1)과 조향 PD 방향은 이미 실차로 정상 확인된 상태이므로, 남은 원인은
//      차동 계산의 부호뿐. STEER_DIFF_SIGN을 -1 -> +1로 뒤집어 해결.
//
//  [0727-2] 차동 게인 0.15 -> 0.05 (가산식은 유지, 게인만 조정)
//      기존 0.15는 40도에서 ±6펄스로, 애커만 기하가 요구하는 값보다 3~6배 과했음.
//      애커만 : v_바깥/v_안쪽 = 1 ± W·tan(d)/(2L)  (W=윤거, L=축거, d=조향각)
//      가산 근사 : gain[pulse/deg] ~= v_typ x (W/2L) x tan(40도)/40도
//                              = 6 x 0.33 x 0.021 ~= 0.04
//      -> 0.05 채택 (40도에서 ±2펄스, 10도부터 ±1펄스)
//      실차 윤거/축거 실측 후 위 식으로 재계산 권장.
//
//  [0727-3] A1 가변저항 필터를 '9샘플 중앙값 + 지수평활' 2단으로 교체
//      기존 : 지수평활(α=0.3) 단독 -> 스파이크성 노이즈는 평균에 섞여 그대로 남음
//      변경 : 제어주기마다 analogRead를 9회 연속 수행해 중앙값 채택(스파이크 완전 제거,
//             소요 ~1ms) -> 그 값을 기존 지수평활에 통과(잔떨림 억제)
//
//  [0727-4] 텔레메트리 출력을 기존 "S,<펄스>,<펄스>" 2필드 형식으로 복귀
//      (직전 개발본에서 11필드 공백구분으로 바뀌어 있었으나 ROS 파서 규약 유지를 위해 원복)
//
//  [0727-5] 입출력 좌/우 순서 통일 : 입력도 출력도 항상 "왼쪽 먼저, 오른쪽 나중"
//      (0727-1의 라벨 교정에 따라 첫 필드가 실제 왼쪽 모터를 가리키게 됨)
//
//  --- 이하 구조는 0717과 동일 ---
//  입력 : "<값>" 또는 "<왼쪽값>,<오른쪽값>"  (부호 없는 정수, 개행 종료)
//         - 단일 값: 펄스 전용(0~15), 범위 밖/숫자 아님은 무시
//         - 콤마 2값: 좌/우 독립 — 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지
//  출력 : "S,<왼쪽펄스>,<오른쪽펄스>" (평상시) / "STOP" (e-stop 중)
//  제어주기 : 20ms, 출력주기 : 50ms
//    - 왼쪽 : 펄스 21 → PID → PWM 8 / 오른쪽 : 펄스 20 → PID → PWM 9
//    - FF 보간 테이블(펄스->PWM, 라그랑주 2차보간) + PID(0.4/0.03/0.2, 좌우 게인 별도 관리)
//    - PWM 슬루레이트 제한(+4/cycle) + 조건부 적분(|오차|<4, 기여 ±40 클램프)
//    - 하강 코스트-캐치, 폭주 감지(목표+2펄스 과속 1초 연속 시 코스트)
//      ※ 위 안전장치들은 '펄스 모드'에만 적용. 직접 PWM 모드는 무보호(테스트 전용).
//  E-stop 스위치: 13번 핀, NC(Normally Closed) 방식, B보드와 병렬 감지
//    - 평상시 GND와 단락(LOW), 버튼 누름/단선 시 개방(HIGH) → e-stop
//    - e-stop 발동 시 직접 PWM 모드도 즉시 해제(펄스 0으로 복귀)
//  E-stop 조건 (매 루프 재평가) : 13번 핀 500ms 연속 개방(HIGH) (외부 개입만, 타임아웃 없음)
//  E-stop 동작 : 좌우 인휠 PWM 0, 양쪽 PID 상태 리셋, "STOP" 출력
// ============================================================


// ================= 핀 정의 (여기서 조절) =================
// ★ 실측 확정 배선 (0727) ★
//     왼쪽  모터 : 펄스 21 -> PID -> PWM 8
//     오른쪽 모터 : 펄스 20 -> PID -> PWM 9
//   즉 좌/우 각각 '자기 바퀴의 펄스'로 '자기 바퀴의 PWM'을 닫는다 (교차 없음).
//   ※ 좌우가 뒤바뀐 것으로 보이면 이 4줄만 고치면 된다.
//     아래 파서/PID/차동/출력은 전부 LEFT/RIGHT 인덱스로만 동작하므로 자동으로 따라간다.

// --- 홀센서 (인터럽트 핀, XOR 합산신호) ---
const uint8_t HALL_PIN_L = 21;   // 왼쪽 모터컨트롤러 펄스 (왼쪽 PID 피드백)
const uint8_t HALL_PIN_R = 20;   // 오른쪽 모터컨트롤러 펄스 (오른쪽 PID 피드백)

// --- 인휠 주행 PWM ---
const uint8_t PWM_PIN_L = 8;     // 왼쪽 모터 PWM (펄스 21과 같은 컨트롤러, 0727 재확인)
const uint8_t PWM_PIN_R = 9;     // 오른쪽 모터 PWM (펄스 20과 같은 컨트롤러, 0727 재확인)

// --- E-stop (NC: 평상시 LOW, 개방 시 HIGH → e-stop) ---
const uint8_t ESTOP_PIN = 13;
const bool ESTOP_ENABLED = true;   // false로 두면 핀 e-stop 비활성

// --- 조향각 가변저항 (B보드 DC_POT_PIN과 동일 신호선을 분기, A1으로 병렬 입력) ---
const uint8_t DC_POT_PIN = A1;


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= ★ 좌우 차동 보정 (조향각 기반, 튜닝 지점) ★ =================
// B보드(kasa_0727_B.ino)와 동일한 raw <-> 각도 매핑을 재사용해야 두 보드가
// "같은 각도"를 같은 의미로 해석함. B보드 상수값 변경 시 여기도 반드시 동기화할 것.
const bool ENABLE_STEER_DIFF = true;   // false로 두면 차동 보정 없이 좌우 target 그대로 사용(디버그용)

const int RAW_LEFT_LIMIT  = 1001;   // B보드와 동일 (왼쪽 끝 하드 리밋 raw)
const int RAW_RIGHT_LIMIT = 751;    // B보드와 동일 (오른쪽 끝 하드 리밋 raw)
const int STEER_SAFETY_MARGIN = 10; // B보드와 동일

const int STEER_ANGLE_MAX = 40;     // B보드와 동일
const int STEER_ANGLE_MIN = -STEER_ANGLE_MAX;

const int POT_AT_ANGLE_MIN = RAW_LEFT_LIMIT  - STEER_SAFETY_MARGIN;   // 각도 -40(왼쪽) -> 이 raw값
const int POT_AT_ANGLE_MAX = RAW_RIGHT_LIMIT + STEER_SAFETY_MARGIN;   // 각도 +40(오른쪽) -> 이 raw값

// ★ [0727-6] 차동 방향 부호 (여기 하나만 뒤집으면 좌우 차동 방향이 통째로 반전됨)
//   실차 텔레메트리로 재확인한 결과(핀/방향/PID는 이미 정상 확인됨), 차동 부호만 반대였음:
//     오른쪽 30도 조향 시 관측 "S,<좌>,<우>" 예: 13,16 / 13,14 / 12,15 -> 왼쪽이 계속 더 느림
//       (물리적으로는 오른쪽 조향 시 왼쪽이 바깥쪽이라 더 빨라야 함 -> 반대로 나타남)
//     왼쪽  -30도 조향 시 관측 예: 15,13 / 16,13 / 17,13 -> 왼쪽이 계속 더 빠름
//       (물리적으로는 왼쪽 조향 시 왼쪽이 안쪽이라 더 느려야 함 -> 반대로 나타남)
//   두 경우 모두 부호가 정반대로 걸려 있었으므로 이 상수만 뒤집으면 해결됨 (-1 -> +1).
//   +1 : 오른쪽 조향(양수각) -> 왼쪽(바깥) 증속 / 오른쪽(안쪽) 감속  (물리적으로 올바른 방향)
//   -1 : 그 반대 (구버전, 위 증상의 원인)
const int STEER_DIFF_SIGN = 1;

// ★ [0727-2] 조향각 1도당 좌/우에 가감할 펄스량 (헤더의 애커만 근사식 참고)
//   0.05 -> 40도에서 ±2펄스, 10도부터 ±1펄스.
//   윤거(W)/축거(L) 실측 후 gain = v_typ x (W/2L) x tan(40도)/40도 로 재계산 권장.
float diffGainPulsePerDeg = 0.05;

// [0727 추가] 조향 0점 오프셋 보정 (단위: 도, 기본 0 = 동작 변화 없음)
//   매핑 중앙 raw는 (POT_AT_ANGLE_MIN + POT_AT_ANGLE_MAX)/2 = 876 인데,
//   기계적 직진 위치의 실측 raw가 이와 다르면 직진 중에도 각도가 0이 아니게 읽힌다.
//   (예: 실측 중앙 886 -> potToAngle(886) = -4도)
//   [설정법] 핸들을 기계적 직진에 두고 B보드 "P,<각도>" 출력값을 그대로 여기에 넣는다.
//   ※ 현재 게인 0.05에서는 4도 정도의 치우침은 round()에서 0으로 흡수되어 무해하므로
//     기본값 0으로 둠. 게인을 0.1 이상으로 올릴 경우 반드시 실측해서 채울 것.
const int STEER_ZERO_OFFSET_DEG = 0;

// 직진(0도) 근처 데드밴드. 가변저항 노이즈로 각도가 0 근처에서 미세하게 흔들리면
// diffOffset이 0<->1 사이를 반복 플리커링해서 PID 목표가 계속 흔들리고
// (FF테이블 0->1펄스 구간이 가파름) 모터가 살짝살짝 움직이려는 증상이 생김.
// 이 각도 이내는 무조건 직진(오프셋 0)으로 강제 고정해 방지.
const int STEER_DEADBAND_DEG = 3;

// ================= [0727-3] 가변저항 필터 : 9샘플 중앙값 + 지수평활 =================
// 1단(중앙값) : 제어주기마다 analogRead를 POT_MEDIAN_N회 연속 수행해 중앙값만 채택.
//   ADC 스파이크(모터 노이즈 유입 등)를 '평균에 섞지 않고' 통째로 버리는 것이 목적.
//   Mega의 analogRead는 약 104us이므로 9회 ≈ 0.94ms (제어주기 20ms의 5% 수준).
// 2단(지수평활) : 중앙값 출력에 남는 소진폭 잔떨림을 억제. 기존 α=0.3 그대로 유지.
const uint8_t POT_MEDIAN_N = 9;   // 반드시 홀수
const float STEER_ADC_SMOOTH_ALPHA = 0.3;
float steerAdcFiltered = -1;   // -1 = 아직 초기화 안 됨


// ================= 공통 제어주기 =================
const unsigned long CONTROL_WINDOW_MS = 20;


// ================= ★ 인휠 FF 보간 테이블 (펄스 -> PWM, 실측으로 조절, 좌우 공통) ★ =================
const int FF_TABLE_N = 12;
const float ffPulseTable[FF_TABLE_N] = { 1.00,  2.00,  3.00,  4.00,  5.00,  6.50,  8.00, 10.09, 13.05, 16.05, 20.45, 24.00};
const float ffPwmTable[FF_TABLE_N]   = {60,    70,    80,    90,    100,   110,   120,   130,   140,   150,   160,   170};

// ================= ★ 인휠 PID 게인 (튜닝 지점, 좌[0]/우[1] 별도 관리) ★ =================
float kp[2] = {0.4,  0.4};    // {왼쪽(펄스21-PWM8), 오른쪽(펄스20-PWM9)}
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
const int PWM_SLEW_MAX = 4;

// 적분 누적을 오차가 작을 때(목표 근접 시)만 허용 - 큰 오차 구간(가속 중)에서의 와인드업 방지
const int I_ACCUM_ERR_MAX = 4;

// ================= ★ 폭주 감지 (튜닝 지점) ★ =================
// 한쪽 컨트롤러의 과속 특성 대비 안전망: 목표보다 RUNAWAY_ERR_OVER 펄스 이상 과속이
// RUNAWAY_CONFIRM_CYCLES 주기(20ms) 연속되면 해당 바퀴만 PWM 0(코스트) → 캐치로 재개.
const int RUNAWAY_ERR_OVER = 2;
const int RUNAWAY_CONFIRM_CYCLES = 50;   // 50주기 = 1초


// ================= 좌/우 PID 상태 (완전 분리) =================
// 주의: Arduino IDE는 함수 프로토타입을 파일 맨 위(커스텀 타입 정의보다 앞)에 자동 삽입한다.
// struct로 상태를 묶으면 그 프로토타입이 struct 정의보다 앞에 삽입되어 컴파일 에러가 남.
// 그래서 기본 타입(int/float/bool) 배열 + 좌(0)/우(1) 인덱스로 상태를 분리한다.
const uint8_t LEFT  = 0;   // 왼쪽  : 펄스 21 피드백 -> PWM 8
const uint8_t RIGHT = 1;   // 오른쪽 : 펄스 20 피드백 -> PWM 9

float pidI[2]        = {0, 0};
int   pidLastErr[2]  = {0, 0};
bool  pidCoasting[2] = {false, false};
int   pidLastPwm[2]  = {0, 0};
int   runawayCnt[2]  = {0, 0};   // 폭주 판정용 연속 과속 주기 카운터


// ================= 구동 모드 (좌/우 독립) =================
const uint8_t DRIVE_PULSE = 0;   // 펄스 목표 PID 주행 (0~15)
const uint8_t DRIVE_PWM   = 1;   // 직접 PWM 출력 (16~255, 무보호)

uint8_t driveMode[2] = {DRIVE_PULSE, DRIVE_PULSE};
int     pwmDirect[2] = {0, 0};   // 직접 PWM 모드에서 출력할 값


// ================= 인휠 상태 =================
volatile long encCountL = 0;    // 왼쪽 펄스 (21번 핀)
volatile long encCountR = 0;    // 오른쪽 펄스 (20번 핀)
int target[2] = {0, 0};         // 좌/우 목표펄스 (0~TARGET_MAX, 펄스 모드용)
int wheelSpeedL = 0;
int wheelSpeedR = 0;
unsigned long wheel_t = 0;

// A1으로 읽은 현재 조향각(도)과, 그로부터 계산된 좌/우 유효 목표펄스 (디버깅용 보관)
int steerAngleDeg = 0;
int effTargetL = 0;
int effTargetR = 0;


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


// ================= ISR (21번=왼쪽 펄스, 20번=오른쪽 펄스) =================
void encISR_L() { encCountL++; }
void encISR_R() { encCountR++; }


// ================= 조향각 raw -> 각도 변환 (B보드 potToAngle과 동일 로직) =================
int potToAngle(int raw) {
  int angle = map(raw, POT_AT_ANGLE_MIN, POT_AT_ANGLE_MAX, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
  return constrain(angle, STEER_ANGLE_MIN, STEER_ANGLE_MAX);
}


// ================= [0727-3] 1단 필터 : 9샘플 중앙값 =================
// 읽으면서 바로 삽입정렬로 정렬해두고 가운데 값을 반환 (9개 기준 약 20회 비교, 부하 무시 가능).
// analogRead 9회 ≈ 0.94ms 소요. 제어주기(20ms) 안에서만 호출할 것.
int readPotMedian() {
  int s[POT_MEDIAN_N];
  for (uint8_t i = 0; i < POT_MEDIAN_N; i++) {
    int v = analogRead(DC_POT_PIN);
    uint8_t j = i;
    while (j > 0 && s[j - 1] > v) {
      s[j] = s[j - 1];
      j--;
    }
    s[j] = v;
  }
  return s[POT_MEDIAN_N / 2];
}


// ================= [0727-3] 2단 필터 + 각도 환산 =================
// 중앙값(스파이크 제거) -> 지수평활(잔떨림 억제) -> 각도
int readSteerAngleFiltered() {
  int med = readPotMedian();
  if (steerAdcFiltered < 0) {
    steerAdcFiltered = med;
  } else {
    steerAdcFiltered += STEER_ADC_SMOOTH_ALPHA * ((float)med - steerAdcFiltered);
  }
  return potToAngle((int)steerAdcFiltered);
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
  }
  effTargetL = 0;
  effTargetR = 0;
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

  // 조향각 가변저항 (B보드와 신호선 공유, analogRead는 pinMode 없어도 되지만 명시)
  pinMode(DC_POT_PIN, INPUT);

  // [0727-3] 필터 초기화: 첫 제어주기부터 안정된 값으로 시작하도록 중앙값으로 프라이밍
  steerAdcFiltered = readPotMedian();
  steerAngleDeg = potToAngle((int)steerAdcFiltered);

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
void setPulseTarget(uint8_t idx, int newTarget) {
  if (driveMode[idx] == DRIVE_PWM) {
    driveMode[idx] = DRIVE_PULSE;
    pwmDirect[idx] = 0;
    pidI[idx] = 0;
    pidLastErr[idx] = 0;
    pidLastPwm[idx] = 0;      // 슬루 기준점 리셋: 캐치 후 0부터 다시 램프업
    runawayCnt[idx] = 0;
    pidCoasting[idx] = true;
    target[idx] = newTarget;
    return;
  }
  // 목표 하강 → 코스트 진입(무동력 감속), 상승 → 코스트 해제 (좌우 개별)
  if (newTarget < target[idx]) {
    pidCoasting[idx] = true;
    pidI[idx] = 0;
  } else if (newTarget > target[idx]) {
    pidCoasting[idx] = false;
  }
  target[idx] = newTarget;
}


// ================= 값 -> 한쪽 바퀴 적용 (콤마 2값 형식 전용) =================
// 0~15 펄스 / 16~255 직접 PWM / 256 이상 정지(펄스 0)
void applySide(uint8_t idx, long v) {
  if (v <= TARGET_MAX) {
    setPulseTarget(idx, (int)v);
  } else if (v <= PWM_DIRECT_MAX) {
    driveMode[idx] = DRIVE_PWM;
    pwmDirect[idx] = (int)v;
  } else {
    setPulseTarget(idx, 0);
  }
}


// ================= 입력 파서 =================
// "<값>" 단일 = 펄스 전용(0~15, 좌우 공통), "<좌값>,<우값>" = 좌/우 독립(펄스/PWM/정지).
// 형식이 안 맞으면 무시.
void handleLine(char* line) {
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
    applySide(LEFT, vL);
    applySide(RIGHT, vR);
  } else {
    // 단일 값은 무조건 펄스 모드: 0~15만 받고 그 외는 무시
    // (직접 PWM은 콤마 2값 형식으로만 가능 — 일반 주행 경로에서의 오발동 방지)
    long v = parseValue(line);
    if (v < 0 || v > TARGET_MAX) return;

    if (estop_active) return;
    setPulseTarget(LEFT, (int)v);
    setPulseTarget(RIGHT, (int)v);
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
  wheelSpeedL = (int)cL;
  wheelSpeedR = (int)cR;

  // ── A1 조향각 실측 -> 좌/우 차동 오프셋 계산 ──
  // 회전 안쪽 바퀴는 느리게, 바깥쪽 바퀴는 빠르게. (디퍼런셜이 없으므로 두 모터로 흉내)
  // target[] 자체는 건드리지 않고(코스트/PID 상태 로직 보존), PID에 넘기는 목표값에만 반영.
  //
  // [0727-6] STEER_DIFF_SIGN으로 방향을 통제한다. 부호 의미는 상수 정의부 주석 참조.
  // [0727-2] 게인 0.05 (40도에서 ±2펄스)
  //
  // ★ 중요: 좌/우 base target이 둘 다 0(정지 명령)이면 조향 오프셋도 무시하고 0 유지.
  // 안 그러면 정지 상태에서 조향각(A1 미배선/노이즈 포함)만으로 한쪽 바퀴가
  // constrain 클램프 비대칭 때문에 혼자 돌아버리는 오발진이 생김.
  steerAngleDeg = readSteerAngleFiltered();
  int effAngle = steerAngleDeg - STEER_ZERO_OFFSET_DEG;   // 0점 보정 후 각도

  int diffOffset = 0;
  if (ENABLE_STEER_DIFF && abs(effAngle) > STEER_DEADBAND_DEG) {
    diffOffset = STEER_DIFF_SIGN * (int)round(diffGainPulsePerDeg * (float)effAngle);
  }
  if (target[LEFT] == 0 && target[RIGHT] == 0) {
    effTargetL = 0;
    effTargetR = 0;
  } else {
    effTargetL = constrain(target[LEFT]  + diffOffset, 0, TARGET_MAX);
    effTargetR = constrain(target[RIGHT] - diffOffset, 0, TARGET_MAX);
  }

  // 직접 PWM 모드면 받은 값을 그대로 출력 (상한/슬루/PID 미적용),
  // 펄스 모드면 기존 PID. 펄스 계측은 두 모드 공통으로 계속된다.
  int pwmL = (driveMode[LEFT] == DRIVE_PWM)
               ? pwmDirect[LEFT]
               : updatePid(LEFT,  effTargetL, wheelSpeedL);   // 왼쪽 : 펄스 21 -> PWM 8
  int pwmR = (driveMode[RIGHT] == DRIVE_PWM)
               ? pwmDirect[RIGHT]
               : updatePid(RIGHT, effTargetR, wheelSpeedR);   // 오른쪽 : 펄스 20 -> PWM 9

  analogWrite(PWM_PIN_L, pwmL);
  analogWrite(PWM_PIN_R, pwmR);
}


// ================= 출력 =================
// [0727-4/5] 기존 2필드 규약 유지: "S,<왼쪽펄스>,<오른쪽펄스>"
//   입력 "<왼쪽>,<오른쪽>"과 순서가 동일하다 (왼쪽이 앞, 오른쪽이 뒤).
//   직접 PWM 모드에서도 펄스 계측/출력은 계속됨 (PWM-펄스 특성 계측용)
void sendOutput(unsigned long now) {
  if (now - tele_t < TELE_MS) return;
  tele_t = now;

  if (estop_active) {
    Serial.println("STOP");
    return;
  }

  Serial.print("S,");
  Serial.print(wheelSpeedL);
  Serial.print(',');
  Serial.println(wheelSpeedR);
}


// ================= loop =================
void loop() {
  unsigned long now = millis();
  pollSerial();

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
