// ============================================================
//  linear_0803_speed.ino : 리니어(브레이크) '구동시간 -> 이동량' 실측표 만들기
//                          (Arduino Mega 2560, 자동 시퀀스)
//
//  ★★ 왜 이 코드인가 (linear_0804.ino 실패 로그에서 나온 결론) ★★
//    linear_0804.ino는 엔코더를 매 루프 보며 '목표에 닿는 순간' PWM을 끊는 폐루프였다.
//    그런데 실측 로그가 이렇게 나왔다:
//        # MOVE L1 rel=150 cur=0 err=150
//        ... 12 20 29 37 48 ... 197 202 206 209 211 213 215 216 217 217
//        # DONE L1 OVER 73 -> ZERO
//    목표 150인데 217에서 멈췄다(오버슛 73 = 목표의 49%). 상승 구간이 5ms 로그로 약 45줄
//    (≈225ms)에 217카운트니 ★대략 1카운트/ms★ 로 움직인다. 이 속도에서는
//      - 5ms 로그 한 줄 사이에 5카운트가 지나가고,
//      - PWM을 끊어도 관성으로 수십 카운트가 더 밀린다.
//    즉 '판정을 더 자주 하기'로는 해결되지 않는 문제다(POS_TOLERANCE를 0으로 해도 남는다).
//    PD/PID로 감속하는 방법도 있지만 브레이크는 반응성이 최우선이고 러프해도 되므로,
//    ★kasa_0731_B.ino가 이미 쓰는 방식 — 정해진 시간만큼 PWM 255로 미는 열린루프 —★
//    로 가는 것이 맞다. 그때 필요한 것이 딱 하나, "몇 ms 밀면 몇 카운트 가는가" 표다.
//    ★ 이 코드가 그 표를 만든다 ★ 결과는 곧 0731_B의 BRAKE_*_MS 값이 된다.
//
//  ★ 시퀀스 (자동, 입력 없이 진행) ★
//    한 사이클 = [복귀 1초] -> [정착 -> 영점] -> [밀기 t ms] -> [정착 -> 측정] -> [쉬기 1초]
//      1) HOME  : 들어가는 방향(REV)으로 HOME_MS(1000ms) — 하드스톱까지 밀어 기준 회복
//      2) 정착  : PWM 0으로 SETTLE_MS(200ms) 기다린 뒤 ★그 자리를 영점(0)으로★
//      3) PUSH  : 나오는 방향(FWD)으로 t ms (t = 50, 100, 150, ... STEP_MS씩 증가)
//      4) 정착  : PWM 0으로 SETTLE_MS 기다린 뒤 엔코더를 읽어 기록
//                 (끊은 순간의 값도 함께 남겨 '관성으로 더 밀린 양 = coast'을 뽑는다)
//      5) REST  : REST_MS(1000ms) 쉼 -> t를 STEP_MS 늘려 1)로
//    ★ 측정값이 TARGET_COUNT(300)에 도달하면 그 사이클을 끝으로 종료 ★
//    -> 지금까지의 표를 한 번에 출력하고 아무것도 하지 않는다(ST_DONE).
//
//  ★ 출력 ★
//    진행 중에는 사이클마다 한 줄만:
//        # STEP 3 : 150ms -> stop 141  settled 158  coast 17  err 0
//    끝나면 탭 구분 표 (엑셀/시트에 그대로 붙여넣기 좋게 '#' 없이):
//        ms	stop	settled	coast	err
//        50	38	45	7	0
//        ...
//    이어서 선형 보간 추정치를 낸다 — ★이것이 실제로 쓸 값이다★:
//        # 추정 : 150카운트 = 165ms / 300카운트 = 320ms
//    ※ STREAM_ENABLED를 true로 두면 구동 중 카운트를 5ms마다 흘려 궤적도 볼 수 있다
//      (기본 false. 표만 볼 때는 로그가 깨끗한 편이 낫다).
//
//  ★ 입력 (선택, 115200, ★개행 전송★) ★
//      s = 즉시 중단 (PWM 0, 지금까지의 표 출력)
//      r = 처음부터 다시 (표를 비우고 1단계부터)
//    ※ 그 외 입력은 무시. 시퀀스 자체는 켜면 바로 시작한다.
//
//  ★ 배선 (linear_0804.ino와 동일) ★
//    - 리니어 MD20A : DIR = D8, PWM = D9      리니어 MB - 빨간색, MA - 검은색
//        DIR = LOW(FWD)  = 로드가 나옴 = 브레이크 밟는 방향  (카운트 증가 방향)
//        DIR = HIGH(REV) = 로드가 들어감 = 브레이크 놓는 방향
//    - 엔코더 E40S8-2048-3-T-5 : A -> D2(INT4), B -> D3(INT5), C(Z) 미사용, 5V/GND
//        x4 디코딩 -> 1회전 8192카운트, 1카운트 = 0.044도
//    ※ 0804 실측 로그에서 FWD 구동 시 카운트가 +로 증가함을 확인했으므로 INVERT_DIR=false다.
//      (뒤집혀 있으면 표의 부호가 전부 음수로 나와 종료조건에 영원히 못 닿는다)
//
//  ★★ 안전 ★★
//    - HOME_MS(1초)는 하드스톱에 PWM 255로 밀어붙이는 ★스톨 시간★이다. 실제 복귀는 0.3초
//      정도면 끝나므로 나머지는 스톨이다. 필요 이상 늘리지 말 것(드라이버·모터 발열).
//    - PUSH 중 카운트가 ABORT_COUNT(400)를 넘으면 그 즉시 정지하고 테스트를 끝낸다.
//      전 행정이 약 400카운트(linear_0803_pot.ino 실측)이므로 그 이상은 하드스톱을 밀는 것이다.
//    - MAX_STEPS(24 = 1.2초)까지만 시도한다. 엔코더가 죽어 300에 못 닿아도 여기서 멈춘다.
//    - E-stop/리미트 스위치는 없다(단독 실측 코드). 손은 기구에서 떼고 돌릴 것.
//
//  동작 : delay() 미사용, millis() 기준 논블로킹 상태머신 (카운트는 인터럽트로만 갱신)
// ============================================================


// ================= 핀 정의 =================
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;

// 엔코더는 D2/D3 고정 (readAB()의 포트 읽기와 ISR 벡터 이름이 이 두 핀 전용)
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;

// 정방향(체결, 나옴) = LOW / 역방향(해제, 들어감) = HIGH
#define LINEAR_FWD  LOW
#define LINEAR_REV  HIGH


// ================= 시퀀스 파라미터 (여기서 조절) =================
const int DRIVE_PWM = 255;              // 항상 이 값 (요구사항)

const unsigned long STEP_MS = 50;       // 밀기 시간 증가폭 (0.05초)
const uint8_t       MAX_STEPS = 24;     // 최대 단계 수 (24 x 50ms = 1.2초까지)

const unsigned long HOME_MS   = 1000;   // 매 사이클 시작 시 복귀(REV) 시간 ★스톨 주의★
const unsigned long SETTLE_MS = 200;    // 정지 후 관성이 멈추기를 기다리는 시간
const unsigned long REST_MS   = 1000;   // 측정 후 쉬는 시간

const long TARGET_COUNT = 300;          // ★이 값에 도달하면 테스트 종료★
const long MID_COUNT    = TARGET_COUNT / 2;   // 표 아래 추정치를 함께 뽑을 중간값(150)

// PUSH 중 이 카운트를 넘으면 즉시 중단 (전 행정 약 400 — 그 이상은 하드스톱을 미는 것)
const long ABORT_COUNT = 400;

// 첫 사이클 측정값이 이 크기 이상 '음수'면 엔코더 방향이 반대라고 보고 중단한다
const long DIR_CHECK_COUNT = 5;

// 구동 중 카운트를 5ms마다 흘려볼지 (기본 false = 표만 깨끗하게)
const bool STREAM_ENABLED = false;
const unsigned long STREAM_MS = 5;

const unsigned long BAUD = 115200;


// ================= 엔코더 옵션 =================
const long PPR            = 2048;
const long COUNTS_PER_REV = PPR * 4;   // 8192 (참고용)

const bool USE_PULLUP = false;         // 토템폴(T) 출력이라 INPUT이 맞다
const bool INVERT_DIR = false;         // FWD가 음수로 세어지면 true (0804 로그로 false 확인)
const bool RECOVER_LOST_EDGES = true;  // 유실 엣지 ±2 보정 (linear_0803_pot.ino와 동일)


// ================= x4 쿼드러처 전이 표 =================
// 상태 = (A<<1)|B, 인덱스 = (이전<<2)|현재. +1/-1 = 정상 전이, 0 = 무변화 또는 동시변화(유실).
const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};


// ================= ISR이 갱신하는 값 =================
volatile long     enc_count = 0;
volatile uint8_t  enc_state = 0;
volatile uint16_t enc_err   = 0;
volatile int8_t   enc_dir   = 0;


// ================= 상태머신 =================
//   ST_HOME        : REV 구동 (하드스톱까지 복귀)
//   ST_HOME_SETTLE : 정지 -> 정착 후 ★영점★
//   ST_PUSH        : FWD 구동 (이번 단계의 시간만큼)
//   ST_HOLD        : 정지 -> 정착 후 ★측정·기록★
//   ST_REST        : 쉼 -> 다음 단계
//   ST_DONE        : 종료 (무출력 유지)
enum SeqState { ST_HOME, ST_HOME_SETTLE, ST_PUSH, ST_HOLD, ST_REST, ST_DONE };
SeqState state = ST_HOME;

uint8_t  step_idx = 0;             // 지금 단계 (0 -> 50ms, 1 -> 100ms, ...)
unsigned long phase_t = 0;         // 지금 상태에 들어온 시각
long     push_stop_count = 0;      // PWM을 끊은 순간의 카운트

// ================= 결과 표 =================
long     tbl_stop[MAX_STEPS];      // 끊은 순간의 카운트
long     tbl_settled[MAX_STEPS];   // 정착 후 카운트 ★이것이 '그 시간의 이동량'★
uint16_t tbl_err[MAX_STEPS];       // 그 사이클에서 관측된 엣지 유실 수
uint8_t  n_done = 0;               // 기록된 단계 수

unsigned long stream_t = 0;


// ================= 함수 선언 =================
static inline uint8_t readAB();
long encRead();
uint16_t errRead();
void zeroHere();
void linearStop();
void linearDrive(uint8_t dir, int p);
void enterState(SeqState s, unsigned long now);
long pushMsOf(uint8_t i);
void recordStep(unsigned long now);
void printTable();
long estimateMs(long target);
void abortAll(unsigned long now, const char* why);
void restartAll(unsigned long now);
void updateSeq(unsigned long now);
void streamOut(unsigned long now);
void pollSerial(unsigned long now);


// ================= A/B 레벨 읽기 =================
// ★ Mega 2560 : D2 = PE4, D3 = PE5 ★ 포트를 한 번에 읽어 A/B가 어긋나지 않게 한다.
static inline uint8_t readAB() {
  uint8_t p = PINE;
  uint8_t a = (p >> 4) & 0x01;   // PE4 = D2
  uint8_t b = (p >> 5) & 0x01;   // PE5 = D3
  return (uint8_t)((a << 1) | b);
}


// ================= 엔코더 1엣지 처리 =================
static inline void encStep() __attribute__((always_inline));
static inline void encStep() {
  uint8_t cur  = readAB();
  uint8_t prev = enc_state;
  if (cur == prev) return;                    // 같은 상태로의 인터럽트(글리치)
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


// ================= ISR 벡터 직결 (INT4=D2, INT5=D3) =================
// attachInterrupt(5~6us) 대신 벡터 직결(2~3us)로 누락 한계를 약 2배 올린다.
// 엣지 발생률 = 8192 x (RPM/60) = 약 136.5 x RPM.
ISR(INT4_vect) { encStep(); }
ISR(INT5_vect) { encStep(); }


// ================= 원자적 읽기 (4바이트는 원자적이지 않다) =================
long encRead() {
  long c;
  noInterrupts();
  c = enc_count;
  interrupts();
  return c;
}

uint16_t errRead() {
  uint16_t e;
  noInterrupts();
  e = enc_err;
  interrupts();
  return e;
}


// ================= 영점 =================
// ★ enc_state는 건드리지 않는다 ★ 진행 중인 쿼드러처 전이의 판정 기준이라, 다시 읽어
//   넣으면 마침 대기 중이던 엣지를 한 카운트 잃는다. 기준점만 옮기면 된다.
void zeroHere() {
  noInterrupts();
  enc_count = 0;
  enc_err   = 0;   // 이 사이클에서 생긴 유실만 세도록 함께 초기화
  enc_dir   = 0;
  interrupts();
}


// ================= 모터 출력 =================
void linearStop() { analogWrite(LINEAR_PWM_PIN, 0); }

// ★ DIR을 먼저 쓰고 PWM을 나중에 ★ 순서가 반대면 전환 순간 잘못된 방향으로 힘이 들어간다.
void linearDrive(uint8_t dir, int p) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, 255));
}


// ================= 상태 전환 =================
// 각 상태 진입 시 '그 상태에서 내보낼 출력'을 여기서 한 번만 정한다. PWM이 상수라
// 주기적으로 다시 인가할 것이 없다(그래서 제어주기가 없다 — linear_0804.ino 헤더 참고).
void enterState(SeqState s, unsigned long now) {
  state   = s;
  phase_t = now;

  switch (s) {
    case ST_HOME:        linearDrive(LINEAR_REV, DRIVE_PWM); break;
    case ST_PUSH:        linearDrive(LINEAR_FWD, DRIVE_PWM); break;
    case ST_HOME_SETTLE:
    case ST_HOLD:
    case ST_REST:
    case ST_DONE:        linearStop(); break;
  }
}


// ================= 단계 i의 밀기 시간 =================
long pushMsOf(uint8_t i) { return (long)STEP_MS * (long)(i + 1); }


// ================= 측정·기록 (ST_HOLD 정착 후 한 번) =================
void recordStep(unsigned long now) {
  long settled = encRead();
  uint16_t e   = errRead();

  if (n_done < MAX_STEPS) {
    tbl_stop[n_done]    = push_stop_count;
    tbl_settled[n_done] = settled;
    tbl_err[n_done]     = e;
    n_done++;
  }

  Serial.print("# STEP ");
  Serial.print(step_idx + 1);
  Serial.print(" : ");
  Serial.print(pushMsOf(step_idx));
  Serial.print("ms -> stop ");
  Serial.print(push_stop_count);
  Serial.print("  settled ");
  Serial.print(settled);
  Serial.print("  coast ");
  Serial.print(settled - push_stop_count);   // 끊은 뒤 관성으로 더 밀린 양
  Serial.print("  err ");
  Serial.println(e);
}


// ================= 선형 보간 : 목표 카운트에 필요한 시간 추정 =================
// 표의 이웃한 두 점 사이를 직선으로 잇는다. 첫 점에서 이미 목표를 넘었다면 원점(0ms,0카운트)
// 과 첫 점을 이어 추정한다. 아직 목표에 못 닿았으면 -1(추정 불가).
long estimateMs(long target) {
  if (n_done == 0) return -1;

  if (tbl_settled[0] >= target) {
    if (tbl_settled[0] <= 0) return -1;
    return target * pushMsOf(0) / tbl_settled[0];
  }

  for (uint8_t i = 1; i < n_done; i++) {
    if (tbl_settled[i] >= target) {
      long d = tbl_settled[i] - tbl_settled[i - 1];
      if (d <= 0) return -1;   // 진행이 없었다(스톨/노이즈) -> 추정 포기
      return pushMsOf(i - 1)
           + (target - tbl_settled[i - 1]) * (pushMsOf(i) - pushMsOf(i - 1)) / d;
    }
  }
  return -1;
}


// ================= 표 출력 =================
// 표 본문은 '#' 없이 탭 구분으로 낸다 — 시리얼 모니터에서 그대로 복사해 시트에 붙일 수 있다.
void printTable() {
  Serial.println();
  Serial.println("# ===== 결과표 : 구동시간 -> 이동량 (PWM 255) =====");
  Serial.println("# stop=끊은순간, settled=정착후(이 값을 쓴다), coast=관성분, err=엣지유실");
  Serial.println("ms\tstop\tsettled\tcoast\terr");

  for (uint8_t i = 0; i < n_done; i++) {
    Serial.print(pushMsOf(i));
    Serial.print('\t');
    Serial.print(tbl_stop[i]);
    Serial.print('\t');
    Serial.print(tbl_settled[i]);
    Serial.print('\t');
    Serial.print(tbl_settled[i] - tbl_stop[i]);
    Serial.print('\t');
    Serial.println(tbl_err[i]);
  }

  // ★ 실제로 쓸 값 ★ 이 두 숫자가 kasa_0731_B.ino의 BRAKE_*_MS 후보가 된다.
  long t_mid  = estimateMs(MID_COUNT);
  long t_full = estimateMs(TARGET_COUNT);

  Serial.print("# 추정 : ");
  Serial.print(MID_COUNT);
  Serial.print("카운트 = ");
  if (t_mid < 0) Serial.print("범위밖"); else { Serial.print(t_mid); Serial.print("ms"); }
  Serial.print(" / ");
  Serial.print(TARGET_COUNT);
  Serial.print("카운트 = ");
  if (t_full < 0) Serial.print("범위밖"); else { Serial.print(t_full); Serial.print("ms"); }
  Serial.println("  (선형 보간)");

  // 평균 속도도 함께 — 표를 눈으로 볼 때 기준이 된다(0804 로그에서는 약 1카운트/ms였다)
  if (n_done > 0 && pushMsOf(n_done - 1) > 0) {
    Serial.print("# 마지막 단계 평균속도 = ");
    Serial.print(tbl_settled[n_done - 1] * 100 / pushMsOf(n_done - 1));
    Serial.println(" 카운트/100ms");
  }

  Serial.println("# 끝. 다시 하려면 r, 리셋 버튼도 같음");
}


// ================= 중단 =================
void abortAll(unsigned long now, const char* why) {
  linearStop();
  enterState(ST_DONE, now);
  Serial.print("# ABORT : ");
  Serial.println(why);
  printTable();
}


// ================= 재시작 =================
void restartAll(unsigned long now) {
  linearStop();
  n_done   = 0;
  step_idx = 0;
  Serial.println("# RESTART");
  enterState(ST_HOME, now);
}


// ================= 시퀀스 진행 (★매 루프★ 호출) =================
void updateSeq(unsigned long now) {
  switch (state) {

    case ST_HOME:
      // 하드스톱까지 복귀. 엔코더는 보지 않는다(어디까지 갔는지가 아니라 '끝까지'가 목적).
      if (now - phase_t >= HOME_MS) enterState(ST_HOME_SETTLE, now);
      return;

    case ST_HOME_SETTLE:
      if (now - phase_t >= SETTLE_MS) {
        zeroHere();                      // ★ 여기가 이 사이클의 0 ★
        enterState(ST_PUSH, now);
        Serial.print("# PUSH ");
        Serial.print(pushMsOf(step_idx));
        Serial.println("ms");
      }
      return;

    case ST_PUSH: {
      // 안전 : 전 행정을 넘어 하드스톱을 미는 것은 즉시 중단
      if (encRead() >= ABORT_COUNT) {
        abortAll(now, "ABORT_COUNT 초과 (하드스톱 도달 의심)");
        return;
      }
      if (now - phase_t >= (unsigned long)pushMsOf(step_idx)) {
        linearStop();
        push_stop_count = encRead();     // 끊은 '순간'의 값 (coast 계산용)
        enterState(ST_HOLD, now);
      }
      return;
    }

    case ST_HOLD:
      if (now - phase_t >= SETTLE_MS) {
        recordStep(now);

        // ★ 첫 사이클은 '방향 확인'도 겸한다 ★ FWD로 밀었는데 카운트가 음수로 갔다면
        //   A/B가 뒤바뀐 것이다. 그대로 두면 종료조건(+300)에 영원히 못 닿아 MAX_STEPS까지
        //   24사이클(약 1분)을 해제 방향 하드스톱에 밀어붙이며 낭비한다 -> 즉시 중단.
        if (step_idx == 0 && tbl_settled[0] <= -DIR_CHECK_COUNT) {
          abortAll(now, "첫 사이클이 음수 = 엔코더 방향 반대. INVERT_DIR을 true로 바꿀 것");
          return;
        }

        // ★ 종료 판정 : 이번 측정이 목표에 닿았으면 이 사이클을 끝으로 마친다 ★
        if (tbl_settled[n_done - 1] >= TARGET_COUNT) {
          enterState(ST_DONE, now);
          Serial.print("# 목표 ");
          Serial.print(TARGET_COUNT);
          Serial.println(" 도달 -> 테스트 종료");
          printTable();
          return;
        }

        if (step_idx + 1 >= MAX_STEPS) {
          abortAll(now, "MAX_STEPS 도달 (목표에 못 닿았다)");
          return;
        }

        enterState(ST_REST, now);
      }
      return;

    case ST_REST:
      if (now - phase_t >= REST_MS) {
        step_idx++;                      // 다음 단계 = 시간 +STEP_MS
        enterState(ST_HOME, now);
      }
      return;

    case ST_DONE:
    default:
      return;                            // 무출력 유지 (enterState에서 이미 정지)
  }
}


// ================= 구동 중 카운트 흘려보기 (옵션) =================
void streamOut(unsigned long now) {
  if (!STREAM_ENABLED) return;
  if (state != ST_PUSH && state != ST_HOLD) return;
  if (now - stream_t < STREAM_MS) return;
  stream_t = now;
  Serial.println(encRead());
}


// ================= 시리얼 입력 (줄 단위, "s"/"r") =================
char rxBuf[8];
uint8_t rxLen = 0;

void pollSerial(unsigned long now) {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen == 1) {
        if (rxBuf[0] == 's' || rxBuf[0] == 'S') {
          if (state != ST_DONE) abortAll(now, "사용자 중단(s)");
        } else if (rxBuf[0] == 'r' || rxBuf[0] == 'R') {
          restartAll(now);
        }
      }
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = ch;
    } else {
      rxLen = 0;
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

  // 현재 레벨을 시작 상태로 채택한 뒤 인터럽트를 건다
  // (먼저 걸면 enc_state가 0이라 첫 전이가 엉뚱하게 판정된다)
  enc_state = readAB();
  enc_count = 0;
  enc_err   = 0;
  enc_dir   = 0;

  // INT4(D2)·INT5(D3)를 양쪽 엣지(CHANGE)로. EICRB의 2비트: 00=LOW, 01=양쪽엣지, 10=하강, 11=상승
  EICRB = (EICRB & ~((1 << ISC41) | (1 << ISC40) | (1 << ISC51) | (1 << ISC50)))
                 | (1 << ISC40) | (1 << ISC50);
  EIFR  = (1 << INTF4) | (1 << INTF5);
  EIMSK |= (1 << INT4) | (1 << INT5);

  unsigned long now = millis();
  stream_t = now;

  Serial.println();
  Serial.println("# linear_0803_speed : 구동시간 -> 이동량 실측 (PWM 255 고정)");
  Serial.print  ("# 사이클 = 복귀 ");
  Serial.print(HOME_MS);
  Serial.print  ("ms(REV) -> 정착 ");
  Serial.print(SETTLE_MS);
  Serial.print  ("ms+영점 -> 밀기 t ms(FWD) -> 정착+측정 -> 쉼 ");
  Serial.print(REST_MS);
  Serial.println("ms");
  Serial.print  ("# t = ");
  Serial.print(STEP_MS);
  Serial.print  ("ms부터 ");
  Serial.print(STEP_MS);
  Serial.print  ("ms씩 증가, 측정값이 ");
  Serial.print(TARGET_COUNT);
  Serial.println(" 이상이면 종료 후 표 출력");
  Serial.println("# 입력 : s=중단(표 출력) / r=재시작.  ★손을 기구에서 떼고 시작★");
  Serial.println();

  enterState(ST_HOME, now);   // 켜면 바로 시작
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  pollSerial(now);
  updateSeq(now);    // 주기 게이트 없음 — 시간 경계를 최대한 정확히 잡기 위해 매 루프
  streamOut(now);
}
