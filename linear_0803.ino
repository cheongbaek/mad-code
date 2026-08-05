// ============================================================
//  linear_0803.ino : 로터리 엔코더 단독 동작확인 코드 (Arduino Mega 2560)
//
//  ★ 목적 ★
//    엔코더가 "돌아가는지"만 확인한다. 모터·드라이버·리미트는 일절 건드리지 않는 순수
//    계측 코드다(출력 핀을 아예 쓰지 않으므로 이 스케치만 올려도 기구가 움직일 일이 없다).
//    확인할 것은 세 가지다:
//      1) 손으로 축을 돌리면 카운트가 변하는가          -> A/B 배선·전원 확인
//      2) 시계/반시계로 돌릴 때 부호가 반대로 나오는가   -> A/B 순서(방향) 확인
//      3) 한 바퀴 돌렸을 때 카운트가 ±8192 근처인가      -> 분해능·엣지 유실 확인
//
//  ★ 엔코더 : E40S8-2048-3-T-5 (Autonics) ★
//      E40S8 = 샤프트형 Ø40 본체 / 축 Ø8
//      2048  = 2048 PPR (1회전당 A상 펄스 2048개)
//      3     = 3상 출력 (A, B, Z)
//      T     = 토템폴(push-pull) 출력  -> 외부 풀업 불필요, 5V 로직 그대로 입력 가능
//      5     = 전원 5VDC              -> 아두이노 5V 핀에서 공급 가능
//
//  ★ 배선 (5선 중 4선만 쓴다) ★
//      OUT A  -> D2   (외부 인터럽트 INT4)
//      OUT B  -> D3   (외부 인터럽트 INT5)
//      OUT C  -> 미사용 (Z상 = 1회전당 1펄스 원점신호. 이 테스트에서는 쓰지 않는다)
//      5V     -> 5V
//      GND    -> GND
//      ※ 엔코더 케이블 색은 모델·로트마다 다르므로 반드시 본체 라벨/데이터시트를 볼 것.
//      ※ 전원을 12V에 물리면 즉시 손상된다(이 모델은 5V 전용).
//
//  ★ 계수 방식 : A/B 양쪽 CHANGE 인터럽트 = 4배수(x4) 디코딩 ★
//      2048 PPR 엔코더의 A상과 B상은 90° 위상차를 가지므로, 두 신호의 모든 엣지를 세면
//      1회전당 2048 x 4 = 8192 카운트가 된다(COUNTS_PER_REV). 상태 전이 표로 세기 때문에
//      단순 펄스 카운트와 달리 **회전 방향까지 같이 나온다**(정방향 +1 / 역방향 -1).
//      전이 표에 없는 변화(A와 B가 동시에 바뀐 것으로 보이는 경우)는 엣지를 놓쳤다는
//      뜻이므로 err 카운터로 따로 센다 — 이 값이 늘어나면 너무 빠르거나 배선/노이즈 문제다.
//
//  ★ 출력 (REPORT_MS 주기, 값이 변할 때만 + 정지 중에는 IDLE_REPORT_MS마다 한 번) ★
//      count=+00008192 rev=+1.000 ang=  0.0deg rpm=+ 45.2 dir=CW  err=0 A=1 B=0
//        count : 시작(또는 리셋) 이후 누적 카운트 (x4 기준, 부호=방향)
//        rev   : 누적 회전수 = count / 8192
//        ang   : 한 바퀴 안에서의 각도 0~360도 (항상 양수로 환산)
//        rpm   : 최근 REPORT_MS 창에서 계산한 회전속도 (부호=방향)
//        dir   : CW(+) / CCW(-) / --(정지)
//        err   : 누적 전이 오류 수 (0이 정상)
//        A,B   : 지금 읽히는 A/B 레벨 (손으로 천천히 돌리며 0/1이 번갈아 바뀌는지 볼 때 유용)
//
//  ★ 입력 (시리얼 모니터 115200, 개행 전송) ★
//      r  : 카운트·오류·회전수를 0으로 리셋 (한 바퀴 정확히 세어보고 싶을 때)
//      그 외 입력은 무시한다.
//
//  동작 : delay() 미사용, millis() 기준 논블로킹 (카운트는 인터럽트로만 갱신)
// ============================================================


// ================= 핀 =================
// ★ 반드시 외부 인터럽트가 가능한 핀이어야 한다 ★
//   Mega 2560의 인터럽트 핀은 D2, D3, D18, D19, D20, D21 뿐이다.
//   (아래 ISR은 D2/D3를 포트로 직접 읽으므로, 핀을 바꾸면 readAB()도 함께 고쳐야 한다)
const uint8_t ENC_A_PIN = 2;
const uint8_t ENC_B_PIN = 3;


// ================= 엔코더 스펙 =================
const long PPR            = 2048;        // E40S8-2048 : A상 1회전당 펄스 수
const long COUNTS_PER_REV = PPR * 4;     // x4 디코딩 -> 8192


// ================= 옵션 =================
// [1] 입력 모드
//   이 모델은 T(토템폴) 출력이라 엔코더가 HIGH/LOW를 직접 구동한다 -> 풀업 불필요(INPUT).
//   만약 실물이 N(오픈컬렉터) 타입이라면 HIGH를 스스로 못 만들므로 true로 바꿔야 한다
//   (그 경우 풀업 없이는 카운트가 아예 안 늘거나 한쪽 레벨에 붙어 있는 증상이 난다).
const bool USE_PULLUP = false;

// [2] 방향 부호 뒤집기
//   "시계방향으로 돌렸는데 값이 줄어든다" 싶으면 배선을 건드리지 말고 이 값을 true로.
//   (A/B 두 선을 서로 바꿔 꽂는 것과 같은 효과)
const bool INVERT_DIR = false;

// [3] 출력 주기
const unsigned long REPORT_MS      = 200;    // 값이 변하는 동안의 출력 주기
const unsigned long IDLE_REPORT_MS = 2000;   // 정지 중에는 이 주기로만 한 줄 (로그 도배 방지)


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= x4 쿼드러처 전이 표 =================
// 상태 state = (A<<1) | B 로 두고, 인덱스 = (이전상태<<2) | 현재상태.
//   +1/-1 : 정상적인 한 칸 전이 (방향이 부호로 나온다)
//    0    : 변화 없음(같은 상태) 또는 ★두 비트가 동시에 바뀐 경우 = 엣지 유실★
// 유실은 아래 ISR에서 prev != cur 인데 표값이 0인 것으로 구분해 err로 센다.
const int8_t QDEC_TABLE[16] = {
   0, +1, -1,  0,
  -1,  0,  0, +1,
  +1,  0,  0, -1,
   0, -1, +1,  0
};


// ================= ISR이 갱신하는 값 =================
// long은 4바이트라 읽기가 원자적이지 않다 -> loop에서 읽을 때 noInterrupts()로 감싼다.
volatile long          enc_count = 0;     // 누적 카운트(x4, 부호=방향)
volatile unsigned long enc_err   = 0;     // 전이 오류(엣지 유실) 누적
volatile uint8_t       enc_state = 0;     // 직전 (A<<1)|B


// ================= A/B 레벨 읽기 =================
// ★ Mega 2560에서 D2 = PE4, D3 = PE5 ★ (Uno와 포트가 다르다)
//   digitalRead()는 호출당 수 us가 걸려 고분해능 엔코더의 ISR에서는 부담이 된다.
//   8192 카운트/회전이므로 300RPM만 돌아도 엣지 간격이 약 24us밖에 되지 않는다.
//   포트를 직접 읽으면 0.2us 수준이라 여유가 크게 생긴다.
static inline uint8_t readAB() {
  uint8_t p = PINE;
  uint8_t a = (p >> 4) & 0x01;   // PE4 = D2
  uint8_t b = (p >> 5) & 0x01;   // PE5 = D3
  return (uint8_t)((a << 1) | b);
}


// ================= ISR (A상·B상 공용) =================
// A와 B 어느 쪽 엣지든 이 함수가 불린다. 지금 상태를 읽어 직전 상태와의 전이로 판정한다.
//   - 시리얼 출력은 하지 않는다(느리고 위험). 출력은 loop가 맡는다.
//   - 두 핀을 한 번의 포트 읽기로 동시에 얻으므로 A/B가 서로 어긋나 읽힐 일이 없다.
void encISR() {
  uint8_t cur  = readAB();
  uint8_t prev = enc_state;
  if (cur == prev) return;                       // 같은 상태로의 인터럽트(글리치)는 무시

  int8_t step = QDEC_TABLE[(prev << 2) | cur];
  if (step == 0) {
    enc_err++;                                   // 두 비트가 동시에 바뀜 = 엣지를 놓쳤다
  } else {
    enc_count += INVERT_DIR ? -step : step;
  }
  enc_state = cur;
}


// ================= 리셋 =================
void resetCount() {
  noInterrupts();
  enc_count = 0;
  enc_err   = 0;
  enc_state = readAB();   // 현재 위치를 새 원점으로 (전이 판정 기준도 함께 맞춘다)
  interrupts();
  Serial.println("RESET (count=0, err=0)");
}


// ================= 시리얼 입력 =================
char rxBuf[16];
uint8_t rxLen = 0;

void handleLine(char* line) {
  if (line[0] == 'r' || line[0] == 'R') resetCount();
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
      rxLen = 0;   // 너무 긴 줄은 버린다
    }
  }
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  // 토템폴 출력이므로 기본은 INPUT(풀업 없음). USE_PULLUP=true면 내부 풀업을 켠다.
  const uint8_t mode = USE_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_A_PIN, mode);
  pinMode(ENC_B_PIN, mode);

  // 현재 레벨을 시작 상태로 채택한 뒤 인터럽트를 건다
  // (먼저 걸면 초기 enc_state가 0이라 첫 전이가 오류로 잡힐 수 있다)
  enc_state = readAB();
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encISR, CHANGE);

  Serial.println();
  Serial.println("=== linear_0803 : 로터리 엔코더 단독 동작확인 ===");
  Serial.println("엔코더 : E40S8-2048-3-T-5 (2048PPR, 3상 A/B/Z, 토템폴, 5V)");
  Serial.print  ("배선   : A=D");
  Serial.print(ENC_A_PIN);
  Serial.print(", B=D");
  Serial.print(ENC_B_PIN);
  Serial.println(", C(Z)=미사용, 5V, GND");
  Serial.print  ("계수   : A/B 양쪽 CHANGE(x4) -> 1회전 ");
  Serial.print(COUNTS_PER_REV);
  Serial.println(" 카운트");
  Serial.print  ("입력모드: ");
  Serial.println(USE_PULLUP ? "INPUT_PULLUP (오픈컬렉터용)" : "INPUT (토템폴 = 이 모델 기본)");
  Serial.print  ("방향반전: ");
  Serial.println(INVERT_DIR ? "ON" : "OFF");
  // A/B는 반드시 한 번의 읽기에서 뽑는다(두 번 읽으면 그 사이에 바뀌어 짝이 안 맞을 수 있다)
  const uint8_t ab0 = readAB();
  Serial.print  ("현재 레벨 : A=");
  Serial.print((ab0 >> 1) & 1);
  Serial.print(" B=");
  Serial.println(ab0 & 1);
  Serial.println("-----------------------------------------------");
  Serial.println("손으로 축을 천천히 돌려보세요.");
  Serial.print  ("  - 한 바퀴에 count가 약 +-");
  Serial.print(COUNTS_PER_REV);
  Serial.println(" 변하면 정상");
  Serial.println("  - 반대로 돌리면 부호가 반대로 나와야 정상");
  Serial.println("  - err가 계속 늘어나면 배선/노이즈 또는 너무 빠른 회전");
  Serial.println("  - 'r' 입력 = 카운트 리셋");
  Serial.println("===============================================");
}


// ================= 한 줄 출력 =================
void report(long count, unsigned long err, long delta, unsigned long window_ms) {
  // 회전수·각도 : x4 카운트 기준
  float rev = (float)count / (float)COUNTS_PER_REV;

  // 한 바퀴 안에서의 각도(0~360). 음수 카운트에서도 양수로 나오게 보정한다.
  long m = count % COUNTS_PER_REV;
  if (m < 0) m += COUNTS_PER_REV;
  float ang = (float)m * 360.0f / (float)COUNTS_PER_REV;

  // rpm = (창 안의 회전수) * (분당 창 개수)
  float rpm = 0.0f;
  if (window_ms > 0) {
    rpm = ((float)delta / (float)COUNTS_PER_REV) * (60000.0f / (float)window_ms);
  }

  uint8_t ab = readAB();

  Serial.print("count=");
  if (count >= 0) Serial.print('+');
  Serial.print(count);
  Serial.print(" rev=");
  if (rev >= 0) Serial.print('+');
  Serial.print(rev, 3);
  Serial.print(" ang=");
  Serial.print(ang, 1);
  Serial.print("deg rpm=");
  if (rpm >= 0) Serial.print('+');
  Serial.print(rpm, 1);
  Serial.print(" dir=");
  Serial.print(delta > 0 ? "CW " : (delta < 0 ? "CCW" : "-- "));
  Serial.print(" err=");
  Serial.print(err);
  Serial.print(" A=");
  Serial.print((ab >> 1) & 1);
  Serial.print(" B=");
  Serial.println(ab & 1);
}


// ================= loop =================
void loop() {
  static unsigned long last_report_t     = 0;
  static long          last_count        = 0;
  static unsigned long last_err          = 0;
  static unsigned long last_idle_print_t = 0;

  pollSerial();

  unsigned long now = millis();
  if (now - last_report_t < REPORT_MS) return;

  // ISR과 겹치지 않게 한 번에 떠온다 (4바이트 읽기는 원자적이지 않다)
  long          count;
  unsigned long err;
  noInterrupts();
  count = enc_count;
  err   = enc_err;
  interrupts();

  long delta = count - last_count;
  unsigned long window_ms = now - last_report_t;

  // 움직임(또는 새 오류)이 있으면 매 주기, 정지 중이면 IDLE_REPORT_MS마다 한 줄만.
  // 정지 중에도 주기적으로 찍어주는 이유 : 연결이 살아 있다는 것과 현재 A/B 레벨을
  // 계속 보여줘야 "아무 반응이 없다"와 "돌리지 않아서 그대로다"를 구분할 수 있다.
  bool moved = (delta != 0) || (err != last_err);
  if (moved || (now - last_idle_print_t >= IDLE_REPORT_MS)) {
    report(count, err, delta, window_ms);
    last_idle_print_t = now;
  }

  last_report_t = now;
  last_count    = count;
  last_err      = err;
}
