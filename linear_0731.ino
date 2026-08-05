// ============================================================
//  linear_0731.ino : 리니어(브레이크)모터 + 리미트 스위치 누드 테스트 (Arduino Mega 2560)
//  linear_0718.ino의 직접 구동 방식(초 + 부호있는 PWM)을 그대로 쓰고, 여기에
//  리미트 스위치(D2) 감시만 추가한 코드. 모터드라이버는 MD20A(DIR + PWM).
//
//  ★ 목적 ★
//    1) 어떤 방향·PWM·시간이 이 기구에 적절한지 값을 직접 넣어보며 실측
//       (리니어에 가변저항이 없어 위치를 알 수 없으므로 눈으로 보고 맞추는 수밖에 없다.
//        여기서 찾은 값을 kasa_0731_B.ino의 브레이크 전이 파라미터에 옮겨 넣는다)
//    2) 리미트 스위치가 눌림/떼짐을 제대로 감지하는지 확인
//
//  ★ 배선 (이 3개만 연결하면 된다) ★
//    리니어 DIR : D8
//    리니어 PWM : D9
//    리미트     : D2 ── 스위치 ── GND   (INPUT_PULLUP, 외부 풀업저항 불필요)
//                 ★ 2026-07-31 실측 : 누르면 통전(LOW) = 눌림 / 떼면 개방(HIGH) = 정상 ★
//    리니어모터 단자 : MB-RED, MA-BLACK
//
//  ★ 사용법 ★
//    시리얼 모니터(115200, 개행 전송)에 "<구동시간(초)>,<PWM>" 형식으로 입력한다.
//      - 구동시간 : 0보다 큰 실수 (예: 3, 0.1, 1.5)
//      - PWM      : 부호있는 정수 -255~255 (★부호 = 방향★, 절댓값 = 세기)
//          "0.1,255"  -> 0.1초 동안 PWM 255, 양수 방향(= MA에 +, 바깥으로 나오는 방향)
//          "0.1,-255" -> 0.1초 동안 PWM 255, 음수 방향(= MB에 +, 안쪽으로 들어가는 방향)
//          "1,0"      -> 즉시 정지
//      - 형식 오류/시간<=0 은 그냥 무시한다
//      - 구동 중 새 명령이 들어오면 기존 구동을 중단하고 새 명령으로 갱신한다
//    ★ 방향 규약 (2026-07-31 확정) ★
//       MB에 12V -> 안쪽으로 들어감 / MA에 12V -> 바깥으로 나옴
//       따라서 이 코드는 '양수 PWM = MA에 + = 바깥으로 나옴'을 의도한다.
//       다만 DIR 레벨과 MA/MB의 대응은 드라이버·배선에 달려 있어 코드로는 보장할 수 없다.
//       "3,255"를 넣어 실제로 바깥으로 나오는지 확인하고, 반대면 아래
//       DIR_FOR_POSITIVE / DIR_FOR_NEGATIVE 두 값을 서로 바꾼다(배선은 그대로).
//    ※ 그 확인 결과를 B코드의 LINEAR_FWD/LINEAR_REV 정의에도 반영해야 한다
//      (B코드의 '정방향'은 브레이크를 밟는 방향이다 — 어느 쪽이 밟는 것인지도 함께 확인).
//
//  ★ 출력 ★
//    "RUN,<초>,<pwm>"  : 구동 시작
//    "DONE"            : 구동시간 만료로 정상 종료
//    "STOP,0"          : PWM 0 명령으로 정지
//    "LIMIT"           : 리미트가 눌린 순간 한 번만 (눌려 있는 동안 반복 출력하지 않는다)
//                        구동 중이었다면 "(구동 시작 후 NNms / 설정 NNms)"를 함께 찍는다 —
//                        설정 시간이 기구 행정보다 긴지 판단하는 근거가 된다.
//    "LIMIT RELEASED"  : 떼짐이 확정된 순간 (이후 다시 눌리면 "LIMIT"가 또 나온다)
//
//  ★ 리미트가 최우선 ★
//    눌린 순간 ISR이 곧바로 PWM을 0으로 떨어뜨린다. 리니어는 PWM만 0으로 두면 그 위치에
//    머물기 때문에, 늦게 반응하면 기구를 계속 밀어붙인 상태가 된다.
//    - 밟힘은 디바운스 없이 즉시 (늦는 것이 위험하고, 노이즈로 한 번 더 힘을 빼는 것은 무해)
//    - 떼짐만 micros 기준 LIMIT_RELEASE_CONFIRM_US 동안 유지되어야 인정 (접점 바운스 제거)
//    - 눌린 동안에는 방향을 가리지 않고 어떤 구동도 시작하지 않는다
//      (브레이크는 사람이 주로 밟고 리니어는 보조 수단이라, 애매할 때 힘을 빼는 쪽이 안전하다)
//
//  동작 : delay() 미사용, millis() 기준 논블로킹 (리미트 떼짐 판정만 micros 사용)
// ============================================================


// ================= 핀 정의 =================
// ★ DIR/PWM이 서로 바뀌어 연결되어 있지 않은지 반드시 확인할 것 ★
//   드라이버 커넥터의 라벨 순서를 착각해 두 선이 바뀌면, 드라이버의 DIR 입력에 PWM 신호가
//   들어가 방향이 수백 Hz로 뒤집힌다. 그러면 모터는 회전하지 못하고 "떨거나 돌려는 시늉만
//   하다 마는" 증상이 되는데, 드라이버 LED는 신호를 받으니 정상처럼 보인다.
//   의심되면 배선을 건드리지 말고 아래 두 숫자를 서로 바꿔(8<->9) 한 번 시험해 볼 것.
//   (D8은 Timer4, D9는 Timer2 PWM 핀이라 바꿔도 analogWrite는 정상 동작한다)
const uint8_t LINEAR_DIR_PIN = 8;
const uint8_t LINEAR_PWM_PIN = 9;
const uint8_t LIMIT_PIN      = 2;   // 외부 인터럽트 가능 핀


// ================= ★ 진단 옵션 (MD20A 궁합 확인용) ★ =================
// [1] PWM 주파수
//   PWM 핀(기본 D9)은 Timer2를 쓰고, 아두이노 기본 주파수는 약 490Hz다. 인덕턴스가 큰
//   리니어 액추에이터에서는 이 낮은 주파수에서 전류 리플이 커져, 평균 전류는 낮은데도
//   피크 전류가 드라이버의 과전류 보호를 건드릴 수 있다(= 잠깐 힘이 들어가다 끊긴다).
//   MD20A는 최대 20kHz를 지원하므로 올려서 비교해 볼 수 있다.
//   ※ Timer2는 D9/D10 PWM 전용이라 millis()/micros()(Timer0)에는 영향이 없다.
//   ※ 듀티 100%(PWM 255)에서는 스위칭이 없어 주파수가 무관하다 — 255에서도 안 돌면
//     주파수 문제가 아니라 전원/배선 쪽이다.
//     0 = 기본 490Hz 유지 / 1 = 약 980Hz / 2 = 약 3.9kHz / 3 = 약 31kHz(권장 상한 초과)
const uint8_t PWM_FREQ_MODE = 0;

// [2] 소프트 스타트
//   0보다 크면 목표 PWM까지 이 시간에 걸쳐 선형으로 올린다. 기동 전류 피크를 낮춰
//   드라이버 보호가 걸리는지 확인하는 데 쓴다 (예: 200).  0이면 즉시 목표값.
const unsigned long SOFT_START_MS = 0;


// ================= ★ DIR 매핑 (여기서 바꿀 수 있다) ★ =================
// ★ 기구 규약 (2026-07-31 확정) ★
//     MB에 12V(+)  ->  로드가 안쪽으로 들어간다 (수축)
//     MA에 12V(+)  ->  로드가 바깥으로 나온다   (신장)
//   그래서 이 코드의 목표 규약은 이렇다:
//     양수 PWM  =  MA에 +  =  바깥으로 나옴
//     음수 PWM  =  MB에 +  =  안쪽으로 들어감
//
// ※★ 아래 두 값이 그 규약을 만족하는지는 코드로 알 수 없다 ★
//   코드가 정하는 것은 "양수일 때 DIR 핀을 어느 레벨로 둘지"까지다. 그 DIR 레벨에서
//   드라이버가 MA에 +를 주는지 MB에 +를 주는지는 MD20A의 내부 로직과 모터 단자를
//   M+/M-에 어떻게 꽂았는지에 달려 있어, 실측으로만 확인된다.
//   확인 방법 : "3,255"를 넣어보고 로드가 '바깥으로 나오면' 이 매핑이 맞다.
//               '안쪽으로 들어가면' 아래 두 값을 서로 바꾸면 된다(배선은 그대로).
//   (지금 값은 linear_0718.ino와 같은 매핑 — 양수 = HIGH — 으로 시작한 것이다)
const uint8_t DIR_FOR_POSITIVE = HIGH;   // 양수 PWM(= MA에 +, 바깥으로) 에 줄 DIR 레벨
const uint8_t DIR_FOR_NEGATIVE = LOW;    // 음수 PWM(= MB에 +, 안쪽으로) 에 줄 DIR 레벨


// ================= ★ 리미트 '눌림'으로 읽을 레벨 (2026-07-31 실측으로 확정) ★ =================
// 실측 결과 떼면 걸리고 누르면 풀렸다 = 누를 때 통전(LOW)되는 접점이었다.
// 즉 NC가 아니라 NO(Normally Open)처럼 동작한다 -> 눌림 = LOW.
// ※ 스위치를 NC 접점으로 다시 배선하면 이 한 줄만 HIGH로 바꾸면 된다.
//   (NC 쪽이 단선 시에도 '눌림'으로 읽혀 페일세이프가 되므로 원래 그쪽을 의도했다)
const uint8_t LIMIT_PRESSED_LEVEL = LOW;

// 떼짐 확정에 필요한 유지시간. 밟힘은 즉시 차단하고 떼짐만 이 시간으로 확인한다.
// 반응이 늦으면 기구를 계속 밀어붙이므로 접점 바운스만 겨우 걸러낼 만큼 짧게 잡는다.
const unsigned long LIMIT_RELEASE_CONFIRM_US = 3000;   // 3ms


// ================= 통신 =================
const unsigned long BAUD = 115200;


// ================= PWM 상한 =================
const int PWM_MAX = 255;


// ================= 구동 상태 =================
bool running = false;
unsigned long run_start_t = 0;
unsigned long run_dur_ms  = 0;
int  run_pwm = 0;             // 지금 구동 중인 명령값 (부호 포함, 로그용)
int  run_mag = 0;             // 목표 PWM 절댓값 (소프트 스타트의 도착점)


// ================= 리미트 스위치 상태 =================
// ISR이 갱신하는 값은 volatile. 4바이트 읽기는 원자적이지 않으므로 loop에서 읽을 때
// noInterrupts()로 감싼다.
volatile bool          limit_hit       = false;   // 지금 눌려 있는가 (ISR 즉시 갱신)
volatile unsigned long limit_change_us = 0;       // 마지막 엣지 시각 (micros)
bool limit_release_pending = false;   // 떼짐 신호를 봤지만 아직 확정 전
bool limit_printed         = false;   // "LIMIT"를 이미 출력했다 (눌린 동안 1회만)


// ================= 시리얼 입력 버퍼 =================
char rxBuf[48];
uint8_t rxLen = 0;


// ================= 모터 출력 =================
void linearStop() { analogWrite(LINEAR_PWM_PIN, 0); }

void linearDrive(uint8_t dir, int p) {
  digitalWrite(LINEAR_DIR_PIN, dir);
  analogWrite(LINEAR_PWM_PIN, constrain(p, 0, PWM_MAX));
}


// ================= [진단 1] PWM 주파수 변경 (Timer2 프리스케일) =================
// Timer2는 phase-correct PWM(분해능 510)으로 돌아가므로 주파수 = 16MHz / (프리스케일 * 510).
// millis()/micros()는 Timer0을 쓰므로 여기서 바꿔도 시간 계산에는 영향이 없다.
void applyPwmFreq() {
  uint8_t cs;
  switch (PWM_FREQ_MODE) {
    case 1:  cs = 0x03; break;   // /32 -> 약 980Hz
    case 2:  cs = 0x02; break;   // /8  -> 약 3.9kHz
    case 3:  cs = 0x01; break;   // /1  -> 약 31kHz (MD20A 권장 상한 20kHz 초과, 참고용)
    default: return;             // 0 = 아두이노 기본(약 490Hz) 그대로
  }
  TCCR2B = (TCCR2B & 0b11111000) | cs;
}


// ================= 리미트 스위치 ISR =================
// D2 CHANGE 인터럽트. 눌림을 본 '즉시' PWM을 0으로 떨어뜨리는 것이 이 함수의 목적이다.
//   - analogWrite(pin, 0)은 내부적으로 digitalWrite(pin, LOW)라 ISR에서 호출해도 안전하다.
//   - micros()도 ISR에서 호출 가능하다(타이머0 카운터를 읽을 뿐).
//   - 시리얼 출력은 ISR에서 하지 않는다(느리고 위험). 출력은 loop의 updateLimit()이 맡는다.
void limitISR() {
  limit_change_us = micros();
  limit_hit = (digitalRead(LIMIT_PIN) == LIMIT_PRESSED_LEVEL);
  if (limit_hit) {
    analogWrite(LINEAR_PWM_PIN, 0);   // ★ 즉시 힘 빼기 ★
  }
}


// ================= 리미트가 구동을 막고 있는가 =================
bool limitBlocking() {
  return limit_hit || limit_release_pending;
}


// ================= 리미트 상태 갱신 (loop 최상위에서 호출) =================
void updateLimit() {
  bool hit;
  unsigned long changed;
  noInterrupts();
  hit     = limit_hit;
  changed = limit_change_us;
  interrupts();

  if (hit) {
    linearStop();
    limit_release_pending = true;   // 이후 떼져도 확정 대기를 거친다

    if (!limit_printed) {
      limit_printed = true;
      Serial.print("LIMIT");
      if (running) {
        // 설정 시간이 기구 행정보다 긴지 판단하는 근거 : 구동 시작 후 몇 ms에 닿았는가
        Serial.print("   (pwm=");
        Serial.print(run_pwm);
        Serial.print(" 구동 시작 후 ");
        Serial.print(millis() - run_start_t);
        Serial.print("ms / 설정 ");
        Serial.print(run_dur_ms);
        Serial.print("ms)");
      }
      Serial.println();
    }

    if (running) {
      running = false;   // 진행 중이던 구동은 취소 (시간이 남았어도 재개하지 않는다)
      Serial.println("  -> 구동 중단");
    }
    return;
  }

  // 떼짐 신호를 본 상태 : micros 기준으로 짧게 유지되어야 확정
  if (limit_release_pending) {
    if (micros() - changed >= LIMIT_RELEASE_CONFIRM_US) {
      limit_release_pending = false;
      limit_printed = false;      // 다음에 다시 눌리면 "LIMIT"를 또 출력한다
      Serial.println("LIMIT RELEASED");
    } else {
      linearStop();               // 확정 전에는 계속 힘을 빼둔다
    }
  }
}


// ================= 입력 형식 검사 =================
// 시간 토큰: 0~9와 '.' 하나만 허용(부호 없음, 최소 숫자 1개)
bool isValidSeconds(const char* s) {
  if (!s || *s == '\0') return false;
  bool sawDigit = false, sawDot = false;
  for (uint8_t i = 0; s[i] != '\0'; i++) {
    if (s[i] == '.') {
      if (sawDot) return false;
      sawDot = true;
    } else if (isdigit((unsigned char)s[i])) {
      sawDigit = true;
    } else {
      return false;
    }
  }
  return sawDigit;
}

// PWM 토큰: 부호(+/-) 허용 정수
bool isValidSignedInt(const char* s) {
  if (!s || *s == '\0') return false;
  uint8_t i = 0;
  if (s[0] == '-' || s[0] == '+') i = 1;
  if (s[i] == '\0') return false;
  for (; s[i] != '\0'; i++) {
    if (!isdigit((unsigned char)s[i])) return false;
  }
  return true;
}


// ================= 구동 시작 =================
void startRun(float seconds, int pwmVal) {
  pwmVal = constrain(pwmVal, -PWM_MAX, PWM_MAX);

  if (pwmVal == 0) {
    linearStop();
    running = false;
    run_pwm = 0;
    Serial.println("STOP,0");
    return;
  }

  // ★ 리미트가 눌려 있으면 방향을 가리지 않고 구동하지 않는다 ★
  if (limitBlocking()) {
    linearStop();
    running = false;
    Serial.println("리미트가 눌려 있어 구동하지 않습니다 (떼고 다시 입력하세요)");
    return;
  }

  uint8_t dir = (pwmVal > 0) ? DIR_FOR_POSITIVE : DIR_FOR_NEGATIVE;
  run_mag = (pwmVal > 0) ? pwmVal : -pwmVal;

  // [진단 2] 소프트 스타트를 쓰면 0에서 출발하고 updateRun()이 목표까지 올린다
  linearDrive(dir, (SOFT_START_MS > 0) ? 0 : run_mag);

  run_start_t = millis();
  run_dur_ms  = (unsigned long)(seconds * 1000.0f);
  run_pwm     = pwmVal;
  running     = true;

  Serial.print("RUN,");
  Serial.print(seconds, 2);
  Serial.print(',');
  Serial.print(pwmVal);
  Serial.print("   (DIR=");
  Serial.print(dir);
  // 의도한 방향을 함께 찍는다 — 실제 움직임이 이와 다르면 DIR 매핑 두 값을 서로 바꾼다
  Serial.println(pwmVal > 0 ? ", 의도: MA에 + = 바깥으로 나옴)"
                            : ", 의도: MB에 + = 안쪽으로 들어감)");
}


// ================= setup =================
void setup() {
  Serial.begin(BAUD);

  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  digitalWrite(LINEAR_DIR_PIN, DIR_FOR_POSITIVE);
  linearStop();
  applyPwmFreq();   // [진단 1] PWM_FREQ_MODE가 0이 아니면 주파수를 바꾼다

  // 리미트 스위치 : D2 ── 스위치 ── GND, 내부 풀업. CHANGE 인터럽트로 양쪽 엣지 감시.
  //   부팅 시 실제 핀 상태를 그대로 채택한다(이미 눌린 채로 켜져도 구동하지 않도록).
  pinMode(LIMIT_PIN, INPUT_PULLUP);
  limit_hit = (digitalRead(LIMIT_PIN) == LIMIT_PRESSED_LEVEL);
  limit_change_us = micros();
  limit_release_pending = limit_hit;
  limit_printed = false;
  attachInterrupt(digitalPinToInterrupt(LIMIT_PIN), limitISR, CHANGE);

  Serial.println();
  Serial.println("=== linear_0731 : 리니어 + 리미트 누드 테스트 ===");
  Serial.println("배선 : DIR=D8, PWM=D9, 리미트=D2(스위치-GND)");
  Serial.println("입력 : <구동시간(초)>,<PWM>   예) 0.1,255 / 0.1,-255 / 1,0(정지)");
  Serial.print("DIR 매핑 : 양수 PWM -> DIR ");
  Serial.print(DIR_FOR_POSITIVE);
  Serial.print(" (의도: MA에 + = 바깥으로) / 음수 PWM -> DIR ");
  Serial.print(DIR_FOR_NEGATIVE);
  Serial.println(" (의도: MB에 + = 안쪽으로)");
  Serial.println("  ※ 실제 움직임이 반대면 DIR_FOR_POSITIVE/NEGATIVE 두 값을 서로 바꿀 것");
  Serial.print("핀 : DIR=D");
  Serial.print(LINEAR_DIR_PIN);
  Serial.print(", PWM=D");
  Serial.println(LINEAR_PWM_PIN);
  Serial.print("PWM 주파수 모드 : ");
  Serial.print(PWM_FREQ_MODE);
  Serial.println(" (0=490Hz 기본 / 1=980Hz / 2=3.9kHz / 3=31kHz)");
  Serial.print("소프트 스타트 : ");
  Serial.print(SOFT_START_MS);
  Serial.println("ms (0=즉시)");
  Serial.print("리미트 현재 상태 : ");
  Serial.println(limit_hit ? "눌림" : "정상");
  Serial.println("=============================================");
}


// ================= 입력 파서 =================
// "<구동시간(초)>,<PWM>" 콤마 구분 2개. 형식이 안 맞으면 무시.
void handleLine(char* line) {
  char* tok1 = strtok(line, ",");
  char* tok2 = tok1 ? strtok(NULL, ",") : NULL;
  char* tok3 = tok2 ? strtok(NULL, ",") : NULL;   // 토큰이 3개 이상이면 형식 오류

  if (!tok1 || !tok2 || tok3 ||
      !isValidSeconds(tok1) || !isValidSignedInt(tok2)) {
    return;
  }

  float seconds = atof(tok1);
  if (seconds <= 0) return;

  int pwmVal = atoi(tok2);
  startRun(seconds, pwmVal);
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


// ================= 구동시간 관리 (millis 기준, 논블로킹) =================
void updateRun(unsigned long now) {
  if (!running) return;

  // 리미트가 걸려 있으면 출력을 다시 올리지 않는다 (ISR이 끊어둔 것을 되살리지 않도록)
  if (limitBlocking()) return;

  // [진단 2] 소프트 스타트: 목표까지 선형 램프 (방향은 startRun에서 이미 정해졌다)
  if (SOFT_START_MS > 0) {
    unsigned long el = now - run_start_t;
    int p = (el >= SOFT_START_MS)
              ? run_mag
              : (int)((long)run_mag * (long)el / (long)SOFT_START_MS);
    analogWrite(LINEAR_PWM_PIN, constrain(p, 0, PWM_MAX));
  }

  if (now - run_start_t >= run_dur_ms) {
    linearStop();
    running = false;
    Serial.println("DONE");
  }
}


// ================= loop =================
void loop() {
  unsigned long now = millis();

  // ★ 최우선 : 리미트 스위치 ★
  //   ISR이 이미 PWM을 떨어뜨렸더라도, 여기서 상태를 정리하고 힘이 빠진 것을 확정한다.
  updateLimit();

  pollSerial();
  updateRun(now);
}
