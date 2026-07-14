// ============================================================
//  DC 조향모터 방향 확인 테스트 코드
//  목적: 소프트웨어상 정의된 "왼쪽"/"오른쪽" 방향과 실제 설치된
//        모터 회전방향이 일치하는지 확인 (왼쪽 0.5초 → 오른쪽 0.5초)
//  방향 정의(DIR_CW/DIR_CCW) : kasa_0708_together.ino 와 동일
//    - DIR_CW  (HIGH) → 조향 raw값 증가 방향 = "왼쪽"으로 정의
//    - DIR_CCW (LOW)  → 조향 raw값 감소 방향 = "오른쪽"으로 정의
// ============================================================

const uint8_t DC_DIR_PIN = 6;
const uint8_t DC_PWM_PIN = 7;

#define DIR_CW   HIGH
#define DIR_CCW  LOW

const int TEST_PWM = 100;            // kasa_0708_together.ino STEER_MIN_PWM 값 참고
const unsigned long TURN_MS = 500;   // 방향별 회전 시간 0.5초
const unsigned long GAP_MS  = 300;   // 방향 전환 사이 관성 정지 대기

void dcStop()     { analogWrite(DC_PWM_PIN, 0); }
void dcCW(int p)  { digitalWrite(DC_DIR_PIN, DIR_CW);  analogWrite(DC_PWM_PIN, p); }
void dcCCW(int p) { digitalWrite(DC_DIR_PIN, DIR_CCW); analogWrite(DC_PWM_PIN, p); }

enum TestState { T_LEFT, T_GAP, T_RIGHT, T_DONE };
TestState state = T_LEFT;
unsigned long stateT = 0;

void setup() {
  Serial.begin(115200);

  pinMode(DC_DIR_PIN, OUTPUT);
  pinMode(DC_PWM_PIN, OUTPUT);
  dcStop();

  Serial.println(F("=== DC 조향모터 방향 확인 테스트 ==="));
  Serial.println(F("-> 왼쪽(CW) 방향 0.5초 회전"));
  dcCW(TEST_PWM);
  stateT = millis();
}

void loop() {
  unsigned long now = millis();

  switch (state) {
    case T_LEFT:
      if (now - stateT >= TURN_MS) {
        dcStop();
        state = T_GAP;
        stateT = now;
      }
      break;

    case T_GAP:
      if (now - stateT >= GAP_MS) {
        Serial.println(F("-> 오른쪽(CCW) 방향 0.5초 회전"));
        dcCCW(TEST_PWM);
        state = T_RIGHT;
        stateT = now;
      }
      break;

    case T_RIGHT:
      if (now - stateT >= TURN_MS) {
        dcStop();
        Serial.println(F("=== 테스트 종료 ==="));
        Serial.println(F("실제로 왼쪽 -> 오른쪽 순으로 돌았는지 확인하세요."));
        Serial.println(F("반대로 돌았다면 DC_DIR_PIN 배선 또는 DIR_CW/DIR_CCW 정의를 반전하세요."));
        state = T_DONE;
      }
      break;

    case T_DONE:
      // 테스트 완료, 추가 동작 없음
      break;
  }
}
