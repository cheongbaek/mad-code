// ============================================================
//  E-stop 배선 확인용 단순 테스트 코드
//  13번 핀 (NC, INPUT_PULLUP) 단락/개방 여부를 0.5초마다 출력
//  A보드/B보드 상관없이 배선 확인용으로 사용
// ============================================================

const uint8_t ESTOP_PIN = 13;
const unsigned long CHECK_MS = 500;

unsigned long lastCheck = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
}

void loop() {
  unsigned long now = millis();
  if (now - lastCheck < CHECK_MS) return;
  lastCheck = now;

  if (digitalRead(ESTOP_PIN) == LOW) {
    Serial.println("단락 (정상)");
  } else {
    Serial.println("개방 (E-STOP)");
  }
}
