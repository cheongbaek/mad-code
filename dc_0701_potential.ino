// =====================================================
// 조향 피드백 포텐셔미터 모니터링 코드
// 목적: 손으로 모터를 돌렸을 때 A0 가변저항 값 확인
// =====================================================

const uint8_t DC_POT_PIN = A2;

int measured_min = 1023;
int measured_max = 0;

void setup() {
  Serial.begin(115200);
  pinMode(DC_POT_PIN, INPUT);

  Serial.println("==============================");
  Serial.println(" 피드백 포텐셔미터 모니터링");
  Serial.println(" 모터를 손으로 돌려보세요");
  Serial.println("==============================");
}

void loop() {
  int currentPos = analogRead(DC_POT_PIN);

  // 최솟값/최댓값 자동 추적
  if (currentPos < measured_min) measured_min = currentPos;
  if (currentPos > measured_max) measured_max = currentPos;

  // 레퍼런스 코드와 동일한 출력 포맷
  Serial.print("C:"); Serial.print(currentPos);
  Serial.print("  MIN:"); Serial.print(measured_min);
  Serial.print("  MAX:"); Serial.println(measured_max);

  delay(100);
}