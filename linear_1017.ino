// ============================================================
//  linear_1017.ino : 시리얼로 받은 시간만큼 리니어를 민다 (Arduino Mega 2560)
//    양수 = 나오는 방향, 음수 = 들어가는 방향, 단위 초.   예) 0.4  /  -1
//    PWM 255 고정. 가변저항은 보지 않는다.
//    핀 : D47 리니어 DIR (LOW = 나옴 / HIGH = 들어감) / D45 리니어 PWM  (통합 보드 kasa_1017~ [1017-1] 배선)
//         ★[2026-10-25] D9 / D8([1015-2] 옛 B보드) → D47 / D45★
// ============================================================

const uint8_t LINEAR_DIR_PIN = 47;
const uint8_t LINEAR_PWM_PIN = 45;
const int     PWM_MAX = 255;

void setup() {
  Serial.begin(115200);
  pinMode(LINEAR_DIR_PIN, OUTPUT);
  pinMode(LINEAR_PWM_PIN, OUTPUT);
  analogWrite(LINEAR_PWM_PIN, 0);
  Serial.println("초 입력 : 양수 = 나옴, 음수 = 들어감 (예: 0.4 / -1)");
}

void loop() {
  if (Serial.available() == 0) return;

  float sec = Serial.readStringUntil('\n').toFloat();
  if (sec == 0) return;

  unsigned long ms = (unsigned long)((sec > 0 ? sec : -sec) * 1000.0 + 0.5);

  digitalWrite(LINEAR_DIR_PIN, sec > 0 ? LOW : HIGH);
  analogWrite(LINEAR_PWM_PIN, PWM_MAX);
  delay(ms);
  analogWrite(LINEAR_PWM_PIN, 0);

  Serial.print(sec > 0 ? "나옴 " : "들어감 ");
  Serial.print(ms / 1000.0, 3);
  Serial.println(" 초");
}
