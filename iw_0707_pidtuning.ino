//변수선언
volatile long encCount = 0;
long lastCount = 0;
unsigned long lastTime = 0;
int target = 0;
float i = 0;
int lastErr = 0;

//PID 게인
float kp = 2.0;
float ki = 0.2;
float kd = 1.2;


void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP);   // 미사용 (자리만)
  pinMode(3, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(2), enc, CHANGE);  // 2번 미사용
  attachInterrupt(digitalPinToInterrupt(3), enc, CHANGE);   // 3번펄스만 사용

  pinMode(10, OUTPUT);

  lastTime = millis();
}

void loop() {
  if (Serial.available()) {
    target = Serial.parseInt();
    Serial.read();
  }
// 밑부분에 제어주기 설정
  if (millis() - lastTime >= 50) {   // 제어주기 50ms
    lastTime = millis();

    long speed = encCount - lastCount;
    lastCount = encCount;

    int err = abs(target) - (int)speed;
    int d = err - lastErr;
    lastErr = err;

    int pwm = kp * err + ki * i + kd * d;

    // 출력이 포화(0 또는 255)면 적분 누적 중단 → windup 방지
    if (pwm > 0 && pwm < 255) {
      i += err;
    }

    // 계산 다시 (누적 반영)
    pwm = kp * err + ki * i + kd * d;
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;

    analogWrite(10, pwm);
    Serial.print(pwm);
    Serial.print(" ");
    Serial.print(target);
    Serial.print(" ");
    Serial.print(speed);
    Serial.print(" ");
    Serial.print(0);
    Serial.print(" ");
    Serial.println(80);
  }
}

void enc() {
  encCount++;
}
