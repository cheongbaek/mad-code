//변수선언
volatile long encCount2 = 0;
volatile long encCount3 = 0;
long lastCount2 = 0;
long lastCount3 = 0;
unsigned long lastTime = 0;
int pwm = 0;      // 시리얼로 받는 PWM 값 (0 ~ 255)

void setup() {
  Serial.begin(115200);

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), enc3, CHANGE);

  pinMode(10, OUTPUT);

  lastTime = millis();
}

void loop() {
  // 시리얼로 PWM 값 수신 (0 ~ 255)
  if (Serial.available()) {
    pwm = Serial.parseInt();
    Serial.read();
    if (pwm > 255) pwm = 255;
    if (pwm < 0) pwm = 0;
  }

  // 받은 PWM 값을 그대로 출력
  analogWrite(10, pwm);

  if (millis() - lastTime >= 50) {   // 제어주기 50ms
    lastTime = millis();

    long c2, c3;
    noInterrupts();
    c2 = encCount2;
    c3 = encCount3;
    interrupts();

    long speed2 = c2 - lastCount2;
    long speed3 = c3 - lastCount3;
    lastCount2 = c2;
    lastCount3 = c3;

    Serial.print(pwm);
    Serial.print(" ");
    Serial.print(speed2);
    Serial.print(" ");
    Serial.print(speed3);
    Serial.print(" ");
    Serial.println(speed2 + speed3);
  }
}

void enc2() {
  encCount2++;
}

void enc3() {
  encCount3++;
}
