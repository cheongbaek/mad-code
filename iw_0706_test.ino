//변수선언
int raw = 0;   // A15 입력값 (0 ~ 1023)
int pwm = 0;   // 0~255로 환산된 PWM 값

void setup() {
  pinMode(A15, INPUT);
  pinMode(44, OUTPUT);
}

void loop() {
  raw = analogRead(A15);
  pwm = map(raw, 0, 1023, 0, 255);

  analogWrite(44, pwm);
}
