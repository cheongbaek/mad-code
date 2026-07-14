// ============================================================
//  PWM -> 펄스 정적 특성 측정 전용 펌웨어 (Arduino Mega 2560)
//  PID 없음. PWM을 60부터 10씩 170까지 올려가며 각 단계 5초 유지,
//  3~5초 구간(정착 후)의 펄스수를 "9개씩 묶어 최빈값 추출 후 평균"으로
//  집계해서, 스윕이 다 끝난 뒤 PWM을 끄고 결과표를 한 번에 출력한다.
//  (PWM-펄스 관계의 비선형성을 실측 테이블로 확인하기 위한 용도)
//
//  홀펄스 : 20/21번 핀 모두 수신 (kasa_0709_none.ino와 동일 배선).
//  PWM 출력 : 핀 10.
//
//  동작 :
//    1) 업로드 후 시리얼 모니터 연결, 아무 키나 입력하면 스윕 시작
//    2) PWM 60→170 (10 단위) 각 5초씩 자동 인가
//       - 0~3초 : 정착 구간 (기록 안 함)
//       - 3~5초 : 20ms마다 21/20번 펄스수 샘플링(100개), 9개씩 묶어
//                 각 묶음의 최빈값을 뽑고 그 최빈값들을 평균 → 대표값
//                 (100을 9로 나눈 나머지 1개는 버림)
//    3) 스윕 종료(또는 중단) 시 PWM 0, 결과를 한 번에 출력
//       "PWM<레벨> : <21번 평균> <20번 평균>"  (소수 2자리)
//    4) 스윕 중 아무 키나 입력하면 그 즉시 중단(PWM 0)하고
//       그때까지 완료된 레벨의 결과만 출력
// ============================================================

const uint8_t HALL_PIN20 = 20;
const uint8_t HALL_PIN21 = 21;
const uint8_t PWM_PIN    = 10;

const int PWM_START = 60;
const int PWM_END   = 170;
const int PWM_STEP  = 10;
const int NUM_LEVELS = (PWM_END - PWM_START) / PWM_STEP + 1;   // 12

const unsigned long HOLD_MS         = 5000;  // 레벨당 유지 시간
const unsigned long RECORD_START_MS = 3000;  // 기록 시작 시점 (정착 대기)
const unsigned long RECORD_END_MS   = 5000;  // 기록 종료 시점
const unsigned long SAMPLE_MS       = 20;    // 샘플링 주기
const int MAX_SAMPLES = (RECORD_END_MS - RECORD_START_MS) / SAMPLE_MS;  // 100
const int GROUP_SIZE  = 9;                   // 최빈값 추출 묶음 크기

volatile long encCount20 = 0;
volatile long encCount21 = 0;
void encISR20() { encCount20++; }
void encISR21() { encCount21++; }

float resultAvg21[NUM_LEVELS];
float resultAvg20[NUM_LEVELS];
int   resultLevel[NUM_LEVELS];
int   resultCount = 0;   // 실제로 채워진 레벨 수 (중단 시 NUM_LEVELS보다 작을 수 있음)

bool aborted = false;

// 묶음(길이 len) 중 최빈값. 동률이면 먼저 나온 값을 채택.
long findMode(long* arr, int len) {
  long bestVal = arr[0];
  int  bestCount = 0;
  for (int i = 0; i < len; i++) {
    int cnt = 0;
    for (int j = 0; j < len; j++) if (arr[j] == arr[i]) cnt++;
    if (cnt > bestCount) { bestCount = cnt; bestVal = arr[i]; }
  }
  return bestVal;
}

// 샘플들을 GROUP_SIZE개씩 묶어 각 묶음의 최빈값을 구하고, 그 최빈값들의 평균을 반환
float modeAverage(long* samples, int n) {
  if (n <= 0) return 0.0;
  if (n < GROUP_SIZE) return (float)findMode(samples, n);

  int numGroups = n / GROUP_SIZE;   // 나머지(끝자락)는 버림
  long sum = 0;
  for (int g = 0; g < numGroups; g++) {
    sum += findMode(samples + g * GROUP_SIZE, GROUP_SIZE);
  }
  return (float)sum / numGroups;
}

// 시리얼에 아무 입력이나 있으면 true (중단 트리거로 사용), 버퍼도 비움
bool checkAbort() {
  if (Serial.available() > 0) {
    while (Serial.available() > 0) Serial.read();
    return true;
  }
  return false;
}

void waitForStart() {
  Serial.println("READY. 아무 키나 입력하면 PWM 60~170 스윕을 시작합니다.");
  while (Serial.available() == 0) { /* 대기 */ }
  while (Serial.available() > 0) Serial.read();  // 트리거 입력 비우기
}

// 한 PWM 레벨을 5초간 유지하며 3~5초 구간을 샘플링, 대표값(avg21/avg20) 산출
void measureLevel(int pwmLevel, float* outAvg21, float* outAvg20) {
  static long samples21[MAX_SAMPLES];
  static long samples20[MAX_SAMPLES];
  int sampleCount = 0;

  analogWrite(PWM_PIN, pwmLevel);

  noInterrupts();
  encCount21 = 0;
  encCount20 = 0;
  interrupts();

  unsigned long levelStart = millis();
  unsigned long nextTick = levelStart;

  while (millis() - levelStart < HOLD_MS) {
    if (checkAbort()) { aborted = true; break; }

    unsigned long now = millis();
    if (now - nextTick >= SAMPLE_MS) {
      nextTick += SAMPLE_MS;

      noInterrupts();
      long c21 = encCount21; encCount21 = 0;
      long c20 = encCount20; encCount20 = 0;
      interrupts();

      unsigned long elapsed = now - levelStart;
      if (elapsed >= RECORD_START_MS && elapsed < RECORD_END_MS && sampleCount < MAX_SAMPLES) {
        samples21[sampleCount] = c21;
        samples20[sampleCount] = c20;
        sampleCount++;
      }
    }
  }

  *outAvg21 = modeAverage(samples21, sampleCount);
  *outAvg20 = modeAverage(samples20, sampleCount);
}

void printResults() {
  Serial.println("---- PWM -> 펄스 특성 (21번 20번, 3~5초 구간 9묶음 최빈값 평균) ----");
  for (int k = 0; k < resultCount; k++) {
    Serial.print("PWM");
    Serial.print(resultLevel[k]);
    Serial.print(" : ");
    Serial.print(resultAvg21[k], 2);
    Serial.print(" ");
    Serial.println(resultAvg20[k], 2);
  }
  Serial.println("---- 끝 ----");
}

void setup() {
  Serial.begin(115200);
  pinMode(HALL_PIN20, INPUT_PULLUP);
  pinMode(HALL_PIN21, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN20), encISR20, CHANGE);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN21), encISR21, CHANGE);
  pinMode(PWM_PIN, OUTPUT);
  analogWrite(PWM_PIN, 0);

  waitForStart();

  for (int lvl = PWM_START; lvl <= PWM_END; lvl += PWM_STEP) {
    Serial.print("PWM=");
    Serial.print(lvl);
    Serial.println(" 측정 중...");

    float avg21, avg20;
    measureLevel(lvl, &avg21, &avg20);

    resultLevel[resultCount] = lvl;
    resultAvg21[resultCount] = avg21;
    resultAvg20[resultCount] = avg20;
    resultCount++;

    if (aborted) {
      Serial.println("중단됨 (입력 감지).");
      break;
    }
  }

  analogWrite(PWM_PIN, 0);
  printResults();
}

void loop() {
  // 스윕은 setup()에서 1회 실행. 끝나면 아무 것도 하지 않음.
}
