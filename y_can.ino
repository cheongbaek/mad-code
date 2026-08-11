#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>

// =====================================================================
// EZkontrol B48-1000 CAN 로거 + PC 시리얼 브리지
// 기준 문서: EZkontrol CANBUS MCU to METER V1.1 20230727 (Golden Motor 공식)
//
// 두 가지를 동시에 한다.
//  (1) SD카드에 CSV 기록 : CAN에서 받는 필드를 전부 담는다(16열).
//      양식은 ksae_canlogging.ino / CANLOG.CSV 와 완전히 동일하다.
//  (2) PC로 시리얼 전송  : 대시보드에 필요한 5개 필드만 100ms마다 보낸다.
//      PC측 y_can.py가 받아 실시간 표시 + 같은 16열 양식으로 CSV 기록
//      (시리얼로 나가지 않는 열은 공란).
//
// 즉 SD는 '전량 기록', 시리얼은 '실시간 관측'으로 역할이 나뉜다.
// USB를 안 꽂아도 SD 기록은 정상 동작한다(메가는 while(!Serial)에서 안 멈춤).
// =====================================================================

#define CAN_CS   53
#define CAN_INT  2
#define SD_CS    4     // SD 모듈 CS핀. 실제 배선한 핀 번호로 맞출 것

MCP_CAN CAN0(CAN_CS);

const unsigned long SERIAL_INTERVAL_MS = 100;   // PC 대시보드용 (가볍다)
const unsigned long SD_INTERVAL_MS     = 500;   // SD 기록용. flush가 수~수십ms 블로킹하므로
                                                // 시리얼과 같은 100ms로 올리면 CAN 프레임을 놓친다

unsigned long lastSerial = 0;
unsigned long lastSd     = 0;

File logFile;
bool sdReady = false;

// 로그 파일명은 1.csv, 2.csv, ... 로 부팅마다 하나씩 올라간다
const unsigned int LOG_MAX_INDEX = 999;   // 이 개수를 넘으면 마지막 번호를 재사용
char logName[13];                         // "999.csv" + NUL (SD는 8.3 이름만 확실히 먹는다)

// ---- Message I (0x180117EF) : Voltage / Current / Speed ----
float battVoltage  = 0;
float battCurrent  = 0;
float phaseCurrent = 0;
int   motorSpeed   = 0;

// ---- Message II (0x180217EF) : Temp / Status / Error ----
int  controllerTemp = 0;
int  motorTemp      = 0;
byte accelPct       = 0;    // Byte2 : Accelerator Opening (0~100 %)

// Byte3 : STATUS (Gear / Brake / Operation Mode / DC Contactor)
byte gear          = 0;     // bit2-0 : 0=NO,1=R,2=N,3=D1,4=D2,5=D3,6=S,7=P
bool brakeOn       = false;  // bit3
byte opMode        = 0;      // bit6-4
bool dcContactorOn = false;  // bit7

// Byte4~6 : ERROR
byte err1 = 0, err2 = 0, err3 = 0;
byte lifeCounter = 0;        // Byte7

// ---- 변화 감지용 (바뀔 때만 '#' 이벤트 줄을 시리얼에 남긴다) ----
byte lastGear = 0xFF, lastOpMode = 0xFF;
byte lastErr1 = 0xFF, lastErr2 = 0xFF, lastErr3 = 0xFF;
bool lastBrake = false, lastDcContactor = false;

const char* gearName(byte g);
const char* opModeName(byte m);
void pickLogName();
void logToSD();

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(CAN_INT, INPUT);

  // MCP2515와 SD가 SPI 버스를 공유하므로, 초기화 전에 양쪽 CS를 HIGH(비활성)로 고정한다.
  // CS가 뜬 채로 있으면 SD가 MISO를 잡아 CAN 수신이 깨질 수 있다.
  pinMode(CAN_CS, OUTPUT); digitalWrite(CAN_CS, HIGH);
  pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS, HIGH);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    // '#' 로 시작하는 줄은 y_can.py가 무시하므로, 데이터 형식을 깨지 않고 알릴 수 있다
    unsigned long lastWarn = 0;
    while (1)
    {
      if (millis() - lastWarn >= 1000)
      {
        lastWarn = millis();
        Serial.println("# MCP2515 Initialization Failed - check CS/SPI wiring");
      }
    }
  }

  CAN0.setMode(MCP_LISTENONLY);

  // ---- SD 카드 ----
  if (SD.begin(SD_CS))
  {
    pickLogName();                       // 부팅마다 새 파일. 이어붙이면 millis가 0으로
                                         // 되돌아가 시간축이 뒤로 뛴다
    logFile = SD.open(logName, FILE_WRITE);
    if (logFile)
    {
      logFile.println("millis,batteryVoltage_V,batteryCurrent_A,phaseCurrent_A,motorSpeed_rpm,"
                      "controllerTemp_C,motorTemp_C,accelPct,gear,brake,opMode,dcContactor,"
                      "err1_hex,err2_hex,err3_hex,lifeCounter");
      logFile.flush();
      sdReady = true;
      Serial.print("# SD logging to ");
      Serial.println(logName);
    }
    else
    {
      Serial.println("# SD file open failed - serial only");
    }
  }
  else
  {
    Serial.println("# SD Card Initialization Failed - serial only");
  }
}

// 안 쓰인 가장 낮은 번호를 고른다 (1.csv, 2.csv, ...).
// 아두이노에는 시계가 없어 날짜·시각으로 이름을 지을 수 없으므로 번호로 구분한다.
// LOG_MAX_INDEX개가 다 차면 마지막 것을 지우고 재사용한다.
void pickLogName()
{
  for (unsigned int i = 1; i <= LOG_MAX_INDEX; i++)
  {
    itoa(i, logName, 10);
    strcat(logName, ".csv");
    if (!SD.exists(logName)) return;
  }
  SD.remove(logName);   // 여기 오면 logName은 마지막 번호
}

void loop()
{
  // --- [수신부] ---
  if (!digitalRead(CAN_INT))
  {
    unsigned long rxId;
    byte len;
    byte buf[8];

    // 읽기 실패 시 buf는 쓰레기값이다. 그대로 디코드하면 잘못된 전압/전류가
    // 기록과 전력량 적분에 그대로 누적되므로 반드시 성공한 프레임만 쓴다.
    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK)
    {
      rxId &= 0x1FFFFFFF;

      // Message I : Voltage / Current / Speed
      if (rxId == 0x180117EF && len >= 8)
      {
        uint16_t rawVoltage      = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        uint16_t rawBusCurrent   = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
        uint16_t rawPhaseCurrent = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
        uint16_t rawSpeed        = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

        battVoltage  = rawVoltage * 0.1;
        battCurrent  = rawBusCurrent * 0.1 - 3200.0;
        phaseCurrent = rawPhaseCurrent * 0.1 - 3200.0;
        motorSpeed   = (int32_t)rawSpeed - 32000;
      }
      // Message II : Temp / Status / Error (METER V1.1 기준 오프셋)
      else if (rxId == 0x180217EF && len >= 8)
      {
        controllerTemp = buf[0] - 40;
        motorTemp      = buf[1] - 40;
        accelPct       = buf[2];

        byte statusByte = buf[3];
        gear          = statusByte & 0x07;
        brakeOn       = (statusByte >> 3) & 0x01;
        opMode        = (statusByte >> 4) & 0x07;
        dcContactorOn = (statusByte >> 7) & 0x01;

        err1 = buf[4];
        err2 = buf[5];
        err3 = buf[6];
        lifeCounter = buf[7];

        // 값이 바뀔 때만 이벤트 줄. '#' 접두라 y_can.py 파싱을 깨지 않는다
        if (gear != lastGear || brakeOn != lastBrake || opMode != lastOpMode ||
            dcContactorOn != lastDcContactor ||
            err1 != lastErr1 || err2 != lastErr2 || err3 != lastErr3)
        {
          Serial.print("# ");
          Serial.print(millis());
          Serial.print("ms CHANGED Gear=");
          Serial.print(gearName(gear));
          Serial.print(" Brake=");
          Serial.print(brakeOn ? "ON" : "OFF");
          Serial.print(" Mode=");
          Serial.print(opModeName(opMode));
          Serial.print(" DC=");
          Serial.print(dcContactorOn ? "ON" : "OFF");
          Serial.print(" err=0x"); Serial.print(err1, HEX);
          Serial.print("/0x");     Serial.print(err2, HEX);
          Serial.print("/0x");     Serial.println(err3, HEX);

          lastGear = gear; lastBrake = brakeOn; lastOpMode = opMode;
          lastDcContactor = dcContactorOn;
          lastErr1 = err1; lastErr2 = err2; lastErr3 = err3;
        }
      }
    }
  }

  // --- [출력부 1] PC 시리얼 : 대시보드용 5필드 ---
  if (millis() - lastSerial >= SERIAL_INTERVAL_MS)
  {
    lastSerial = millis();

    // 속도는 10으로 나눠 보낸다. 정밀 rpm이 아니라 회전 여부 확인 용도이고,
    // SD쪽에는 원본 rpm이 그대로 남으므로 정보가 사라지지는 않는다.
    Serial.print(motorSpeed / 10);
    Serial.print(',');
    Serial.print(battVoltage, 1);
    Serial.print(',');
    Serial.print(battCurrent, 1);
    Serial.print(',');
    Serial.print(phaseCurrent, 1);
    Serial.print(',');
    Serial.println(gear);
  }

  // --- [출력부 2] SD CSV : 전량 기록 ---
  if (millis() - lastSd >= SD_INTERVAL_MS)
  {
    lastSd = millis();
    logToSD();
  }
}

void logToSD()
{
  if (!sdReady) return;

  logFile.print(millis());                     logFile.print(',');
  logFile.print(battVoltage, 1);               logFile.print(',');
  logFile.print(battCurrent, 1);               logFile.print(',');
  logFile.print(phaseCurrent, 1);              logFile.print(',');
  logFile.print(motorSpeed);                   logFile.print(',');
  logFile.print(controllerTemp);               logFile.print(',');
  logFile.print(motorTemp);                    logFile.print(',');
  logFile.print(accelPct);                     logFile.print(',');
  logFile.print(gearName(gear));               logFile.print(',');
  logFile.print(brakeOn ? "ON" : "OFF");       logFile.print(',');
  logFile.print(opModeName(opMode));           logFile.print(',');
  logFile.print(dcContactorOn ? "ON" : "OFF"); logFile.print(',');
  logFile.print(err1, HEX);                    logFile.print(',');
  logFile.print(err2, HEX);                    logFile.print(',');
  logFile.print(err3, HEX);                    logFile.print(',');
  logFile.println(lifeCounter);

  logFile.flush();   // 전원이 갑자기 끊겨도 마지막 줄까지 남도록 매번 flush
}

const char* gearName(byte g)
{
  switch (g)
  {
    case 0: return "NO";
    case 1: return "R";
    case 2: return "N";
    case 3: return "D1";
    case 4: return "D2";
    case 5: return "D3";
    case 6: return "S";
    case 7: return "P";
    default: return "UNKNOWN";
  }
}

const char* opModeName(byte m)
{
  switch (m)
  {
    case 0: return "Torque(추정)";
    case 1: return "Speed(추정)";
    case 2: return "Cruise";
    case 3: return "EBS";
    case 4: return "Hold";
    default: return "UNKNOWN";
  }
}
