#include <SPI.h>
#include <mcp_can.h>
#include <SD.h>

#define CAN_CS   53
#define CAN_INT  2
#define SD_CS    4     // SD 모듈 CS핀. 실제 배선한 핀 번호로 바꿔주세요 (보통 4/8/9/10 중 하나)

MCP_CAN CAN0(CAN_CS);

// =====================================================================
// EZkontrol B481000 CAN Logger + SD Card CSV Logging
// 기준 문서: EZkontrol CANBUS MCU to METER V1.1 20230727 (Golden Motor 공식)
// =====================================================================

unsigned long lastPrint = 0;

// 로그 파일명은 1.csv, 2.csv, ... 로 부팅마다 하나씩 올라간다.
// 아두이노에는 시계가 없어 날짜·시각으로 이름을 지을 수 없으므로 번호로 구분한다.
const unsigned int LOG_MAX_INDEX = 999;   // 이 개수를 넘으면 마지막 번호를 재사용
char logName[13];                         // "999.csv" + NUL (SD는 8.3 이름만 확실히 먹는다)

File logFile;
bool sdReady = false;

// ---- Message I (0x180117EF) : Voltage / Current / Speed ----
float batteryVoltage = 0;
float batteryCurrent = 0;
float phaseCurrent   = 0;
int   motorSpeed     = 0;

// ---- Message II (0x180217EF) : Temp / Status / Error ----
int  controllerTemp = 0;
int  motorTemp      = 0;
byte accelPct        = 0;    // Byte2 : Accelerator Opening (0~100 %)

// Byte3 : STATUS (Gear / Brake / Operation Mode / DC Contactor)
byte gear          = 0;      // bit2-0 : 0=NO,1=R,2=N,3=D1,4=D2,5=D3,6=S,7=P
bool brakeOn       = false;  // bit3   : 0=No brake, 1=Brake
byte opMode        = 0;      // bit6-4 : 2=Cruise,3=EBS,4=Hold (0/1은 Torque/Speed로 추정)
bool dcContactorOn = false;  // bit7   : 0=OFF, 1=ON

// Byte4~6 : 진짜 ERROR 바이트
byte err1 = 0;  // Byte4 : Overcurrent ~ Motor Out of phase
byte err2 = 0;  // Byte5 : Motor Sensor ~ DC Contactor
byte err3 = 0;  // Byte6 : Power valve ~ Software

byte lifeCounter = 0;  // Byte7

// ---- 변화 감지용 (값이 바뀔 때만 이벤트 로그 출력, 11~12단계 검증용) ----
byte lastGear = 0xFF, lastOpMode = 0xFF;
byte lastErr1 = 0xFF, lastErr2 = 0xFF, lastErr3 = 0xFF;
bool lastBrake = false, lastDcContactor = false;

void printSnapshot();
void logToSD();
void pickLogName();
void printErrors(byte e1, byte e2, byte e3);
const char* gearName(byte g);
const char* opModeName(byte m);

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(CAN_INT, INPUT);

  Serial.println("==================================");
  Serial.println(" Golden Motor EZkontrol CAN Logger");
  Serial.println("==================================");

  // SPI 버스를 MCP2515와 SD가 공유하므로, 초기화 전에 서로의 CS를 HIGH(비활성)로 고정
  pinMode(CAN_CS, OUTPUT); digitalWrite(CAN_CS, HIGH);
  pinMode(SD_CS, OUTPUT);  digitalWrite(SD_CS, HIGH);

  byte result = CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);

  if (result != CAN_OK)
  {
    Serial.println("MCP2515 Initialization Failed.");
    while (1);
  }

  Serial.println("MCP2515 Initialization Success.");
  CAN0.setMode(MCP_LISTENONLY);

  // ---- SD 카드 초기화 ----
  if (SD.begin(SD_CS))
  {
    sdReady = true;
    pickLogName();   // 부팅마다 안 쓰인 새 번호. 한 파일에 이어붙이면 재부팅 때
                     // millis가 0으로 되돌아가 시간축이 뒤로 점프한다

    logFile = SD.open(logName, FILE_WRITE);
    if (logFile)
    {
      // 항상 새 파일이므로 헤더는 무조건 쓴다
      logFile.println("millis,batteryVoltage_V,batteryCurrent_A,phaseCurrent_A,motorSpeed_rpm,"
                       "controllerTemp_C,motorTemp_C,accelPct,gear,brake,opMode,dcContactor,"
                       "err1_hex,err2_hex,err3_hex,lifeCounter");
      logFile.flush();

      Serial.print("SD Card Initialization Success. Logging to ");
      Serial.println(logName);
    }
    else
    {
      sdReady = false;
      Serial.println("SD Card file open failed - logging to serial only.");
    }
  }
  else
  {
    Serial.println("SD Card Initialization Failed - logging to serial only.");
  }

  Serial.println("Waiting CAN Data...");
}

void loop()
{
  // --- [수신부] CAN 신호가 들어왔을 때 변수 업데이트 ---
  if (!digitalRead(CAN_INT))
  {
    unsigned long rxId;
    byte len;
    byte buf[8];

    CAN0.readMsgBuf(&rxId, &len, buf);
    rxId &= 0x1FFFFFFF;

    // Message I : Voltage / Current / Speed
    if (rxId == 0x180117EF)
    {
      uint16_t rawVoltage      = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      uint16_t rawBusCurrent   = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      uint16_t rawPhaseCurrent = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
      uint16_t rawSpeed        = (uint16_t)buf[6] | ((uint16_t)buf[7] << 8);

      batteryVoltage = rawVoltage * 0.1;
      batteryCurrent = rawBusCurrent * 0.1 - 3200.0;
      phaseCurrent   = rawPhaseCurrent * 0.1 - 3200.0;
      motorSpeed     = (int32_t)rawSpeed - 32000;
    }
    // Message II : Temp / Status / Error (METER V1.1 기준 오프셋)
    else if (rxId == 0x180217EF)
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

      // ---- 값이 바뀔 때만 타임스탬프와 함께 이벤트 로그 (시리얼 전용) ----
      if (gear != lastGear || brakeOn != lastBrake || opMode != lastOpMode ||
          dcContactorOn != lastDcContactor ||
          err1 != lastErr1 || err2 != lastErr2 || err3 != lastErr3)
      {
        Serial.print(millis());
        Serial.print(" ms CHANGED -> Gear=");
        Serial.print(gearName(gear));
        Serial.print(" Brake=");
        Serial.print(brakeOn ? "ON" : "OFF");
        Serial.print(" Mode=");
        Serial.print(opModeName(opMode));
        Serial.print(" DC=");
        Serial.print(dcContactorOn ? "ON" : "OFF");
        Serial.print(" err1=0x"); Serial.print(err1, HEX);
        Serial.print(" err2=0x"); Serial.print(err2, HEX);
        Serial.print(" err3=0x"); Serial.println(err3, HEX);

        lastGear = gear; lastBrake = brakeOn; lastOpMode = opMode;
        lastDcContactor = dcContactorOn;
        lastErr1 = err1; lastErr2 = err2; lastErr3 = err3;
      }
    }
  }

  // --- [출력부] 500ms 주기: 시리얼 스냅샷 + SD CSV 한 줄 기록 ---
  if (millis() - lastPrint >= 500)
  {
    lastPrint = millis();
    printSnapshot();
    logToSD();
  }
}

// 안 쓰인 가장 낮은 번호를 고른다 (1.csv, 2.csv, ...).
// LOG_MAX_INDEX개가 다 차면 마지막 번호를 지우고 재사용한다.
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

void logToSD()
{
  if (!sdReady || !logFile) return;

  logFile.print(millis());               logFile.print(",");
  logFile.print(batteryVoltage, 1);       logFile.print(",");
  logFile.print(batteryCurrent, 1);       logFile.print(",");
  logFile.print(phaseCurrent, 1);         logFile.print(",");
  logFile.print(motorSpeed);              logFile.print(",");
  logFile.print(controllerTemp);          logFile.print(",");
  logFile.print(motorTemp);               logFile.print(",");
  logFile.print(accelPct);                logFile.print(",");
  logFile.print(gearName(gear));          logFile.print(",");
  logFile.print(brakeOn ? "ON" : "OFF");  logFile.print(",");
  logFile.print(opModeName(opMode));      logFile.print(",");
  logFile.print(dcContactorOn ? "ON" : "OFF"); logFile.print(",");
  logFile.print(err1, HEX);               logFile.print(",");
  logFile.print(err2, HEX);               logFile.print(",");
  logFile.print(err3, HEX);               logFile.print(",");
  logFile.println(lifeCounter);

  logFile.flush();  // 전원이 갑자기 끊겨도 마지막 줄까지 저장되도록 매번 flush
}

void printSnapshot()
{
  Serial.println();
  Serial.println("======================================");

  Serial.print("Battery Voltage : ");
  Serial.print(batteryVoltage, 1);
  Serial.println(" V");

  Serial.print("Battery Current : ");
  Serial.print(batteryCurrent, 1);
  Serial.println(" A");

  Serial.print("Phase Current   : ");
  Serial.print(phaseCurrent, 1);
  Serial.println(" A");

  Serial.print("Motor Speed     : ");
  Serial.print(motorSpeed);
  Serial.println(" rpm");

  Serial.println();

  Serial.print("Controller Temp : ");
  Serial.print(controllerTemp);
  Serial.println(" C");

  Serial.print("Motor Temp      : ");
  Serial.print(motorTemp);
  Serial.println(" C");

  Serial.print("Accel Opening   : ");
  Serial.print(accelPct);
  Serial.println(" %");

  Serial.println();
  Serial.println("----- STATUS (Byte3) -----");

  Serial.print("Gear            : ");
  Serial.println(gearName(gear));

  Serial.print("Brake           : ");
  Serial.println(brakeOn ? "ON" : "OFF");

  Serial.print("Operation Mode  : ");
  Serial.println(opModeName(opMode));

  Serial.print("DC Contactor    : ");
  Serial.println(dcContactorOn ? "ON" : "OFF");

  Serial.println();
  Serial.println("===== RAW ERROR BYTES (Byte4~6) =====");

  Serial.print("Error Byte4 : 0x");
  if (err1 < 16) Serial.print("0");
  Serial.print(err1, HEX);
  Serial.print("   BIN : ");
  Serial.println(err1, BIN);

  Serial.print("Error Byte5 : 0x");
  if (err2 < 16) Serial.print("0");
  Serial.print(err2, HEX);
  Serial.print("   BIN : ");
  Serial.println(err2, BIN);

  Serial.print("Error Byte6 : 0x");
  if (err3 < 16) Serial.print("0");
  Serial.print(err3, HEX);
  Serial.print("   BIN : ");
  Serial.println(err3, BIN);

  Serial.println("======================================");

  printErrors(err1, err2, err3);

  Serial.println();
  Serial.print("Life Counter : ");
  Serial.println(lifeCounter);

  Serial.print("SD Logging   : ");
  if (sdReady)
  {
    Serial.print("ON (");
    Serial.print(logName);
    Serial.println(")");
  }
  else
  {
    Serial.println("OFF");
  }

  Serial.println("======================================");
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

void printErrors(byte e1, byte e2, byte e3)
{
  if ((e1 == 0) && (e2 == 0) && (e3 == 0))
  {
    Serial.println("Fault           : NONE");
    return;
  }

  Serial.println("Fault Detected");

  if (e1 & 0x01) Serial.println("- Over Current");
  if (e1 & 0x02) Serial.println("- Over Load");
  if (e1 & 0x04) Serial.println("- Over Voltage");
  if (e1 & 0x08) Serial.println("- Under Voltage");
  if (e1 & 0x10) Serial.println("- Controller Overheat");
  if (e1 & 0x20) Serial.println("- Motor Overheat");
  if (e1 & 0x40) Serial.println("- Motor Stalled");
  if (e1 & 0x80) Serial.println("- Motor Out of Phase");

  if (e2 & 0x01) Serial.println("- Motor Sensor");
  if (e2 & 0x02) Serial.println("- Motor AUX Sensor");
  if (e2 & 0x04) Serial.println("- Encoder Misaligned");
  if (e2 & 0x08) Serial.println("- Anti-Runaway Engaged");
  if (e2 & 0x10) Serial.println("- Main Accelerator");
  if (e2 & 0x20) Serial.println("- AUX Accelerator");
  if (e2 & 0x40) Serial.println("- Pre-charge");
  if (e2 & 0x80) Serial.println("- DC Contactor");

  if (e3 & 0x01) Serial.println("- Power Valve");
  if (e3 & 0x02) Serial.println("- Current Sensor");
  if (e3 & 0x04) Serial.println("- Auto Tune");
  if (e3 & 0x08) Serial.println("- RS485");
  if (e3 & 0x10) Serial.println("- CAN");
  if (e3 & 0x20) Serial.println("- Software");
}
