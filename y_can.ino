#include <SPI.h>
#include <mcp_can.h>

// =====================================================================
// EZkontrol B48-1000 CAN -> PC 시리얼 브리지
// 기준 문서: EZkontrol CANBUS MCU to METER V1.1 20230727 (Golden Motor 공식)
// SD카드 없이 필요한 필드만 추출해 시리얼로 전송한다. CSV 생성/가공은
// PC측 y_can.py가 담당 (ksae_canlogging.ino의 SD 로깅 역할을 대체).
// =====================================================================

#define CAN_CS   53
#define CAN_INT  2
#define SD_CS    4     // SD는 쓰지 않지만, 모듈이 SPI에 물려 있으면 CS를 HIGH로 묶어둬야 한다

MCP_CAN CAN0(CAN_CS);

const unsigned long PRINT_INTERVAL_MS = 100;
unsigned long lastPrint = 0;

// ---- Message I (0x180117EF) : Voltage / Current / Speed ----
float battVoltage  = 0;
float battCurrent  = 0;
float phaseCurrent = 0;
int   motorSpeed   = 0;

// ---- Message II (0x180217EF) : STATUS Byte3 중 기어(bit2-0)만 사용 ----
byte gear = 0;

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  pinMode(CAN_INT, INPUT);

  // MCP2515와 SD가 SPI 버스를 공유하므로, CAN 초기화 전에 양쪽 CS를 HIGH(비활성)로 고정한다.
  // 이 코드는 SD를 쓰지 않지만 SD 모듈이 물려 있는 채로 CS가 뜨면 SD가 MISO를 잡아
  // CAN 수신이 깨질 수 있어, 배선이 남아 있는 한 반드시 눌러둬야 한다.
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
}

void loop()
{
  if (!digitalRead(CAN_INT))
  {
    unsigned long rxId;
    byte len;
    byte buf[8];

    // 읽기 실패 시 buf는 쓰레기값이다. 그대로 디코드하면 잘못된 전압/전류가
    // 파이썬 쪽 전력량 적분에 그대로 누적되므로 반드시 성공한 프레임만 쓴다.
    if (CAN0.readMsgBuf(&rxId, &len, buf) == CAN_OK)
    {
      rxId &= 0x1FFFFFFF;

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
      else if (rxId == 0x180217EF && len >= 8)
      {
        gear = buf[3] & 0x07;  // STATUS 바이트의 나머지 비트(브레이크/모드/컨택터)는 폐기
      }
    }
  }

  if (millis() - lastPrint >= PRINT_INTERVAL_MS)
  {
    lastPrint = millis();

    Serial.print(motorSpeed);
    Serial.print(',');
    Serial.print(battVoltage, 1);
    Serial.print(',');
    Serial.print(battCurrent, 1);
    Serial.print(',');
    Serial.print(phaseCurrent, 1);
    Serial.print(',');
    Serial.println(gear);
  }
}
