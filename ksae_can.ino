#include <SPI.h>
#include <mcp_can.h>

// =====================================================================
// EZkontrol B48-1000 CAN -> PC 시리얼 브리지
// 기준 문서: EZkontrol CANBUS MCU to METER V1.1 20230727 (Golden Motor 공식)
// SD카드 없이 필요한 필드만 추출해 시리얼로 전송한다. CSV 생성/가공은
// PC측 ksae_can.py가 담당 (ksae_canlogging.ino의 SD 로깅 역할을 대체).
// =====================================================================

#define CAN_CS   53
#define CAN_INT  2

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
  pinMode(CAN_CS, OUTPUT);
  digitalWrite(CAN_CS, HIGH);

  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
  {
    // 초기화 실패 시 배선/보드 확인 필요 (MCP2515 CS/SPI 결선)
    while (1);
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

    CAN0.readMsgBuf(&rxId, &len, buf);
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
