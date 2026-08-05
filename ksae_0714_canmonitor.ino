/*
  Golden Motor EZkontrol CAN Monitor (VCU role, read-only check)
  ----------------------------------------------------------------------
  Board   : Arduino Mega 2560
  CAN     : HW-184 module (MCP2515 + TJA1050), 250 kbps
  Purpose : SD 로깅을 붙이기 전, CAN에서 Phase Current / Speed(rpm)가
            제대로 수신되는지만 확인하는 최소 버전.

  Safety note:
  - 하트비트의 Command byte는 항상 0x00 (bit0=0 -> HALTED),
    Target Phase Current / Target Speed도 항상 0으로 보냄.
    즉 이 스케치는 컨트롤러에 구동 명령을 절대 내리지 않음 -
    핸드셰이크만 유지해서 텔레메트리(전류/속도)만 받아본다.

  Library required: MCP_CAN_lib by coryjfowler
    https://github.com/coryjfowler/MCP_CAN_lib
*/

#include <SPI.h>
#include <mcp_can.h>

// ---------------- Pin mapping (Arduino Mega 2560) ----------------
const uint8_t CAN_CS_PIN  = 9;
const uint8_t CAN_INT_PIN = 2;

MCP_CAN CAN0(CAN_CS_PIN);

// ---------------- EZkontrol MCU<->VCU protocol IDs ----------------
const uint32_t ID_MCU_TO_VCU_MSG1 = 0x1801D0EF; // MCU -> VCU, Message I (+ 0x55 handshake request)
const uint32_t ID_VCU_TO_MCU_CMD  = 0x0C01EFD0; // VCU -> MCU, control command (+ 0xAA handshake ack)

// ---------------- Physical conversion constants ----------------
const float PHASE_CURRENT_RES  = 0.1f;    // A/bit
const float PHASE_CURRENT_OFFS = -3200.0f;
const float SPEED_RES  = 1.0f;            // rpm/bit
const float SPEED_OFFS = -32000.0f;

// ---------------- Link state machine ----------------
enum LinkState { WAIT_HANDSHAKE, CONNECTED };
LinkState linkState = WAIT_HANDSHAKE;

uint8_t lifeSignal = 0;
unsigned long lastHeartbeatMs = 0;
unsigned long lastRxMs = 0;

const unsigned long HEARTBEAT_INTERVAL_MS = 50;
const unsigned long LINK_TIMEOUT_MS = 500; // 이 시간 동안 MCU 프레임이 없으면 링크 끊긴 것으로 보고 재핸드셰이크

void setup() {
  Serial.begin(115200);
  while (!Serial) { }

  Serial.println(F("Initializing MCP2515..."));
  while (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN_INT_PIN, INPUT);

  Serial.println(F("CAN ready. Waiting for MCU handshake (0x55 on 0x1801D0EF)..."));
}

void sendHandshakeAck() {
  uint8_t ackData[8];
  memset(ackData, 0xAA, 8);
  CAN0.sendMsgBuf(ID_VCU_TO_MCU_CMD, 1, 8, ackData); // ext=1
}

// HALTED 하트비트: Command=0x00, Target current/speed=0. 구동 명령 없음.
void sendHaltedHeartbeat() {
  uint8_t buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  buf[7] = lifeSignal++;
  CAN0.sendMsgBuf(ID_VCU_TO_MCU_CMD, 1, 8, buf);
}

bool isHandshakePattern(uint8_t *data, uint8_t len) {
  for (uint8_t i = 0; i < len && i < 8; i++) {
    if (data[i] != 0x55) return false;
  }
  return true;
}

void handleMessage1(uint8_t *data, uint8_t len) {
  if (len < 8) return;

  if (isHandshakePattern(data, len)) {
    sendHandshakeAck();
    if (linkState != CONNECTED) {
      Serial.println(F("Handshake complete. Reading telemetry..."));
    }
    linkState = CONNECTED;
    lifeSignal = 0;
    lastHeartbeatMs = millis();
    return;
  }

  if (linkState != CONNECTED) return; // 핸드셰이크 전 텔레메트리는 무시

  // Message I: Byte0-1 BusV, Byte2-3 BusI, Byte4-5 PhaseI, Byte6-7 Speed
  uint16_t phaseRaw = data[4] | (data[5] << 8);
  uint16_t speedRaw = data[6] | (data[7] << 8);

  float phaseCurrentA = phaseRaw * PHASE_CURRENT_RES + PHASE_CURRENT_OFFS;
  float speedRpm       = speedRaw * SPEED_RES + SPEED_OFFS;

  Serial.print(F("Phase current: "));
  Serial.print(phaseCurrentA, 1);
  Serial.print(F(" A   Speed: "));
  Serial.print(speedRpm, 0);
  Serial.println(F(" rpm"));
}

void loop() {
  // 1) 수신
  if (!digitalRead(CAN_INT_PIN)) {
    long unsigned int rxIdRaw;
    uint8_t len = 0;
    uint8_t rxBuf[8];

    if (CAN0.readMsgBuf(&rxIdRaw, &len, rxBuf) == CAN_OK) {
      uint32_t extId = rxIdRaw & 0x1FFFFFFF;
      lastRxMs = millis();

      if (extId == ID_MCU_TO_VCU_MSG1) {
        handleMessage1(rxBuf, len);
      }
    }
  }

  // 2) 하트비트 (연결된 동안만, 50ms 주기)
  if (linkState == CONNECTED && (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS)) {
    sendHaltedHeartbeat();
    lastHeartbeatMs = millis();
  }

  // 3) 링크 워치독
  if (linkState == CONNECTED && (millis() - lastRxMs > LINK_TIMEOUT_MS)) {
    Serial.println(F("No MCU frames received recently. Waiting for handshake again."));
    linkState = WAIT_HANDSHAKE;
  }
}
