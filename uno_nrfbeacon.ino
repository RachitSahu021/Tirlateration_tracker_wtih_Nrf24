#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>

static const uint8_t PIN_CE  = 9;
static const uint8_t PIN_CSN = 10;

static const uint8_t CHANNEL = 76;
static const uint8_t PIPE_ADDR[5] = {'R', 'D', 'R', 'B', 'C'};
static const uint32_t MAGIC_VAL = 0xBEACBEAC;
static const uint8_t BEACON_ID = 0x01;

// Uses Uno hardware SPI: SCK=D13, MOSI=D11, MISO=D12.
RF24 radio(PIN_CE, PIN_CSN, 4000000);

struct BeaconPacket {
  uint32_t magic;
  uint8_t pa_level;   // 0=MIN, 1=LOW, 2=HIGH, 3=MAX
  uint8_t seq;
  uint8_t beacon_id;
  uint8_t checksum;
};

static_assert(sizeof(BeaconPacket) == 8, "Unexpected packet size");

uint8_t seq_no = 0;

uint8_t checksumPacket(const BeaconPacket &pkt) {
  const uint8_t *b = reinterpret_cast<const uint8_t *>(&pkt);
  uint8_t cs = 0;

  for (uint8_t i = 0; i < 7; i++) {
    cs ^= b[i];
  }
  return cs;
}

uint8_t paFromIndex(uint8_t pa) {
  switch (pa) {
    case 0: return RF24_PA_MIN;
    case 1: return RF24_PA_LOW;
    case 2: return RF24_PA_HIGH;
    default: return RF24_PA_MAX;
  }
}

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    Serial.println(F("NRF24 begin() failed. Check wiring and 3.3V supply."));
    while (true) {
      delay(500);
    }
  }

  radio.setChannel(CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(sizeof(BeaconPacket));
  radio.setAutoAck(false);
  radio.disableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  radio.openWritingPipe(PIPE_ADDR);
  radio.stopListening();

  Serial.println(F("Uno beacon ready."));
}

void loop() {
  for (uint8_t pa = 0; pa < 4; pa++) {
    radio.setPALevel(paFromIndex(pa));
    delayMicroseconds(200);

    BeaconPacket pkt;
    pkt.magic = MAGIC_VAL;
    pkt.pa_level = pa;
    pkt.seq = seq_no++;
    pkt.beacon_id = BEACON_ID;
    pkt.checksum = checksumPacket(pkt);

    radio.write(&pkt, sizeof(pkt));
    delay(3);
  }

  // Optional gap between complete PA sweeps.
  delay(20);
}