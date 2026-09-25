/**********************************************************************
 * ESP32 NRF24 3-Anchor Receiver
 *
 * Board: ESP32 Dev Module
 * Library: RF24 by TMRh20
 *
 * VSPI bus:
 *   SCK=18, MISO=19, MOSI=23
 *   Anchor 0: CSN=5,  CE=4
 *   Anchor 1: CSN=17, CE=16
 *
 * HSPI bus:
 *   SCK=14, MISO=12, MOSI=13
 *   Anchor 2: CSN=15, CE=2
 *
 * TDM:
 *   Only one NRF24 is in RX listening mode at a time.
 *   Each anchor listens for 20 ms, then returns to power-down.
 *   A 10-20 us RX window is too short for NRF24 packet reception; the
 *   radio needs about 1.5 ms just to wake from power-down.
 *
 * Output: one JSON line per full anchor scan, read by Python GUI.
 **********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include <WiFi.h>
#include "esp_bt.h"
#include "esp_wifi.h"

#define REG_RF_CH       0x05
#define REG_CONFIG      0x00
#define REG_RPD         0x09
#define CMD_R_REGISTER  0x00
#define CMD_NOP         0xFF

SPIClass VSPI_BUS(VSPI);
SPIClass HSPI_BUS(HSPI);

static const uint8_t A0_CSN = 32;
static const uint8_t A0_CE  = 33;
static const uint8_t A1_CSN = 17;
static const uint8_t A1_CE  = 16;
static const uint8_t A2_CSN = 25;
static const uint8_t A2_CE  = 26;

static const uint8_t CSN_PINS[3] = {A0_CSN, A1_CSN, A2_CSN};
static const uint8_t CE_PINS[3]  = {A0_CE,  A1_CE,  A2_CE};
static const uint8_t BUS_ID[3]   = {0, 0, 1}; // 0=VSPI, 1=HSPI
static const char *NAMES[3] = {"A0", "A1", "A2"};

RF24 radio0(A0_CE, A0_CSN, 4000000);
RF24 radio1(A1_CE, A1_CSN, 4000000);
RF24 radio2(A2_CE, A2_CSN, 4000000);
RF24 *radios[3] = {&radio0, &radio1, &radio2};

static const uint8_t CHANNEL = 76;
static const uint8_t PIPE_ADDR[5] = {'R', 'D', 'R', 'B', 'C'};
static const uint32_t MAGIC_VAL = 0xBEACBEAC;

// Equilateral triangle, side length 50 cm, centroid at (0, 0).
static const float ANCHOR_X[3] = {  0.00f, -25.00f,  25.00f};
static const float ANCHOR_Y[3] = { 28.87f, -14.43f, -14.43f};

// RPD thresholds are environment-dependent. Calibrate these after first run.
static const float DIST_THRESH[4] = {
  35.0f,   // PA_MIN
  70.0f,   // PA_LOW
  140.0f,  // PA_HIGH
  280.0f   // PA_MAX
};

struct BeaconPacket {
  uint32_t magic;
  uint8_t pa_level;
  uint8_t seq;
  uint8_t beacon_id;
  uint8_t checksum;
};

uint16_t rpd_hit[3][4] = {{0}};
uint16_t pkt_total[3][4] = {{0}};

float distances[3] = {500.0f, 500.0f, 500.0f};
float solved_x = 0.0f;
float solved_y = 0.0f;
float smooth_x = 0.0f;
float smooth_y = 0.0f;
bool fix_valid = false;

static const uint8_t SMOOTH_LEN = 5;
float hist_x[SMOOTH_LEN] = {0};
float hist_y[SMOOTH_LEN] = {0};
uint8_t hist_ptr = 0;
uint8_t hist_count = 0;

int active_idx = -1;
uint32_t tdm_timer_us = 0;

// Keep this in microseconds so the duty cycle is explicit.
// 20000 us = 20 ms per anchor, 60 ms for a full 3-anchor scan.
static const uint32_t NRF_WAKE_US = 1500;
static const uint32_t TDM_LISTEN_US = 20000;

uint32_t total_packets = 0;
uint32_t total_solves = 0;
uint32_t valid_fixes = 0;
uint32_t boot_time_ms = 0;

SPIClass *busFor(uint8_t idx) {
  return BUS_ID[idx] == 0 ? &VSPI_BUS : &HSPI_BUS;
}

void quietEspRadios() {
  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_OFF);
  esp_wifi_stop();
  esp_bt_controller_disable();
}

uint8_t readReg(uint8_t idx, uint8_t reg) {
  SPIClass *bus = busFor(idx);
  for (uint8_t i = 0; i < 3; i++) digitalWrite(CSN_PINS[i], HIGH);

  bus->beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CSN_PINS[idx], LOW);
  bus->transfer(CMD_R_REGISTER | (reg & 0x1F));
  uint8_t val = bus->transfer(CMD_NOP);
  digitalWrite(CSN_PINS[idx], HIGH);
  bus->endTransaction();
  return val;
}

bool verifyChecksum(const BeaconPacket &pkt) {
  const uint8_t *b = reinterpret_cast<const uint8_t *>(&pkt);
  uint8_t cs = 0;
  for (uint8_t i = 0; i < 7; i++) cs ^= b[i];
  return cs == pkt.checksum;
}

bool initRadio(uint8_t idx) {
  RF24 *r = radios[idx];
  if (!r->begin(busFor(idx))) {
    Serial.printf("ERR,%s,begin_failed\n", NAMES[idx]);
    return false;
  }

  r->setPALevel(RF24_PA_MAX);
  r->setDataRate(RF24_250KBPS);
  r->setChannel(CHANNEL);
  r->setPayloadSize(sizeof(BeaconPacket));
  r->setAutoAck(false);
  r->disableDynamicPayloads();
  r->setCRCLength(RF24_CRC_16);
  r->openReadingPipe(1, PIPE_ADDR);
  r->flush_rx();
  r->powerDown();

  uint8_t cfg = readReg(idx, REG_CONFIG);
  uint8_t ch = readReg(idx, REG_RF_CH);
  Serial.printf("INIT,%s,CONFIG,0x%02X,CH,%u\n", NAMES[idx], cfg, ch);
  return cfg != 0x00 && cfg != 0xFF && ch == CHANNEL;
}

void selectAnchor(uint8_t idx) {
  if (active_idx >= 0) {
    radios[active_idx]->stopListening();
    radios[active_idx]->powerDown();
  }

  active_idx = idx;
  radios[active_idx]->powerUp();
  delayMicroseconds(NRF_WAKE_US);
  radios[active_idx]->flush_rx();
  radios[active_idx]->startListening();
}

void pollAnchor(uint8_t idx) {
  BeaconPacket pkt;
  while (radios[idx]->available()) {
    radios[idx]->read(&pkt, sizeof(pkt));
    if (pkt.magic != MAGIC_VAL) continue;
    if (!verifyChecksum(pkt)) continue;
    if (pkt.pa_level > 3) continue;

    uint8_t rpd = readReg(idx, REG_RPD) & 0x01;
    pkt_total[idx][pkt.pa_level]++;
    if (rpd) rpd_hit[idx][pkt.pa_level]++;
    total_packets++;

    Serial.printf("RAW,%s,PA,%u,RPD,%u,SEQ,%u\n",
                  NAMES[idx], pkt.pa_level, rpd, pkt.seq);
  }
}

void computeDistances() {
  for (uint8_t i = 0; i < 3; i++) {
    int best_pa = -1;

    for (uint8_t pa = 0; pa < 4; pa++) {
      if (pkt_total[i][pa] < 2) continue;
      float rate = static_cast<float>(rpd_hit[i][pa]) / pkt_total[i][pa];
      if (rate >= 0.50f) {
        best_pa = pa;
        break;
      }
    }

    if (best_pa < 0) {
      distances[i] = 500.0f;
    } else {
      float rate = static_cast<float>(rpd_hit[i][best_pa]) / pkt_total[i][best_pa];
      float d_min = best_pa > 0 ? DIST_THRESH[best_pa - 1] : 0.0f;
      float d_max = DIST_THRESH[best_pa];
      float interp = constrain((rate - 0.5f) * 2.0f, 0.0f, 1.0f);
      distances[i] = d_min + (d_max - d_min) * (1.0f - interp);
    }

    for (uint8_t pa = 0; pa < 4; pa++) {
      rpd_hit[i][pa] = 0;
      pkt_total[i][pa] = 0;
    }
  }
}

bool trilaterate() {
  if (distances[0] >= 490.0f || distances[1] >= 490.0f || distances[2] >= 490.0f) {
    return false;
  }

  float x0 = ANCHOR_X[0], y0 = ANCHOR_Y[0];
  float x1 = ANCHOR_X[1], y1 = ANCHOR_Y[1];
  float x2 = ANCHOR_X[2], y2 = ANCHOR_Y[2];
  float d0 = distances[0], d1 = distances[1], d2 = distances[2];

  float A00 = 2.0f * (x1 - x0);
  float A01 = 2.0f * (y1 - y0);
  float A10 = 2.0f * (x2 - x0);
  float A11 = 2.0f * (y2 - y0);
  float B0 = d0 * d0 - d1 * d1 + x1 * x1 - x0 * x0 + y1 * y1 - y0 * y0;
  float B1 = d0 * d0 - d2 * d2 + x2 * x2 - x0 * x0 + y2 * y2 - y0 * y0;
  float det = A00 * A11 - A01 * A10;

  if (fabsf(det) < 0.001f) return false;

  solved_x = (B0 * A11 - B1 * A01) / det;
  solved_y = (A00 * B1 - A10 * B0) / det;
  return fabsf(solved_x) <= 120.0f && fabsf(solved_y) <= 120.0f;
}

void smoothFix() {
  hist_x[hist_ptr] = solved_x;
  hist_y[hist_ptr] = solved_y;
  hist_ptr = (hist_ptr + 1) % SMOOTH_LEN;
  if (hist_count < SMOOTH_LEN) hist_count++;

  float sx = 0.0f, sy = 0.0f;
  for (uint8_t i = 0; i < hist_count; i++) {
    sx += hist_x[i];
    sy += hist_y[i];
  }
  smooth_x = sx / hist_count;
  smooth_y = sy / hist_count;
}

void sendJSON() {
  float fix_pct = total_solves > 0 ? 100.0f * valid_fixes / total_solves : 0.0f;
  Serial.printf(
    "{\"fix\":%s,\"x\":%.2f,\"y\":%.2f,\"sx\":%.2f,\"sy\":%.2f,"
    "\"d\":[%.2f,%.2f,%.2f],\"pkts\":%lu,\"solves\":%lu,"
    "\"fixes\":%lu,\"fix_pct\":%.1f,\"uptime\":%lu,\"t\":%lu}\n",
    fix_valid ? "true" : "false",
    solved_x, solved_y, smooth_x, smooth_y,
    distances[0], distances[1], distances[2],
    total_packets, total_solves, valid_fixes, fix_pct,
    millis() - boot_time_ms, millis()
  );
}

void setup() {
  Serial.begin(9600);
  delay(600);
  quietEspRadios();

  for (uint8_t i = 0; i < 3; i++) {
    pinMode(CSN_PINS[i], OUTPUT);
    pinMode(CE_PINS[i], OUTPUT);
    digitalWrite(CSN_PINS[i], HIGH);
    digitalWrite(CE_PINS[i], LOW);
  }

  VSPI_BUS.begin(18, 19, 23, A0_CSN);
  HSPI_BUS.begin(14, 12, 13, A2_CSN);

  bool ok = true;
  for (uint8_t i = 0; i < 3; i++) {
    if (!initRadio(i)) ok = false;
  }

  if (!ok) {
    Serial.println("FATAL,radio_init_failed");
    while (true) delay(1000);
  }

  boot_time_ms = millis();
  selectAnchor(0);
  tdm_timer_us = micros();
  Serial.println("READY,esp32_receiver");
}

void loop() {
  uint32_t now_us = micros();
  bool cycle_complete = false;
  Serial.println("READY,esp32_receiver");

  if ((uint32_t)(now_us - tdm_timer_us) >= TDM_LISTEN_US) {
    uint8_t next_idx = (active_idx + 1) % 3;
    cycle_complete = active_idx == 2 && next_idx == 0;
    selectAnchor(next_idx);
    tdm_timer_us = micros();
  }

  if (active_idx >= 0) {
    pollAnchor(active_idx);
  }

  if (cycle_complete) {
    computeDistances();
    fix_valid = trilaterate();
    if (fix_valid) {
      smoothFix();
      valid_fixes++;
    }
    total_solves++;
    sendJSON();
  }
}
