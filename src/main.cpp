/*
 * Pixel Update Receiver — PlatformIO version
 * ============================================
 * Supports LILYGO T-Display (ESP32) and T-Display S3 Touch (ESP32-S3).
 *
 * Board-specific settings (display size, pins, backlight) are supplied via
 * build_flags in platformio.ini — nothing to change here.
 *
 * WiFi credentials are also set in platformio.ini (WIFI_SSID / WIFI_PASSWORD).
 *
 * Protocol v2 (little-endian):
 *   Pixel header : 'PXUP' (4 B) + version (1 B, 0x02) + frame_id (u32) + count (u16)
 *   Pixel body   : count × { x (u8), y (u8), color (u16 RGB565) }
 *
 *   Run header   : 'PXUR' (4 B) + version (1 B, 0x01) + frame_id (u32) + count (u16)
 *   Run body     : count × { y (u8), x0 (u8), length (u8), color (u16 RGB565) }
 */

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFi.h>
#include <esp_heap_caps.h>

// ── Display dimensions (set via build_flags, defaults for T-Display) ────────
#ifndef DISPLAY_WIDTH
#define DISPLAY_WIDTH 135
#endif
#ifndef DISPLAY_HEIGHT
#define DISPLAY_HEIGHT 240
#endif

// ── Backlight pin (set via build_flags) ─────────────────────────────────────
#ifndef BACKLIGHT_PIN
#define BACKLIGHT_PIN 4
#endif

// ── WiFi credentials (set via build_flags in platformio.ini) ────────────────
#ifndef WIFI_SSID
#define WIFI_SSID "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

// ── TFT MADCTL register for RGB/BGR swap ────────────────────────────────────
#ifndef TFT_MADCTL
#define TFT_MADCTL 0x36
#endif
#define TFT_MADCTL_RGB 0x00
#define TFT_MADCTL_BGR 0x08

TFT_eSPI tft = TFT_eSPI();

// WiFi credentials
const char *ssid     = WIFI_SSID;
const char *password = WIFI_PASSWORD;

// Network
WiFiServer server(8090);
WiFiClient client;

// Protocol constants
const uint8_t MAGIC[4]     = {'P', 'X', 'U', 'P'};
const uint8_t PROTO_VERSION = 0x02;
const size_t  HEADER_SIZE   = 11;  // MAGIC(4) + version(1) + frame_id(4) + count(2)

const uint8_t MAGIC_RUN[4] = {'P', 'X', 'U', 'R'};
const uint8_t RUN_VERSION   = 0x01;
const size_t  RUN_HEADER_SIZE = 11;

// Colour configuration (adjust if colours appear swapped on your panel)
bool swapBytesSetting = false;
bool useBgrSetting    = true;

// Stats
unsigned long frameCount      = 0;
unsigned long lastStats       = 0;
unsigned long updatesApplied  = 0;
uint32_t      lastFrameId     = 0;

// ── Pixel / run buffer ─────────────────────────────────────────────────────
struct PixelUpdate {
  uint8_t  x;
  uint8_t  y;
  uint8_t  len;     // used by run packets
  uint16_t color;
};

PixelUpdate *updateBuffer   = nullptr;
uint32_t     bufferCapacity = 0;
bool         dmaEnabled     = false;

bool ensureUpdateBuffer(uint32_t needed) {
  if (needed <= bufferCapacity && updateBuffer != nullptr) return true;

  PixelUpdate *tmp = (PixelUpdate *)ps_malloc(needed * sizeof(PixelUpdate));
  if (!tmp) tmp = (PixelUpdate *)malloc(needed * sizeof(PixelUpdate));
  if (!tmp) {
    Serial.println("Failed to allocate update buffer");
    return false;
  }
  if (updateBuffer) free(updateBuffer);
  updateBuffer   = tmp;
  bufferCapacity = needed;
  return true;
}

// ── Helpers ─────────────────────────────────────────────────────────────────
bool readExactly(WiFiClient &c, uint8_t *dst, size_t len) {
  size_t got = 0;
  while (got < len && c.connected()) {
    int chunk = c.read(dst + got, len - got);
    if (chunk > 0) got += chunk;
    else           delay(1);
  }
  return got == len;
}

void applyColorConfig() {
  tft.setSwapBytes(swapBytesSetting);
  tft.writecommand(TFT_MADCTL);
  tft.writedata(useBgrSetting ? TFT_MADCTL_BGR : TFT_MADCTL_RGB);
}

void showWaitingScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 20);
  tft.setTextSize(2);
  tft.println("Pixel RX");

  tft.setCursor(10, 50);
  tft.setTextSize(1);
  tft.println("IP Address:");

  tft.setCursor(10, 70);
  tft.setTextSize(2);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.println(WiFi.localIP().toString());

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(10, 100);
  tft.setTextSize(1);
  tft.println("Waiting for");
  tft.setCursor(10, 115);
  tft.println("connection...");
}

// ── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Pixel Update Receiver (PlatformIO) ===");
  Serial.printf("Display: %d x %d\n", DISPLAY_WIDTH, DISPLAY_HEIGHT);

  // ── Power-on pin (T-Display S3 needs GPIO 15 HIGH to enable LCD) ──
#ifdef POWER_ON_PIN
  pinMode(POWER_ON_PIN, OUTPUT);
  digitalWrite(POWER_ON_PIN, HIGH);
#endif

  // ── Backlight ──
  pinMode(BACKLIGHT_PIN, OUTPUT);
  digitalWrite(BACKLIGHT_PIN, HIGH);

  // ── TFT init ──
  tft.init();
#ifndef TFT_PARALLEL_8_BIT
  dmaEnabled = tft.initDMA();
#endif
  tft.setRotation(0);  // portrait
  applyColorConfig();
  tft.fillScreen(TFT_BLACK);

  // ── WiFi ──
  Serial.printf("Connecting to WiFi: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(250);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi connection failed!");
    tft.fillScreen(TFT_RED);
    tft.setTextColor(TFT_WHITE, TFT_RED);
    tft.setCursor(10, 50);
    tft.setTextSize(2);
    tft.println("WiFi FAILED!");
    while (true) delay(1000);
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  showWaitingScreen();

  server.begin();
  server.setNoDelay(true);
  Serial.println("Server listening on port 8090");
}

// ── Client handler ──────────────────────────────────────────────────────────
bool handleClient() {
  // Accept new client
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("Client connected");
      client.setNoDelay(true);
      client.setTimeout(50);
      frameCount     = 0;
      updatesApplied = 0;
      tft.fillScreen(TFT_BLACK);
    }
  }

  if (!client || !client.connected()) return false;

  // Wait for at least a full header
  if (client.available() < 11) return true;

  // Read magic bytes to determine packet type
  uint8_t magicBuf[4];
  if (!readExactly(client, magicBuf, 4)) { client.stop(); return false; }

  bool isPixel = (memcmp(magicBuf, MAGIC,     4) == 0);
  bool isRun   = (memcmp(magicBuf, MAGIC_RUN, 4) == 0);

  if (!isPixel && !isRun) {
    Serial.println("Bad magic; flushing stream");
    client.stop();
    return false;
  }

  // ── Pixel packet ──────────────────────────────────────────────────────
  if (isPixel) {
    uint8_t rest[HEADER_SIZE - 4];
    if (!readExactly(client, rest, sizeof(rest))) {
      Serial.println("Failed to read pixel header; dropping client");
      client.stop();
      return false;
    }
    if (rest[0] != PROTO_VERSION) {
      Serial.printf("Unsupported pixel version: 0x%02X\n", rest[0]);
      client.stop();
      return false;
    }

    uint32_t frameId = (uint32_t)rest[1]       | ((uint32_t)rest[2] << 8)
                     | ((uint32_t)rest[3] << 16) | ((uint32_t)rest[4] << 24);
    uint16_t count   = rest[5] | (rest[6] << 8);

    if (count == 0) { frameCount++; lastFrameId = frameId; return true; }
    if (count > (DISPLAY_WIDTH * DISPLAY_HEIGHT)) {
      Serial.printf("Update count too large: %u\n", count);
      client.stop();
      return false;
    }

    if (!ensureUpdateBuffer(count)) { client.stop(); return false; }

    uint8_t entry[4];
    for (uint16_t i = 0; i < count; i++) {
      if (!readExactly(client, entry, 4)) {
        Serial.println("Stream ended mid-frame; dropping client");
        client.stop();
        return false;
      }
      updateBuffer[i].x     = entry[0];
      updateBuffer[i].y     = entry[1];
      updateBuffer[i].color = entry[2] | (entry[3] << 8);
    }

    tft.startWrite();
    for (uint16_t i = 0; i < count; i++) {
      uint8_t x = updateBuffer[i].x;
      uint8_t y = updateBuffer[i].y;
      if (x < DISPLAY_WIDTH && y < DISPLAY_HEIGHT) {
        tft.setAddrWindow(x, y, 1, 1);
        tft.writeColor(updateBuffer[i].color, 1);
        updatesApplied++;
      }
    }
    tft.endWrite();

    frameCount++;
    lastFrameId = frameId;

    unsigned long now = millis();
    if (now - lastStats > 2000) {
      Serial.printf("Frames: %lu (frameId %u) | Updates: %lu\n",
                     frameCount, lastFrameId, updatesApplied);
      lastStats = now;
    }
    return true;
  }

  // ── Run packet ────────────────────────────────────────────────────────
  uint8_t rest[RUN_HEADER_SIZE - 4];
  if (!readExactly(client, rest, sizeof(rest))) {
    Serial.println("Failed to read run header; dropping client");
    client.stop();
    return false;
  }
  if (rest[0] != RUN_VERSION) {
    Serial.printf("Unsupported run version: 0x%02X\n", rest[0]);
    client.stop();
    return false;
  }

  uint32_t frameId = (uint32_t)rest[1]       | ((uint32_t)rest[2] << 8)
                   | ((uint32_t)rest[3] << 16) | ((uint32_t)rest[4] << 24);
  uint16_t count   = rest[5] | (rest[6] << 8);

  if (count == 0) { frameCount++; lastFrameId = frameId; return true; }
  if (count > (DISPLAY_WIDTH * DISPLAY_HEIGHT)) {
    Serial.printf("Run count too large: %u\n", count);
    client.stop();
    return false;
  }

  if (!ensureUpdateBuffer(count)) { client.stop(); return false; }

  // Each run entry: y(1) + x0(1) + length(1) + color(2) = 5 bytes
  uint8_t entry[5];
  for (uint16_t i = 0; i < count; i++) {
    if (!readExactly(client, entry, 5)) {
      Serial.println("Stream ended mid-run frame; dropping client");
      client.stop();
      return false;
    }
    updateBuffer[i].y     = entry[0];
    updateBuffer[i].x     = entry[1];
    updateBuffer[i].len   = entry[2];
    updateBuffer[i].color = entry[3] | (entry[4] << 8);
  }

  tft.startWrite();
  for (uint16_t i = 0; i < count; i++) {
    uint8_t x0     = updateBuffer[i].x;
    uint8_t y      = updateBuffer[i].y;
    uint8_t runLen = updateBuffer[i].len;
    if (x0 < DISPLAY_WIDTH && y < DISPLAY_HEIGHT && runLen > 0
        && (x0 + runLen) <= DISPLAY_WIDTH) {
      tft.setAddrWindow(x0, y, runLen, 1);
      if (dmaEnabled) tft.pushBlock(updateBuffer[i].color, runLen);
      else            tft.writeColor(updateBuffer[i].color, runLen);
      updatesApplied += runLen;
    }
  }
  tft.endWrite();

  frameCount++;
  lastFrameId = frameId;

  unsigned long now = millis();
  if (now - lastStats > 2000) {
    Serial.printf("Frames: %lu (frameId %u) | Updates: %lu\n",
                   frameCount, lastFrameId, updatesApplied);
    lastStats = now;
  }
  return true;
}

// ── Main loop ───────────────────────────────────────────────────────────────
void loop() {
  handleClient();
  if (client && !client.connected()) {
    Serial.println("Client disconnected");
    showWaitingScreen();
  }
  delay(1);
}
