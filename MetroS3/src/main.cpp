#include <Arduino.h>
#include "ESPNowCam.h"

#include "lgfx_custom_ili9341_conf.hpp"
#include <LGFX_TFT_eSPI.hpp>

// pins for Metro ESP32S3
/*
#define TFT_CS 2
#define TFT_DC 3
#define TFT_MOSI 42
#define TFT_MISO 21
#define TFT_SCK 39
#define TFT_RST -1 // can set to -1 if tied to Arduino RESET pin
*/

// MAC addr of sender (Metro S3)
// 0x80, 0xB5, 0x4E, 0xCD, 0x29, 0x20

ESPNowCam radio;

static TFT_eSPI lcd;              // Instance of LGFX

// panel logical size
static const int W = 320;
static const int H = 240;

// display globals
int32_t dw = 320;
int32_t dh = 240;

// *** buffers ***
static const size_t JPG_MAX = 96 * 1024;
static uint8_t *jpg = nullptr;


// simple FPS/diag
static uint32_t last_ms = 0;
static uint32_t frames = 0;

static void onDataReady(uint32_t length) {
  if (!length || length > JPG_MAX) {
    Serial.printf("onDataReady: bad len=%u (max=%u)\n", (unsigned)length, (unsigned)JPG_MAX);
    return;
  }

  lcd.startWrite();
  // LovyanGFX decodes JPEG and pushes in tiles safely for ESP32-S3
  lcd.drawJpg(jpg, length, 0, 0);   // x=0, y=0
  lcd.endWrite();

  // crude FPS
  frames++;
  uint32_t now = millis();
  if (now - last_ms >= 1000) {
    Serial.printf("fps=%u, last_jpg_bytes=%u\n", frames, (unsigned)length);
    frames = 0;
    last_ms = now;
  }
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  lcd.init();
  lcd.invertDisplay(true);
  lcd.setRotation(0);
  lcd.setBrightness(128);
  lcd.setColorDepth(16);
  lcd.fillScreen(TFT_BLACK);

 // ---- Buffers ----
  jpg = (uint8_t*) malloc(JPG_MAX);
  if (!jpg) {
    Serial.println("jpg alloc failed");
    while (1) delay(1000);
  }


  // ---- ESPNOW ----
  radio.setRecvBuffer(jpg);             // receive compressed JPEG into 'jpg'
  radio.setRecvCallback(onDataReady);

  if (radio.init()) {
    Serial.println("ESPNow Init Success");
    lcd.setCursor(6, 6);
    lcd.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init FAILED");
    lcd.setCursor(6, 6);
    lcd.println("ESPNow Init FAILED");
  }
}

void loop(void) {
}