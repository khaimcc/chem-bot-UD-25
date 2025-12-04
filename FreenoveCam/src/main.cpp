/**************************************************
 * ESP32-CAM → M5Core2 (ESP-NOW) sender
 * Freenove ESP32 WROVER camera (OV2640) explicit pin map
 * - Sensor JPEG (no software frame2jpg)
 * - DRAM framebuffers, conservative settings
 **************************************************/
#include <Arduino.h>
#include "esp_camera.h"
#include <ESPNowCam.h>

// ===== controller side ESP32 MAC (Metro S3) =====
static const uint8_t MAC_RECV[6] = {0x80, 0xB5, 0x4E, 0xCD, 0x29, 0x20};

// ===== Robot side ESP32 MAC =====
// {0xC0, 0x49, 0xEF, 0xE0, 0xDF, 0xB4};

// ===== Radio (ESP-NOW) =====
ESPNowCam radio;


// Throttle to avoid saturating ESPNOW while you bring things up
static uint32_t lastSend = 0;
static const uint32_t SEND_INTERVAL_MS = 120; // ~8 fps to start (safe)

// ===== frame settings here =====
static int CAM_JPEG_QUALITY = 45;        // lower = better quality (and larger frames)
static framesize_t CAM_FRAMESIZE = FRAMESIZE_HVGA; // 320x240
#define CAM_FB_COUNT     2               // double buffering
#define CAM_XCLK_HZ      10000000        // 10 MHz is conservative and stable

// transmitter pin for TX2, talks to arduino D8
static const int ESP_TX2 = 14;

// controls state from controller
struct controlState {
  int8_t dir;   // -2=left, -1=down, 0=center, 1=up, 2=right
  uint8_t button;
};

controlState latestControl = {0, 0};
volatile bool controlUpdated = false;

volatile bool sequenceStarted = false;
// time since last send
unsigned long lastSentMs = 0;
// maximum ms between sends
const unsigned long HEARTBEAT_MS = 200; // 200ms for 5Hz
// ==== test LED ====
#define builtInLED 2  // on GPIO pin 2 on WROVER is IO2 builtin LED

// ===== Freenove ESP32-WROVER (OV5640) pin map =====
static camera_config_t camCfg() {
  camera_config_t c = {};
  c.ledc_timer   = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;

  // Data lines (Y2..Y9)
  c.pin_d0   = 4;    // Y2
  c.pin_d1   = 5;    // Y3  
  c.pin_d2   = 18;   // Y4
  c.pin_d3   = 19;   // Y5
  c.pin_d4   = 36;   // Y6
  c.pin_d5   = 39;   // Y7
  c.pin_d6   = 34;   // Y8
  c.pin_d7   = 35;   // Y9

  // Control/clock
  c.pin_xclk   = 21;
  c.pin_pclk   = 22;
  c.pin_vsync  = 25;
  c.pin_href   = 23;
  c.pin_sscb_sda = 26; // SIOD
  c.pin_sscb_scl = 27; // SIOC

  // Sensor power/reset (Freenove WROVER typically none)
  c.pin_pwdn  = -1;
  c.pin_reset = -1;

  c.xclk_freq_hz = CAM_XCLK_HZ;
  c.pixel_format = PIXFORMAT_JPEG;      // sensor does JPEG

  if(psramFound()){
    c.fb_location = CAMERA_FB_IN_PSRAM;    // try PSRAM if available
    c.frame_size = FRAMESIZE_QQVGA;   // VGA if PSRAM
    c.jpeg_quality = 20;          
    Serial.println("PSRAM found" );
  } else {
    c.fb_location = CAMERA_FB_IN_DRAM;  // fallback to DRAM
    c.frame_size = FRAMESIZE_HVGA;
    c.jpeg_quality = 20;
    Serial.println("Using DRAM" );
  }
  CAM_JPEG_QUALITY = c.jpeg_quality;
  CAM_FRAMESIZE = c.frame_size;
  c.fb_count     = CAM_FB_COUNT;
  c.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  return c;
}

static void initCameraOrHalt() {
  camera_config_t cfg = camCfg();

esp_err_t err = esp_camera_init(&cfg);
if (err != ESP_OK) {
  Serial.printf("Camera init failed (0x%X)\n", (int)err);
  while (true) { delay(1000); }
}

sensor_t *s = esp_camera_sensor_get();
if (s) {
  s->set_quality(s, CAM_JPEG_QUALITY);  // keep JPEG quality
  s->set_framesize(s, CAM_FRAMESIZE);   // keep resolution
  s->set_contrast(s, 0);                // increase contrast
  // s->set_whitebal(s, 0);             // disable white balance
  // s->set_gain_ctrl(s, 0);            // disable gain control
  // s->set_exposure_ctrl(s, 0);        // disable exposure control
}
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  Serial.printf("Raw len=%d: ", len);
  for (int i = 0; i < len; i++) {
    Serial.printf("%02X ", incomingData[i]);
  }
  Serial.println();

  if (len != sizeof(controlState)) {
    Serial.printf("Received invalid control data size: %d\n", len);
    return;
  }

  controlState tmp;
  memcpy(&tmp, incomingData, sizeof(controlState));

  noInterrupts();
  latestControl = tmp;       // single atomic assignment, no reset
  controlUpdated = true;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nESPNowCam Freenove sender (explicit pin map)");

  Serial2.begin(9600, SERIAL_8N1, -1, ESP_TX2); // TX2 only

  // camera first (avoids radio contention during DMA setup)
  initCameraOrHalt();

  // radio next
  // set target as S3 MAC addr
  radio.setTarget(MAC_RECV);
  radio.init();

  esp_now_register_recv_cb(OnDataRecv);

  // Print PSRAM info (just informational)
  if (psramFound()) {
    size_t mb = esp_spiram_get_size() / 1048576;
    Serial.printf("PSRAM: %u MB\n", (unsigned)mb);
  }
}

// fb size if set to 15000B for the receiver. started work here to determine frame size 
// and adjust buffer size accordingly. couln't log fb size in line 144 just yet.
void loop() {
  controlState cs;

  int count = 0; // counter var to log fb size every 5 sends
  uint32_t now = millis();
  if (now - lastSend < SEND_INTERVAL_MS) {
    delay(1); // yield
    return;
  }
  lastSend = now;

  // Capture already-JPEG frame
  delay(0); // tiny yield before heavy call
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    // If this happens often, reduce framesize or raise SEND_INTERVAL_MS
    Serial.println("Capture failed");
    delay(5);
    return;
  }

  // Send via ESP-NOW (ESPNowCam handles fragmentation)
  radio.sendData(fb->buf, fb->len);

  if ((++count % 5) == 0) // log every 5 frames
    Serial.printf("Sent %u bytes\n", (unsigned)fb->len);

  // Release buffer + friendly yields
  esp_camera_fb_return(fb);
  delay(0);

  // guarantee we process latest control input, not 
  // dir from packet N and start from packet N+1
  noInterrupts();
  memcpy(&cs, (const void*)&latestControl, sizeof(cs));
  controlUpdated = false;
  interrupts();

  bool heartbeat = (millis() - lastSentMs) >= HEARTBEAT_MS; // 5 Hz

  // if heartbeat timeout, send update
  if (heartbeat) {
    // serial print the current state
    lastSentMs = millis();

    int d = (int)cs.dir;
    int b = (int)cs.button;

    Serial.printf("%d,%d\n", d, b);
    Serial2.printf("%d,%d\n", d, b);
  }
}