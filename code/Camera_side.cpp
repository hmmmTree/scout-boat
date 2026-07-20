/*
 * Boat camera — Seeed Studio XIAO ESP32-S3 Sense (OV2640)
 * -------------------------------------------------------
 * Joins the boat's "BoatControl" WiFi as a client with a FIXED IP and
 * serves an MJPEG stream the driver station displays:
 *
 *     http://192.168.4.10:81/stream
 *
 * Arduino IDE setup:
 *   Tools > Board        : "XIAO_ESP32S3"
 *   Tools > PSRAM        : "OPI PSRAM"        <-- required for the camera
 *   Tools > USB CDC On Boot : Enabled
 *
 * No extra libraries needed (esp_camera ships with the ESP32 core).
 */

#include <WiFi.h>
#include "esp_camera.h"

const char* WIFI_SSID = "BoatControl";
const char* WIFI_PASS = "boat12345";

// Static IP so the driver station always finds the stream at the same place
IPAddress CAM_IP(192, 168, 4, 10);
IPAddress GATEWAY(192, 168, 4, 1);
IPAddress SUBNET(255, 255, 255, 0);

WiFiServer server(81);

// XIAO ESP32-S3 Sense camera pin map (OV2640)
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_VGA;     // 640x480: good balance for WiFi
  c.jpeg_quality = 14;                // lower number = better quality, more bytes
  c.fb_count     = 2;
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(CAM_IP, GATEWAY, SUBNET);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Joining BoatControl");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(400);
    Serial.print(".");
  }
  Serial.println(WiFi.status() == WL_CONNECTED
                 ? "\nConnected. Stream: http://192.168.4.10:81/stream"
                 : "\nNot connected yet (will keep retrying)");
}

void setup() {
  Serial.begin(115200);
  delay(300);
  if (!cameraInit()) {
    Serial.println("CAMERA INIT FAILED - check PSRAM setting (OPI PSRAM) "
                   "and that this is the Sense board with camera attached");
  } else {
    Serial.println("Camera OK");
  }
  connectWiFi();
  server.begin();
}

void streamTo(WiFiClient& client) {
  client.print("HTTP/1.1 200 OK\r\n"
               "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
               "Cache-Control: no-cache\r\n\r\n");
  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { delay(50); continue; }
    client.printf("--frame\r\nContent-Type: image/jpeg\r\n"
                  "Content-Length: %u\r\n\r\n", fb->len);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    delay(40);   // ~20 fps cap; keeps WiFi airtime for the control link
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      connectWiFi();
    }
    return;
  }
  WiFiClient client = server.accept();
  if (client) {
    // serve the stream to whoever asked, regardless of path
    Serial.println("Viewer connected");
    unsigned long t0 = millis();
    while (client.connected() && !client.available() && millis() - t0 < 1000) delay(1);
    while (client.available()) client.read();   // discard request
    streamTo(client);
    client.stop();
    Serial.println("Viewer disconnected");
  }
}
