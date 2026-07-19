/*
 * Boat control receiver — Seeed Studio XIAO ESP32-S3
 * --------------------------------------------------
 * Arduino IDE setup:
 *   Tools > Board            : "XIAO_ESP32S3" (preferred) or "ESP32S3 Dev Module"
 *   Tools > USB CDC On Boot  : Enabled          (otherwise Serial prints go nowhere)
 *   Tools > Flash Size       : 8MB
 *
 * Protocol: UDP text packets "L:<0-180>,R:<0-180>,W:<0-180>" to port 4210.
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---- WiFi access point ----
const char*    AP_SSID  = "BoatControl";
const char*    AP_PASS  = "boat12345";   // must be at least 8 characters
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
char packetBuf[64];

// ---- Servos / ESCs ----
// XIAO ESP32-S3 only breaks out GPIO 1-9 and 43/44 (silkscreened D0-D10).
// GPIO 18/19/20 are NOT available here (19/20 are the USB D-/D+ lines).
// Raw GPIO numbers are used (not D-pin macros) so this compiles with either
// "XIAO_ESP32S3" or the generic "ESP32S3 Dev Module" board selected.
Servo leftSide;
Servo rightSide;
Servo winch;
const int leftSidePin  = 2;   // XIAO pad "D1"
const int rightSidePin = 3;   // XIAO pad "D2"
const int winchPin     = 4;   // XIAO pad "D3"

// ---- Throttle ramp (anti-brownout) ----
int targetL = 90, targetR = 90, targetW = 90;   // where we want to be (from packets)
int curL = 90, curR = 90, curW = 90;            // where the outputs actually are
const int RAMP_STEP = 1;                        // max change per tick (lower = gentler)
const unsigned long RAMP_INTERVAL = 20;         // ms between ramp ticks
unsigned long lastRampMs = 0;

// ---- Failsafe ----
unsigned long lastPacketMs = 0;
const unsigned long FAILSAFE_MS = 750;   // hold last command 750ms, then stop
bool stopped = true;

int stepToward(int cur, int target, int step) {
  if (cur < target) {
    cur += step;
    if (cur > target) cur = target;
  } else if (cur > target) {
    cur -= step;
    if (cur < target) cur = target;
  }
  return cur;
}

void hardStop() {
  targetL = targetR = targetW = 90;
  curL = curR = curW = 90;
  leftSide.write(90);
  rightSide.write(90);
  winch.write(90);
}

// Called automatically on WiFi events
void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    Serial.println(">>> Computer connected to BoatControl!");
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    Serial.println(">>> Computer disconnected.");
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // disable brownout reset (rides through brief dips)
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  leftSide.setPeriodHertz(50);
  leftSide.attach(leftSidePin, 1000, 2000);

  rightSide.setPeriodHertz(50);
  rightSide.attach(rightSidePin, 1000, 2000);

  winch.setPeriodHertz(50);
  winch.attach(winchPin);          // default pulse range (matches your test sketch)

  // Arm ESCs at neutral
  hardStop();
  delay(3000);

  // Start the access point
  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  WiFi.softAP(AP_SSID, AP_PASS);
  // The XIAO's regulator is small; full 20dBm TX bursts are a common brownout
  // trigger on battery power. 11dBm is still plenty for a few metres of range.
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.setSleep(false);            // keep UDP latency low and consistent
  Serial.print("Access point started. Join WiFi network: ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.softAPIP());   // usually 192.168.4.1

  udp.begin(UDP_PORT);
  lastPacketMs = millis();
}

void loop() {
  // ---- receive commands -> set TARGETS (don't write directly) ----
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuf, sizeof(packetBuf) - 1);
    if (len > 0) packetBuf[len] = '\0';

    int l, r, w;
    if (sscanf(packetBuf, "L:%d,R:%d,W:%d", &l, &r, &w) == 3) {
      targetL = constrain(l, 0, 180);
      targetR = constrain(r, 0, 180);
      targetW = constrain(w, 0, 180);

      lastPacketMs = millis();
      stopped = false;

      // Acknowledge so the PC knows the link is alive
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("OK");
      udp.endPacket();
    }
  }

  // ---- ramp the outputs toward the targets ----
  if (millis() - lastRampMs >= RAMP_INTERVAL) {
    lastRampMs = millis();
    curL = stepToward(curL, targetL, RAMP_STEP);
    curR = stepToward(curR, targetR, RAMP_STEP);
    curW = stepToward(curW, targetW, RAMP_STEP);
    leftSide.write(curL);
    rightSide.write(curR);
    winch.write(curW);
  }

  // ---- failsafe: after 3s of silence, snap to a stop ----
  if (!stopped && (millis() - lastPacketMs > FAILSAFE_MS)) {
    hardStop();
    stopped = true;
    Serial.print("FAILSAFE: no commands for ");
    Serial.print(FAILSAFE_MS);
    Serial.println("ms - motors stopped");
  }
}
