/*
 * Boat control receiver
 * ---------------------
 * Pick your board with BOARD below:
 *   0 = classic ESP32 dev board        (Arduino board: "ESP32 Dev Module")
 *   1 = Seeed XIAO ESP32-S3            (Arduino board: "XIAO_ESP32S3", USB CDC On Boot: Enabled)
 *   2 = ESP32-S3 dev board / DevKitC   (Arduino board: "ESP32S3 Dev Module";
 *                                       if flashing via the port labeled "USB",
 *                                       set USB CDC On Boot: Enabled)
 *
 * Protocol: UDP text packets "L:<0-180>,R:<0-180>,W:<0-180>,E:<0|1>" to
 * port 4210. E is the enable flag: E:0 = disarmed -> instant hard stop.
 * Packets without the E field are treated as enabled (old senders).
 */

#define BOARD 2   // <-- currently: ESP32-S3 dev board

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// ---- WiFi access point ----
const char*    AP_SSID    = "BoatControl";
const char*    AP_PASS    = "boat12345";   // must be at least 8 characters
const uint16_t UDP_PORT   = 4210;
const int      AP_CHANNEL = 6;   // move off crowded default ch1 (lots of nearby APs)

WiFiUDP udp;
char packetBuf[64];

// ---- Servos / ESCs ----
Servo leftSide;
Servo rightSide;
Servo winch;
#if BOARD == 1
// XIAO only breaks out GPIO 1-9 and 43/44 (pads D0-D10); 18/19 don't exist here.
const int leftSidePin  = 2;   // XIAO pad "D1"
const int rightSidePin = 3;   // XIAO pad "D2"
const int winchPin     = 4;   // XIAO pad "D3"
#elif BOARD == 2
// ESP32-S3 devkit: GPIO 19/20 are the USB D-/D+ lines - never use them.
// (Also avoid strap pins 0/3/45/46.) Only the winch moves vs the classic
// ESP32 wiring: 19 -> 4.
const int leftSidePin  = 18;
const int rightSidePin = 5;
const int winchPin     = 4;
#else
// Classic ESP32 dev board. Do NOT use 2/3/4 here: GPIO3 is the serial RX pin.
const int leftSidePin  = 18;
const int rightSidePin = 5;
const int winchPin     = 19;
#endif

// ---- Throttle ramp (anti-brownout) ----
// XIAO's small regulator needs a gentle ramp; a dev board can move much faster.
// Stick-to-full-throttle time = 90/RAMP_STEP * RAMP_INTERVAL ms.
int targetL = 90, targetR = 90, targetW = 90;   // where we want to be (from packets)
int curL = 90, curR = 90, curW = 90;            // where the outputs actually are
#if BOARD == 1
const int RAMP_STEP = 1;                        // XIAO: full throttle in ~1.8s
#else
const int RAMP_STEP = 4;                        // devkits: full throttle in ~0.45s
#endif
const unsigned long RAMP_INTERVAL = 20;         // ms between ramp ticks
unsigned long lastRampMs = 0;

// ---- Failsafe ----
unsigned long lastPacketMs = 0;
const unsigned long FAILSAFE_MS = 300;   // no packets for 300ms -> instant stop
bool stopped = true;

// Ramp only applies to power INCREASES. Anything moving toward neutral
// (disarm, stick release, direction change) snaps immediately - reducing
// power is always safe and should never lag.
int stepToward(int cur, int target, int step) {
  if (cur > 90 && target < cur) return (target >= 90) ? target : 90;
  if (cur < 90 && target > cur) return (target <= 90) ? target : 90;
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

  // Begin holding neutral for the ESCs; the PWM runs in hardware, so the
  // 3s arming window can overlap with the WiFi startup below.
  hardStop();

  // Start the access point immediately so it's visible as soon as possible
  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL);
#if BOARD == 1
  // The XIAO's regulator is small; full 20dBm TX bursts are a common brownout
  // trigger on battery power. 11dBm is still plenty for a few metres of range.
  WiFi.setTxPower(WIFI_POWER_11dBm);
#endif
  WiFi.setSleep(false);            // keep UDP latency low and consistent

  // Finish the ESC arming window (neutral has been held since hardStop above)
  delay(3000);
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

    int l, r, w, en = 1;
    int n = sscanf(packetBuf, "L:%d,R:%d,W:%d,E:%d", &l, &r, &w, &en);
    if (n >= 3) {
      if (n < 4) en = 1;               // old senders without E: = enabled
      lastPacketMs = millis();

      if (en) {
        targetL = constrain(l, 0, 180);
        targetR = constrain(r, 0, 180);
        targetW = constrain(w, 0, 180);
        stopped = false;
      } else {
        hardStop();                    // disarmed: everything off NOW, no ramp
        stopped = true;
      }

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

  // ---- failsafe: link gone -> instant stop ----
  if (!stopped && (millis() - lastPacketMs > FAILSAFE_MS)) {
    hardStop();
    stopped = true;
    Serial.print("FAILSAFE: no commands for ");
    Serial.print(FAILSAFE_MS);
    Serial.println("ms - motors stopped");
  }
}
