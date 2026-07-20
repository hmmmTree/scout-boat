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
 *
 * Ack: "OK" plain, or with sensors fitted:
 *   "OK T:<lat>,<lon>,<heading>,<sats>,<speed_ms>,<pitch>,<roll>"
 *
 * Sensors (both optional; firmware auto-detects at boot):
 *   u-blox M10 GPS on UART2      - GPS TX -> GPS_RX_PIN, GPS RX -> GPS_TX_PIN
 *   ICM-20948 9-axis IMU on I2C  - SDA/SCL per board below, 3.3V, addr 0x68/0x69
 *
 * Library Manager installs needed (once):
 *   "TinyGPSPlus" (Mikal Hart)
 *   "SparkFun 9DoF IMU Breakout - ICM 20948"
 * Or set USE_GPS / USE_IMU to 0 to compile without them.
 */

#define BOARD 0   // <-- currently: classic ESP32 dev board
                  //     (esptool identified the chip as plain ESP32)

#define USE_GPS 1
#define USE_IMU 1

// Compass tuning: mounting/declination correction added to heading, and
// hard-iron offsets (uT) from a figure-8 calibration. Zeros work to start.
const float HEADING_OFFSET_DEG = 0.0;
const float MAG_OFF_X = 0.0, MAG_OFF_Y = 0.0, MAG_OFF_Z = 0.0;

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
const int I2C_SDA = 5, I2C_SCL = 6;        // pads D4/D5
const int GPS_RX_PIN = 44, GPS_TX_PIN = 43; // pads D7/D6 (free: monitor is USB CDC)
#elif BOARD == 2
// ESP32-S3 devkit: GPIO 19/20 are the USB D-/D+ lines - never use them.
// (Also avoid strap pins 0/3/45/46.) Only the winch moves vs the classic
// ESP32 wiring: 19 -> 4.
const int leftSidePin  = 18;
const int rightSidePin = 5;
const int winchPin     = 4;
const int I2C_SDA = 8, I2C_SCL = 9;
const int GPS_RX_PIN = 16, GPS_TX_PIN = 17;
#else
// Classic ESP32 dev board. Do NOT use 2/3/4 here: GPIO3 is the serial RX pin.
const int leftSidePin  = 18;
const int rightSidePin = 5;
const int winchPin     = 19;
const int I2C_SDA = 21, I2C_SCL = 22;       // board's default I2C pins
const int GPS_RX_PIN = 16, GPS_TX_PIN = 17; // UART2
#endif

// ---- sensors ----
#if USE_GPS
#include <TinyGPSPlus.h>
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
bool gpsBaudSwitched = false;   // M10 modules ship at 9600 or 38400; autodetect
#endif
#if USE_IMU
#include <Wire.h>
#include <Preferences.h>
#include "ICM_20948.h"
ICM_20948_I2C imu;
bool imuOk = false;
unsigned long lastImuMs = 0;
Preferences prefs;
// live mag offsets: loaded from flash at boot (MAGCAL command refreshes
// them at runtime, no reflash needed). Fall back to the constants above.
float magOffX = 0, magOffY = 0, magOffZ = 0;
// runtime heading reference (HDGREF command: "the bow points north now")
float hdgOff = 0;
// staleness watchdog: a dying I2C joint must not freeze the heading
unsigned long imuDataMs = 0, imuRetryMs = 0;
bool imuStale = false;
float lastMx = 0, lastMy = 0, lastMz = 0;      // raw, for diagnostics
// MAGCAL state: collect raw min/max while the user rotates the boat
bool magCalRunning = false;
unsigned long magCalEndMs = 0, magCalNoteMs = 0;
float cMinX, cMaxX, cMinY, cMaxY, cMinZ, cMaxZ;
#endif
// latest readings for the telemetry ack
float telLat = 0, telLon = 0, telSpd = 0;
float telHdg = 0, telPitch = 0, telRoll = 0;
int   telSats = 0;
bool  telHaveFix = false, telHaveHdg = false;
unsigned long lastStatusMs = 0;   // 5s health print over serial

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

#if USE_GPS
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS: listening on UART2 @9600 (autoswitches to 38400)");
#endif
#if USE_IMU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  // scan the whole bus first - tells wiring problems from address problems
  Serial.print("I2C scan:");
  int i2cFound = 0;
  for (byte a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      i2cFound++;
    }
  }
  if (!i2cFound) Serial.print(" NOTHING - check wiring/solder joints");
  Serial.println();
  imu.begin(Wire, 1);                    // try addr 0x69 (AD0 high)
  if (imu.status != ICM_20948_Stat_Ok) imu.begin(Wire, 0);   // then 0x68
  imuOk = (imu.status == ICM_20948_Stat_Ok);
  Serial.println(imuOk ? "IMU: ICM-20948 online" : "IMU: not found (telemetry degrades)");
  prefs.begin("boatcal", false);
  magOffX = prefs.getFloat("mx", MAG_OFF_X);
  magOffY = prefs.getFloat("my", MAG_OFF_Y);
  magOffZ = prefs.getFloat("mz", MAG_OFF_Z);
  hdgOff  = prefs.getFloat("ho", HEADING_OFFSET_DEG);
  Serial.printf("MAG offsets: %.1f %.1f %.1f  hdg ref: %+.1f\n",
                magOffX, magOffY, magOffZ, hdgOff);
  Serial.println("(MAGCAL = figure-8 cal, HDGREF = bow-is-north, *CLR to clear)");
#endif
}

// Read sensors and refresh the tel* snapshot used by the ack.
void updateTelemetry() {
#if USE_GPS
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  // autodetect baud: no NMEA after 5s -> module is at 38400
  if (!gpsBaudSwitched && millis() > 5000 && gps.charsProcessed() < 10) {
    gpsSerial.updateBaudRate(38400);
    gpsBaudSwitched = true;
    Serial.println("GPS: no data at 9600, switched to 38400");
  }
  telHaveFix = gps.location.isValid() && gps.satellites.isValid()
               && gps.satellites.value() >= 4;
  if (telHaveFix) {
    telLat = gps.location.lat();
    telLon = gps.location.lng();
    telSats = gps.satellites.value();
    telSpd = gps.speed.isValid() ? gps.speed.mps() : 0.0f;
  }
#endif
#if USE_IMU
  if (imuOk && millis() - lastImuMs >= 50 && imu.dataReady()) {
    lastImuMs = millis();
    imu.getAGMT();
    float ax = imu.accX(), ay = imu.accY(), az = imu.accZ();
    lastMx = imu.magX(); lastMy = imu.magY(); lastMz = imu.magZ();

    if (magCalRunning) {
      cMinX = min(cMinX, lastMx); cMaxX = max(cMaxX, lastMx);
      cMinY = min(cMinY, lastMy); cMaxY = max(cMaxY, lastMy);
      cMinZ = min(cMinZ, lastMz); cMaxZ = max(cMaxZ, lastMz);
      if (millis() - magCalNoteMs > 3000) {
        magCalNoteMs = millis();
        Serial.printf("MAGCAL: %lus left  spread x=%.0f y=%.0f z=%.0f (want >40 each)\n",
                      (magCalEndMs - millis()) / 1000,
                      cMaxX - cMinX, cMaxY - cMinY, cMaxZ - cMinZ);
      }
      if (millis() >= magCalEndMs) {
        magCalRunning = false;
        magOffX = (cMinX + cMaxX) / 2;
        magOffY = (cMinY + cMaxY) / 2;
        magOffZ = (cMinZ + cMaxZ) / 2;
        prefs.putFloat("mx", magOffX);
        prefs.putFloat("my", magOffY);
        prefs.putFloat("mz", magOffZ);
        Serial.printf("MAGCAL DONE: offsets %.1f %.1f %.1f saved "
                      "(spread %.0f/%.0f/%.0f)\n",
                      magOffX, magOffY, magOffZ,
                      cMaxX - cMinX, cMaxY - cMinY, cMaxZ - cMinZ);
      }
    }

    // AK09916 axes vs accel axes on this chip: y and z are inverted
    float mx = lastMx - magOffX;
    float my = -(lastMy - magOffY);
    float mz = -(lastMz - magOffZ);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float roll  = atan2f(ay, az);
    float xh = mx * cosf(pitch) + mz * sinf(pitch);
    float yh = mx * sinf(roll) * sinf(pitch) + my * cosf(roll)
               - mz * sinf(roll) * cosf(pitch);
    float hdg = atan2f(-yh, xh) * 180.0f / PI + hdgOff;
    while (hdg < 0) hdg += 360.0f;
    while (hdg >= 360.0f) hdg -= 360.0f;
    telHdg = hdg;
    telPitch = pitch * 180.0f / PI;
    telRoll  = roll * 180.0f / PI;
    telHaveHdg = true;
    imuDataMs = millis();
    imuStale = false;
  }
  // no fresh IMU data for 2s: stop reporting the frozen heading and try
  // to bring the sensor back (loose wiring recovers on its own this way)
  if (imuOk && imuDataMs && millis() - imuDataMs > 2000) {
    imuStale = true;
    telHaveHdg = false;
    if (millis() - imuRetryMs > 5000) {
      imuRetryMs = millis();
      imu.begin(Wire, 1);
      if (imu.status != ICM_20948_Stat_Ok) imu.begin(Wire, 0);
      if (imu.status == ICM_20948_Stat_Ok) {
        Serial.println("IMU: recovered");
        imuDataMs = millis();
      }
    }
  }
#endif
#if USE_GPS
  // no IMU? fall back to GPS course-over-ground (valid while moving)
  if (!telHaveHdg && telHaveFix && gps.course.isValid()) {
    telHdg = gps.course.deg();
    telHaveHdg = true;
  }
#endif
}

void buildAck(char* out, size_t n) {
  if (telHaveFix || telHaveHdg) {
    snprintf(out, n, "OK T:%.6f,%.6f,%.1f,%d,%.2f,%.1f,%.1f",
             telHaveFix ? telLat : 0.0f, telHaveFix ? telLon : 0.0f,
             telHaveHdg ? telHdg : 0.0f, telHaveFix ? telSats : 0,
             telHaveFix ? telSpd : 0.0f, telPitch, telRoll);
  } else {
    snprintf(out, n, "OK");
  }
}

// Same protocol over WiFi UDP or the USB serial cable. Acks return on
// whichever transport the command arrived on.
void handleCommand(const char* buf, bool fromSerial) {
#if USE_IMU
  if (strcmp(buf, "MAGCAL") == 0) {
    magCalRunning = true;
    magCalEndMs = millis() + 40000;
    magCalNoteMs = 0;
    cMinX = cMinY = cMinZ = 1e9;
    cMaxX = cMaxY = cMaxZ = -1e9;
    Serial.println("MAGCAL: rotate the boat slowly through ALL directions "
                   "(figure-8, roll it too) for 40 seconds...");
    return;
  }
  if (strcmp(buf, "MAGCLR") == 0) {
    magOffX = magOffY = magOffZ = 0;
    prefs.remove("mx"); prefs.remove("my"); prefs.remove("mz");
    Serial.println("MAGCLR: offsets cleared");
    return;
  }
  if (strcmp(buf, "HDGREF") == 0) {
    // the bow points north RIGHT NOW: fold current heading into the offset
    if (telHaveHdg) {
      hdgOff = hdgOff - telHdg;
      while (hdgOff <= -180.0f) hdgOff += 360.0f;
      while (hdgOff > 180.0f) hdgOff -= 360.0f;
      prefs.putFloat("ho", hdgOff);
      Serial.printf("HDGREF: bow = north captured, offset now %+.1f (saved)\n", hdgOff);
    } else {
      Serial.println("HDGREF: no heading available yet");
    }
    return;
  }
  if (strcmp(buf, "HDGCLR") == 0) {
    hdgOff = HEADING_OFFSET_DEG;
    prefs.remove("ho");
    Serial.println("HDGCLR: heading reference cleared");
    return;
  }
#endif
  int l, r, w, en = 1;
  int n = sscanf(buf, "L:%d,R:%d,W:%d,E:%d", &l, &r, &w, &en);
  if (n < 3) return;
  if (n < 4) en = 1;                 // old senders without E: = enabled
  lastPacketMs = millis();

  if (en) {
    targetL = constrain(l, 0, 180);
    targetR = constrain(r, 0, 180);
    targetW = constrain(w, 0, 180);
    stopped = false;
  } else {
    hardStop();                      // disarmed: everything off NOW, no ramp
    stopped = true;
  }

  char ack[112];
  buildAck(ack, sizeof(ack));
  if (fromSerial) {
    Serial.println(ack);
  } else {
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.print(ack);
    udp.endPacket();
  }
}

char usbBuf[64];
uint8_t usbLen = 0;

void loop() {
  updateTelemetry();

  // ---- wired link: same commands arriving over USB serial ----
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbLen) {
        usbBuf[usbLen] = '\0';
        handleCommand(usbBuf, true);
        usbLen = 0;
      }
    } else if (usbLen < sizeof(usbBuf) - 1) {
      usbBuf[usbLen++] = c;
    } else {
      usbLen = 0;                    // garbage line too long: drop it
    }
  }

  // periodic health line so the serial monitor doubles as a sensor check
  if (millis() - lastStatusMs >= 5000) {
    lastStatusMs = millis();
    Serial.print("STATUS:");
#if USE_IMU
    if (imuOk && imuStale) {
      Serial.print(" imu=STALE (wiring? retrying)");
    } else if (imuOk) {
      Serial.printf(" imu=OK hdg=%.0f pitch=%.0f roll=%.0f rawmag=%.0f/%.0f/%.0f",
                    telHdg, telPitch, telRoll, lastMx, lastMy, lastMz);
    } else {
      Serial.print(" imu=MISSING");
    }
#endif
#if USE_GPS
    Serial.printf(" | gps_chars=%lu sentences=%lu sats=%d fix=%s",
                  (unsigned long)gps.charsProcessed(),
                  (unsigned long)gps.sentencesWithFix(),
                  gps.satellites.isValid() ? (int)gps.satellites.value() : -1,
                  telHaveFix ? "YES" : "no");
    if (gps.charsProcessed() < 10) Serial.print("  <- NO DATA: check TX->16 RX->17 wiring");
#endif
    Serial.println();
  }

  // ---- receive commands -> set TARGETS (don't write directly) ----
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuf, sizeof(packetBuf) - 1);
    if (len > 0) packetBuf[len] = '\0';

    handleCommand(packetBuf, false);
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
