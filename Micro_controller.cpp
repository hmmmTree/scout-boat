#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char*    AP_SSID  = "BoatControl";
const char*    AP_PASS  = "boat12345";  
const uint16_t UDP_PORT = 4210;

WiFiUDP udp;
char packetBuf[64];

Servo leftSide;
Servo rightSide;
Servo winch;
const int leftSidePin  = 18;
const int rightSidePin = 5;
const int winchPin     = 19;

int targetL = 90, targetR = 90, targetW = 90;   
int curL = 90, curR = 90, curW = 90;           
const int RAMP_STEP = 1;                        
const unsigned long RAMP_INTERVAL = 20;         
unsigned long lastRampMs = 0;

unsigned long lastPacketMs = 0;
const unsigned long FAILSAFE_MS = 750;   
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

void onWiFiEvent(WiFiEvent_t event) {
  if (event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    Serial.println(">>> Computer connected to BoatControl!");
  } else if (event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    Serial.println(">>> Computer disconnected.");
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 
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
  winch.attach(winchPin);       

  hardStop();
  delay(3000);

  WiFi.mode(WIFI_AP);
  WiFi.onEvent(onWiFiEvent);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("Access point started. Join WiFi network: ");
  Serial.println(AP_SSID);
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.softAPIP());  

  udp.begin(UDP_PORT);
  lastPacketMs = millis();
}

void loop() {
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

      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.print("OK");
      udp.endPacket();
    }
  }

  if (millis() - lastRampMs >= RAMP_INTERVAL) {
    lastRampMs = millis();
    curL = stepToward(curL, targetL, RAMP_STEP);
    curR = stepToward(curR, targetR, RAMP_STEP);
    curW = stepToward(curW, targetW, RAMP_STEP);
    leftSide.write(curL);
    rightSide.write(curR);
    winch.write(curW);
  }

  if (!stopped && (millis() - lastPacketMs > FAILSAFE_MS)) {
    hardStop();
    stopped = true;
    Serial.println("FAILSAFE: no commands for 3s - motors stopped");
  }
}
