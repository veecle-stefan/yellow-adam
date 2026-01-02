#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include "drivetrain.h"
#include "captiveportal.h"
#include "statusJSON.h"

Axle axleF(UART_NUM_1, HWConfig::Pins::UART::Front::RX, HWConfig::Pins::UART::Front::TX);
Axle axleR(UART_NUM_2, HWConfig::Pins::UART::Rear::RX, HWConfig::Pins::UART::Rear::TX);
Lights lights;
DriveTrain* drive = NULL;

static const char* AP_SSID = "Adam";
static const char* AP_PASS = "opeladam2026"; // >= 8 chars recommended
static const char* hostname = "adam";
CaptivePortalWeb portal;
static CarStatus st;
static char json[256];

// Example "other code" that must keep running
static uint32_t lastBlinkMs = 0;
static bool ledState = false;

static void setupWiFiAP()
{
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);

  // Optional: pick a fixed channel to reduce flakiness in noisy environments
  const int channel = 6;
  const bool hidden = false;
  const int maxConn = 4;

  bool ok = WiFi.softAP(AP_SSID, AP_PASS, channel, hidden, maxConn);

  Serial.printf("[WiFi] softAP %s, SSID='%s', PASS='%s'\n",
                ok ? "started" : "FAILED",
                AP_SSID, AP_PASS);

  Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

static void setupOTA()
{
  // The hostname shows up in the Arduino IDE / platformio-ota list
  ArduinoOTA.setHostname(hostname);

  // Optional: set a password for OTA updates
  // ArduinoOTA.setPassword("ota-pass");
  // or ArduinoOTA.setPasswordHash("...");

  ArduinoOTA
    .onStart([]() {
      Serial.println("[OTA] Start");
      lights.SetOTA(true);
    })
    .onEnd([]() {
      Serial.println("\n[OTA] End");
      lights.SetOTA(false);
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("[OTA] Progress: %u%%\r", (progress * 100U) / total);
      lights.SetOTAprogress(progress * 100 / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("\n[OTA] Error[%u]: ", error);
      switch (error) {
        case OTA_AUTH_ERROR:    Serial.println("Auth Failed"); break;
        case OTA_BEGIN_ERROR:   Serial.println("Begin Failed"); break;
        case OTA_CONNECT_ERROR: Serial.println("Connect Failed"); break;
        case OTA_RECEIVE_ERROR: Serial.println("Receive Failed"); break;
        case OTA_END_ERROR:     Serial.println("End Failed"); break;
        default:                Serial.println("Unknown"); break;
      }
    });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready (AP mode)");

  portal.onWsMessage([](const String& msg){
    // Minimal: just echo
    // You’ll parse JSON here later and map to inputs/actions
    Serial.printf("[APP] got: %s\n", msg.c_str());
  });
  IPAddress apIp = WiFi.softAPIP();
  portal.begin(apIp, hostname);
}


void setup() {
  Serial.begin(115200);
  Serial.println("Adam VCU starting");

  setupWiFiAP();   // returns immediately; no waiting
  setupOTA();      // OTA works over the AP network

  drive = new DriveTrain(axleF, axleR, lights);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(10); // nothing to be done, all in background threads
  ArduinoOTA.handle();

  portal.loop();

  // Example: broadcast status occasionally
  static uint32_t last = 0;
  uint32_t currTime = millis();
  if (currTime - last > 100) {
    last = currTime;

    if (drive->val_accell.has_value()) st.t = *(drive->val_accell);
    if (drive->val_steer.has_value()) st.s = *(drive->val_steer);
    st.tq_fl = drive->sentTorqueFL;
    st.tq_fr = drive->sentTorqueFR;
    st.tq_rl = drive->sentTorqueRL;
    st.tq_rr = drive->sentTorqueRR;
    Axle::MotorStates front = drive->lastFrontFb;
    Axle::MotorStates rear = drive->lastRearFb;
    if (front.has_value()) {
      st.curr_fl = front->sample.currL_meas;
      st.curr_fr = front->sample.currR_meas;
      st.voltage_front = front->sample.batVoltage;
      st.temp_front = front->sample.boardTemp;
    }
    if (rear.has_value()) {
      st.curr_rl = rear->sample.currL_meas;
      st.curr_rr = rear->sample.currR_meas;
      st.voltage_rear = rear->sample.batVoltage;
      st.temp_rear = rear->sample.boardTemp;
    }

    size_t n = EncodeStatusJson(st, json, sizeof(json));
    if (n > 0) {
      portal.broadcastText(json);
    }
  }

}

