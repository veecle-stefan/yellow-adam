#include <Arduino.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "I2Ccontrol.h"
#include "motors.h"

static Motors::CurrLoopConfig tuning;

// Disable WiFi & BT to keep power + timing noise down
static void disableWifiBt() {
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true, true);
  esp_wifi_stop();
  btStop();   // safe even if BT not started
}

// === Callback implementations ===

void onModeCommand(ControlMode mode, int8_t amplitude) {
  // For now, same mode & amplitude to both sides:
  motors.setModeAndAmplitude(mode, amplitude, mode, amplitude);
}

void onAmpUpdate(int8_t ampL, int8_t ampR) {

}

void onParamUpdate(uint8_t paramId, float value) {
  switch (paramId) {
    case PARAM_CURR_P:      tuning.P    = value; break;
    case PARAM_CURR_I:      tuning.I    = value; break;
    case PARAM_CURR_LIMIT:  tuning.limitA  = value; break;
    case PARAM_CURR_LPF_TF: tuning.lpfTf_s  = value; break;
    case PARAM_MAX_CURRENT: tuning.maxCurrentA = value; break;
  }
  motors.setCurrentLoopParams(tuning);
}

void setup() {

  motors.resetSafe();
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println(F("Booting AdamECU (WiFi/BT OFF, I2C slave control)…"));

  disableWifiBt();

  // Init I2C control interface (adjust pins/address if needed)
  I2CMotorCommands.begin(
    I2C_ADDR_DEFAULT + ECU_ID,  // from I2CControl.h (0x10)
    I2C_SDA_DEFAULT,
    I2C_SCL_DEFAULT
  );

  I2CMotorCommands.setModeCommandHandler(onModeCommand);
  I2CMotorCommands.setParamUpdateHandler(onParamUpdate);
  I2CMotorCommands.setAmplitudeHandler(onAmpUpdate);

  motors.begin();
}

void loop() {
  // Let the I2C control layer pull pending updates out of the ISR
  // and invoke your callbacks in a safe context
  // TODO: use i2cControl.setStatus() for current and speed from motors.cpp
  I2CMotorCommands.update();

  // Your FOC control likely runs in a FreeRTOS task, so loop() stays light
  delay(1);
}