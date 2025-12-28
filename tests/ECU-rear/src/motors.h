#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>
#include "I2Ccontrol.h"   // for ControlMode
#include "motorparams.h"

class Motors {
public:
  // Shared state between tasks (protected by spinlock)
  struct CurrLoopConfig {
    float P;
    float I;
    float limitA;
    float lpfTf_s;
    float maxCurrentA;
  };

  Motors();

  // Call once from setup()
  void resetSafe();
  void begin();

  // Called from I2C callbacks or higher-level control code
  void setModeAndAmplitude(ControlMode leftMode,  int8_t leftAmp,
                           ControlMode rightMode, int8_t rightAmp);

  // Update current-loop tuning (both motors share same loop tuning)
  void setCurrentLoopParams(const CurrLoopConfig& newParams);

  // Aggregate status: approx DC current (Iq_L + Iq_R) and max |ω|
  void getStatus(float &iqL, float &iqR, float& speedL, float& speedR);


private:
  // FreeRTOS task trampoline + loop
  static void controlTaskThunk(void* param);
  void controlTask();

  // Hall ISR glue
  static void IRAM_ATTR hallLeftA();
  static void IRAM_ATTR hallLeftB();
  static void IRAM_ATTR hallLeftC();

  static void IRAM_ATTR hallRightA();
  static void IRAM_ATTR hallRightB();
  static void IRAM_ATTR hallRightC();

  // Internal helper to configure motors
  void configureMotorObjects();
  void configureCurrentSense();
  void configurePIDAndLimits();
  void startFOC();


  struct ChannelControl {
    ControlMode mode;
    int8_t     amplitude;    // -127..+128
  };

  // Simple spinlock for cross-task protection
  static portMUX_TYPE s_mux;

  static inline void enterCritical() { portENTER_CRITICAL(&s_mux); }
  static inline void exitCritical()  { portEXIT_CRITICAL(&s_mux);  }

  // Pending / active current-loop config
  CurrLoopConfig _cfgPending;
  CurrLoopConfig _cfgActive;
  volatile bool  _cfgDirty = false;

  // Left/Right control state
  ChannelControl _leftCtrl;
  ChannelControl _rightCtrl;

  // Status (updated by control task, read externally)
  float _statusIqL        = 0.0f;
  float _statusIqR        = 0.0f;
  float _statusVelL       = 0.0f;
  float _statusVelR       = 0.0f;

  // ===== SimpleFOC objects =====
  // (Pins are for your MakerBase Dual FOC + Lolin32 Lite combo)

  // Hall sensors
  HallSensor _hallLeft;
  HallSensor _hallRight;

  // Motors
  BLDCMotor _motorLeft;
  BLDCMotor _motorRight;

  // Drivers (EN pins: 22 for left, 12 for right)
  BLDCDriver3PWM _driverLeft;
  BLDCDriver3PWM _driverRight;

  // Current sense (inline, 2-shunt)
  InlineCurrentSense _csLeft;
  InlineCurrentSense _csRight;
};

// global instance used from main.cpp
extern Motors motors;