#include <Arduino.h>
#include <SimpleFOC.h>
#include "motors.h"
#include "I2Ccontrol.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==============================
// Global configuration
// ==============================


// Control loop period (1 kHz)
static constexpr uint32_t CONTROL_PERIOD_MS = 1;

// ===== Static spinlock =====
portMUX_TYPE Motors::s_mux = portMUX_INITIALIZER_UNLOCKED;

// ===== Global instance =====
Motors motors;

// ===== ISR glue (needs a global pointer to instance) =====
static Motors* s_instance = nullptr;

// ----- Motors ctor -----
Motors::Motors()
  : _cfgPending{CURR_Q_P_DEFAULT, CURR_Q_I_DEFAULT,
                CURR_Q_LIMIT_A,   CURR_LPF_TF_DEFAULT,
                CURR_Q_LIMIT_A},
    _cfgActive(_cfgPending),
    _leftCtrl{ControlMode::MODE_TORQUE_CURRENT, 0},
    _rightCtrl{ControlMode::MODE_TORQUE_CURRENT, 0},
    // Hall sensors: (U,V,W,pole_pairs)
    _hallLeft (HallSensor(PIN_MOTL_HALL_A, PIN_MOTL_HALL_B, PIN_MOTL_HALL_C, PP)),
    _hallRight(HallSensor(PIN_MOTR_HALL_A, PIN_MOTR_HALL_B, PIN_MOTR_HALL_C, PP)),
    // Motors
    _motorLeft (BLDCMotor(PP)),
    _motorRight(BLDCMotor(PP)),
    // Drivers: (U, V, W, EN)
    _driverLeft (BLDCDriver3PWM(PIN_MOTL_PH_A, PIN_MOTL_PH_B, PIN_MOTL_PH_C, PIN_MOTL_PH_EN)),
    _driverRight(BLDCDriver3PWM(PIN_MOTR_PH_A, PIN_MOTR_PH_B, PIN_MOTR_PH_C, PIN_MOTR_PH_EN)),
    // Current sense: (Rshunt, gain, Ia_pin, Ib_pin)
    _csLeft (InlineCurrentSense(SHUNT_RESISTANCE, OPAMP_GAIN, PIN_MOTL_CURR_A, PIN_MOTL_CURR_B)),
    _csRight(InlineCurrentSense(SHUNT_RESISTANCE, OPAMP_GAIN, PIN_MOTR_CURR_A, PIN_MOTR_CURR_B)) {
}

// ===== Public API =====

void Motors::resetSafe() {
// set EN low first then OUTPUT
  digitalWrite(PIN_MOTL_PH_EN, LOW);
  digitalWrite(PIN_MOTR_PH_EN, LOW);
  pinMode(PIN_MOTL_PH_EN, OUTPUT);
  pinMode(PIN_MOTR_PH_EN, OUTPUT);

  digitalWrite(PIN_MOTL_PH_A, LOW);
  digitalWrite(PIN_MOTL_PH_B, LOW);
  digitalWrite(PIN_MOTL_PH_C, LOW);
  digitalWrite(PIN_MOTR_PH_A, LOW);
  digitalWrite(PIN_MOTR_PH_B, LOW);
  digitalWrite(PIN_MOTR_PH_C, LOW);

  pinMode(PIN_MOTL_PH_A, OUTPUT);
  pinMode(PIN_MOTL_PH_B, OUTPUT);
  pinMode(PIN_MOTL_PH_C, OUTPUT);
  pinMode(PIN_MOTR_PH_A, OUTPUT);
  pinMode(PIN_MOTR_PH_B, OUTPUT);
  pinMode(PIN_MOTR_PH_C, OUTPUT);
}

void Motors::begin() {
  s_instance = this;

  configureMotorObjects();
  configureCurrentSense();
  configurePIDAndLimits();
  startFOC();

  // Create control task pinned to core 1 (keep core 0 for WiFi/RTOS even if disabled)
  xTaskCreatePinnedToCore(
    controlTaskThunk,
    "foc_control",
    4096,
    this,
    2,
    nullptr,
    1
  );
}

void Motors::setModeAndAmplitude(ControlMode leftMode,  int8_t leftAmp,
                                 ControlMode rightMode, int8_t rightAmp) {
  enterCritical();
  _leftCtrl.mode   = leftMode;
  _leftCtrl.amplitude  = leftAmp;
  _rightCtrl.mode  = rightMode;
  _rightCtrl.amplitude = rightAmp;
  exitCritical();
}

void Motors::setCurrentLoopParams(const CurrLoopConfig& newParams) {
  enterCritical();
  _cfgPending = newParams;
  _cfgDirty   = true;
  exitCritical();
}

void Motors::getStatus(float &iqL, float &iqR, float& speedL, float& speedR) {
  enterCritical();
  iqL = _statusIqL;
  iqR = _statusIqR;
  speedL = _statusVelL;
  speedR = _statusVelR;
  exitCritical();
}

// ===== Internal configuration =====

void Motors::configureMotorObjects() {
  // Hall sensor pullups + init
  _hallLeft.pullup  = Pullup::USE_INTERN;
  _hallRight.pullup = Pullup::USE_INTERN;

  _hallLeft.init();
  _hallRight.init();

  // Attach hall interrupts
  _hallLeft.enableInterrupts(hallLeftA, hallLeftB, hallLeftC);
  _hallRight.enableInterrupts(hallRightA, hallRightB, hallRightC);

  // Link sensors
  _motorLeft.linkSensor(&_hallLeft);
  _motorRight.linkSensor(&_hallRight);

  // Driver supply configuration
  _driverLeft.voltage_power_supply  = VOLTAGE_SUPPLY;
  _driverRight.voltage_power_supply = VOLTAGE_SUPPLY;

  _driverLeft.init();
  _driverRight.init();

  // Link drivers
  _motorLeft.linkDriver(&_driverLeft);
  _motorRight.linkDriver(&_driverRight);

  // Basic motor config
  _motorLeft.phase_resistance  = PHASE_RESISTANCE;
  _motorRight.phase_resistance = PHASE_RESISTANCE;

  _motorLeft.voltage_limit  = VOLTAGE_LIMIT;
  _motorRight.voltage_limit = VOLTAGE_LIMIT;

  _motorLeft.velocity_limit  = VEL_LIMIT;
  _motorRight.velocity_limit = VEL_LIMIT;

  // Default to current FOC torque control; mode-specific changes happen per-cycle
  _motorLeft.torque_controller  = TorqueControlType::foc_current;
  _motorRight.torque_controller = TorqueControlType::foc_current;

  _motorLeft.controller  = MotionControlType::torque;
  _motorRight.controller = MotionControlType::torque;

  // If you have known calibration values for the sensors, set them here
  // These were from your earlier experiments:
  _motorLeft.zero_electric_angle  = 2.094395f;
  _motorLeft.sensor_direction     = Direction::CCW;
  _motorRight.zero_electric_angle = 2.094395f;
  _motorRight.sensor_direction    = Direction::CCW;

  _motorLeft.LPF_velocity.Tf  = 0.01f;
  _motorRight.LPF_velocity.Tf = 0.01f;
}

void Motors::configureCurrentSense() {
  // Left
  _csLeft.linkDriver(&_driverLeft);
  _csLeft.skip_align = true;
  _csLeft.init();
  _csLeft.offset_ia = LEFT_OFFSET_IA;
  _csLeft.offset_ib = LEFT_OFFSET_IB;
  _motorLeft.linkCurrentSense(&_csLeft);

  // Right
  _csRight.linkDriver(&_driverRight);
  _csRight.skip_align = true;
  _csRight.init();
  _csRight.offset_ia = RIGHT_OFFSET_IA;
  _csRight.offset_ib = RIGHT_OFFSET_IB;
  _motorRight.linkCurrentSense(&_csRight);
}

void Motors::configurePIDAndLimits() {
  // Initial current-loop config (q-axis)
  CurrLoopConfig cfg = _cfgActive;

  _motorLeft.PID_current_q.P   = cfg.P;
  _motorLeft.PID_current_q.I   = cfg.I;
  _motorLeft.PID_current_q.limit = cfg.limitA;
  _motorLeft.LPF_current_q.Tf  = cfg.lpfTf_s;

  _motorRight.PID_current_q.P   = cfg.P;
  _motorRight.PID_current_q.I   = cfg.I;
  _motorRight.PID_current_q.limit = cfg.limitA;
  _motorRight.LPF_current_q.Tf  = cfg.lpfTf_s;

  // d-axis – keep more conservative
  _motorLeft.PID_current_d.P   = CURR_Q_P_DEFAULT;
  _motorLeft.PID_current_d.I   = CURR_Q_I_DEFAULT;
  _motorLeft.PID_current_d.limit = CURR_Q_LIMIT_A;
  _motorLeft.LPF_current_d.Tf  = CURR_LPF_TF_DEFAULT;

  _motorRight.PID_current_d.P   = CURR_Q_P_DEFAULT;
  _motorRight.PID_current_d.I   = CURR_Q_I_DEFAULT;
  _motorRight.PID_current_d.limit = CURR_Q_LIMIT_A;
  _motorRight.LPF_current_d.Tf  = CURR_LPF_TF_DEFAULT;

  // Velocity PID (for braking mode)
  _motorLeft.PID_velocity.P = VEL_P_DEFAULT;
  _motorLeft.PID_velocity.I = VEL_I_DEFAULT;
  _motorLeft.PID_velocity.D = VEL_D_DEFAULT;

  _motorRight.PID_velocity.P = VEL_P_DEFAULT;
  _motorRight.PID_velocity.I = VEL_I_DEFAULT;
  _motorRight.PID_velocity.D = VEL_D_DEFAULT;

  _motorLeft.PID_velocity.output_ramp  = VEL_RAMP;
  _motorRight.PID_velocity.output_ramp = VEL_RAMP;
}

void Motors::startFOC() {
  _motorLeft.init();
  _motorRight.init();

  _motorLeft.initFOC();
  _motorRight.initFOC();

  _motorLeft.enable();
  _motorRight.enable();
}

// ===== FreeRTOS control task =====

void Motors::controlTaskThunk(void* param) {
  auto* self = static_cast<Motors*>(param);
  self->controlTask();
}

void Motors::controlTask() {
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // ---- Snapshot control + config under lock ----
    ChannelControl left, right;
    CurrLoopConfig cfg;
    bool cfgDirty;

    enterCritical();
    left      = _leftCtrl;
    right     = _rightCtrl;
    cfgDirty  = _cfgDirty;
    cfg       = _cfgActive;
    if (cfgDirty) {
      cfg = _cfgPending;
      _cfgActive = cfg;
      _cfgDirty  = false;
    }
    exitCritical();

    // ---- Apply new current-loop config if changed ----
    if (cfgDirty) {
      _motorLeft.PID_current_q.P   = cfg.P;
      _motorLeft.PID_current_q.I   = cfg.I;
      _motorLeft.PID_current_q.limit = cfg.limitA;
      _motorLeft.LPF_current_q.Tf  = cfg.lpfTf_s;

      _motorRight.PID_current_q.P   = cfg.P;
      _motorRight.PID_current_q.I   = cfg.I;
      _motorRight.PID_current_q.limit = cfg.limitA;
      _motorRight.LPF_current_q.Tf  = cfg.lpfTf_s;
    }

    // ---- Map amplitudes (0..255) to targets per motor ----
    auto mapChannel = [&](const ChannelControl& cc, BLDCMotor& m) -> float {
      const float ampNorm = cc.amplitude / 255.0f;
      switch (cc.mode) {
        case ControlMode::MODE_TORQUE_VOLTAGE: {
          const float v = ampNorm * VOLTAGE_LIMIT;
          m.torque_controller = TorqueControlType::voltage;
          m.controller        = MotionControlType::torque;
          return v;
        }
        case ControlMode::MODE_TORQUE_CURRENT: {
          const float iq = ampNorm * cfg.maxCurrentA;
          m.torque_controller = TorqueControlType::foc_current;
          m.controller        = MotionControlType::torque;
          return iq;
        }
        case ControlMode::MODE_VELOCITY_BRAKE: {
          const float vel = ampNorm * VEL_LIMIT;
          m.torque_controller = TorqueControlType::foc_current;
          m.controller        = MotionControlType::velocity;
          return vel;
        }
      }
      return 0.0f;
    };

    float targetL = mapChannel(left,  _motorLeft);
    float targetR = mapChannel(right, _motorRight);

    // ---- FOC computation ----
    _motorLeft.loopFOC();
    _motorRight.loopFOC();

    _motorLeft.move(targetL);
    _motorRight.move(targetR);

    // ---- Status snapshot ----
    const float iqL  = _motorLeft.current.q;
    const float iqR  = _motorRight.current.q;
    const float velL = _motorLeft.shaft_velocity;
    const float velR = _motorRight.shaft_velocity;

    
    enterCritical();
    _statusIqL        = iqL;
    _statusIqR        = iqR;
    _statusVelL       = velL;
    _statusVelR       = velR;
    exitCritical();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// ===== Hall ISR glue =====

void IRAM_ATTR Motors::hallLeftA()  { if (s_instance) s_instance->_hallLeft.handleA(); }
void IRAM_ATTR Motors::hallLeftB()  { if (s_instance) s_instance->_hallLeft.handleB(); }
void IRAM_ATTR Motors::hallLeftC()  { if (s_instance) s_instance->_hallLeft.handleC(); }

void IRAM_ATTR Motors::hallRightA() { if (s_instance) s_instance->_hallRight.handleA(); }
void IRAM_ATTR Motors::hallRightB() { if (s_instance) s_instance->_hallRight.handleB(); }
void IRAM_ATTR Motors::hallRightC() { if (s_instance) s_instance->_hallRight.handleC(); }