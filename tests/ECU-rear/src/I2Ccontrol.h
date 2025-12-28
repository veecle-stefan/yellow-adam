#pragma once

#include <Arduino.h>
#include <Wire.h>

// ===== Control mode enum =====
enum class ControlMode : uint8_t {
  MODE_TORQUE_VOLTAGE = 0,
  MODE_TORQUE_CURRENT = 1,
  MODE_VELOCITY_BRAKE = 2
};

struct StatusUpdate {
  int8_t currentL;
  int8_t currentR;
  int8_t speedL;
  int8_t speedR;
} __attribute__((packed));

// ===== Command IDs =====
constexpr uint8_t CMD_SET_MODE   = 0x01; // [cmd, mode, amplitude]
constexpr uint8_t CMD_SET_AMPL   = 0x02; // [cmd, amplitude]
constexpr uint8_t CMD_SET_PARAM  = 0x03; // [cmd, paramId, float4]

// ===== Parameter IDs for CMD_SET_PARAM =====
constexpr uint8_t PARAM_CURR_P       = 0; // current-loop P
constexpr uint8_t PARAM_CURR_I       = 1; // current-loop I
constexpr uint8_t PARAM_CURR_LIMIT   = 2; // per-axis Iq limit (A)
constexpr uint8_t PARAM_CURR_LPF_TF  = 3; // LPF time constant for current (s)
constexpr uint8_t PARAM_MAX_CURRENT  = 4; // global max current (A)

// ===== Defaults (you can override in begin()) =====
constexpr uint8_t  I2C_ADDR_DEFAULT   = 0x10;
constexpr int      I2C_SDA_DEFAULT    = 16;
constexpr int      I2C_SCL_DEFAULT    = 17;
constexpr uint32_t I2C_FREQ_DEFAULT   = 400000;

// ===== Callback function types =====
typedef void (*ModeCommandHandler)(ControlMode mode, int8_t defaultAmplitude);
typedef void (*AmplitudeHandler)(int8_t ampL, int8_t ampR);
typedef void (*ParamUpdateHandler)(uint8_t paramId, float value);

class I2CControl {
public:
  I2CControl();

  void begin(uint8_t addr = I2C_ADDR_DEFAULT,
             int sda = I2C_SDA_DEFAULT,
             int scl = I2C_SCL_DEFAULT,
             uint32_t freq = I2C_FREQ_DEFAULT);

  void setModeCommandHandler(ModeCommandHandler handler);
  void setAmplitudeHandler(AmplitudeHandler handler);
  void setParamUpdateHandler(ParamUpdateHandler handler);

  // Call regularly from loop() (or a low-prio task)
  void update();

  // Expose current status to master on read
  void setStatus(const StatusUpdate& newStatus);

private:
  // Singleton instance used by Wire callbacks
  static I2CControl* s_instance;

  static void onReceiveThunk(int numBytes);
  static void onRequestThunk();

  void handleReceive(int numBytes);
  void handleRequest();

  ModeCommandHandler _modeHandler;
  ParamUpdateHandler _paramHandler;
  AmplitudeHandler _ampHandler;

  // Pending events from ISR → processed in update()
  volatile bool       _modePending;
  volatile ControlMode _pendingMode;
  volatile bool       _ampPending;
  volatile int8_t    _pendingDefaultAmplitude;
  volatile int8_t    _pendingAmpL;
  volatile int8_t    _pendingAmpR;

  struct ParamUpdate {
    uint8_t paramId;
    float   value;

    ParamUpdate(ParamUpdate& other) : paramId(other.paramId), value(other.value) {}
    ParamUpdate() : paramId(0), value(0.0f) {}
  };

  volatile bool        _paramPending;
  volatile ParamUpdate _pendingParam;

  StatusUpdate _status;
};

extern I2CControl I2CMotorCommands;