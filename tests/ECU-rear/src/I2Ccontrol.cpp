#include "I2Ccontrol.h"

// ===== Static instance pointer =====
I2CControl* I2CControl::s_instance = nullptr;

// ===== Constructor =====
I2CControl::I2CControl()
  : _modeHandler(nullptr),
    _ampHandler(nullptr),
    _paramHandler(nullptr),
    _modePending(false),
    _pendingMode(ControlMode::MODE_TORQUE_CURRENT),
    _pendingDefaultAmplitude(0),
    _pendingAmpL(0),
    _pendingAmpR(0),
    _paramPending(false),
    _ampPending(false)
{

}

// ===== Public API =====

void I2CControl::begin(uint8_t addr, int sda, int scl, uint32_t freq) {
  s_instance = this;

  Wire.begin(addr, sda, scl, freq);
  Wire.onReceive(I2CControl::onReceiveThunk);
  Wire.onRequest(I2CControl::onRequestThunk);
}

void I2CControl::setModeCommandHandler(ModeCommandHandler handler) {
  _modeHandler = handler;
}

void I2CControl::setAmplitudeHandler(AmplitudeHandler handler) {
  _ampHandler = handler;
}

void I2CControl::setParamUpdateHandler(ParamUpdateHandler handler) {
  _paramHandler = handler;
}

void I2CControl::setStatus(const StatusUpdate& newUpdate) {
  _status = newUpdate;
}

void I2CControl::update() {
  // Process mode/amplitude update
  if (_modePending) {
    noInterrupts();
    ControlMode mode = _pendingMode;
    int8_t amp      = _pendingDefaultAmplitude;
    _modePending     = false;
    interrupts();

    if (_modeHandler) {
      _modeHandler(mode, amp);
    }
  }

  if (_ampPending) {
    noInterrupts();
    int8_t ampL = _pendingAmpL;
    int8_t ampR = _pendingAmpR;
    _ampPending      = false;
    interrupts();

    if (_ampHandler) {
      _ampHandler(ampL, ampR);
    }
  }

  // Process parameter update
  if (_paramPending) {
    noInterrupts();
    uint8_t paramID = _pendingParam.paramId;
    float paramValue = _pendingParam.value;
    _paramPending   = false;
    interrupts();

    if (_paramHandler) {
      _paramHandler(paramID, paramValue);
    }
  }
}

// ===== Wire callback glue =====

void I2CControl::onReceiveThunk(int numBytes) {
  if (s_instance) {
    s_instance->handleReceive(numBytes);
  } else {
    while (Wire.available()) Wire.read(); // flush
  }
}

void I2CControl::onRequestThunk() {
  if (s_instance) {
    s_instance->handleRequest();
  } else {
    // nothing
  }
}

// ===== Internal handlers (called from ISR context) =====

void I2CControl::handleReceive(int numBytes) {
  if (numBytes < 1) return;

  uint8_t cmd = Wire.read();
  numBytes--;

  switch (cmd) {
    case CMD_SET_MODE: {
      // [cmd, mode, amplitude]
      if (numBytes >= 2) {
        uint8_t mode = Wire.read();
        int8_t amp  = Wire.read();

        if (mode <= static_cast<uint8_t>(ControlMode::MODE_VELOCITY_BRAKE)) {
          _pendingMode      = static_cast<ControlMode>(mode);
          _pendingDefaultAmplitude = amp;
          _modePending      = true;
        }
      } else {
        while (Wire.available()) Wire.read();
      }
      break;
    }

    case CMD_SET_AMPL: {
      // [cmd, amplitude]
      if (numBytes >= 1) {
        int8_t ampL = Wire.read();
        int8_t ampR = Wire.read();
        _pendingAmpL = ampL;
        _pendingAmpR = ampR;
        _modePending      = true; // treat as mode+ampl update with same mode
      } else {
        while (Wire.available()) Wire.read();
      }
      break;
    }

    case CMD_SET_PARAM: {
      // [cmd, paramId, float4]
      if (numBytes >= 5) {
        uint8_t paramId = Wire.read();
        uint8_t buf[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4 && Wire.available(); i++) {
          buf[i] = Wire.read();
        }
        float value;
        memcpy(&value, buf, sizeof(float)); // assumes little-endian

        _pendingParam.paramId = paramId;
        _pendingParam.value   = value;
        _paramPending         = true;
      } else {
        while (Wire.available()) Wire.read();
      }
      break;
    }

    default:
      // Unknown command, flush remaining bytes
      while (Wire.available()) Wire.read();
      break;
  }
}

void I2CControl::handleRequest() {
  Wire.write((uint8_t*)&_status, sizeof(StatusUpdate));
}

I2CControl I2CMotorCommands;