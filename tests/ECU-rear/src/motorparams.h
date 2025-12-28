#pragma once

// Pins
#define PIN_MOTL_PH_A 32
#define PIN_MOTL_PH_B 33
#define PIN_MOTL_PH_C 25
#define PIN_MOTL_PH_EN 22
#define PIN_MOTL_HALL_A 5
#define PIN_MOTL_HALL_B 23
#define PIN_MOTL_HALL_C 13
#define PIN_MOTL_CURR_A 39
#define PIN_MOTL_CURR_B 36

#define PIN_MOTR_PH_A 26
#define PIN_MOTR_PH_B 27
#define PIN_MOTR_PH_C 14
#define PIN_MOTR_PH_EN 12
#define PIN_MOTR_HALL_A 18
#define PIN_MOTR_HALL_B 19
#define PIN_MOTR_HALL_C 15
#define PIN_MOTR_CURR_A 35
#define PIN_MOTR_CURR_B 34

#include "motors.h"

// ===== Board / motor constants =====
static constexpr int   PP                 = 15;   // hoverboard motor pole pairs
static constexpr float VOLTAGE_SUPPLY     = 24.0f;
static constexpr float VOLTAGE_LIMIT      = 6.0f;
static constexpr float VEL_LIMIT          = 45.0f;      // rad/s
static constexpr float PHASE_RESISTANCE   = 1.0f;

// Current sense config
static constexpr float SHUNT_RESISTANCE   = 0.01f;      // 10 mΩ
static constexpr float OPAMP_GAIN         = 50.0f;

// Measured DC offsets (your values)
static constexpr float LEFT_OFFSET_IA     = 1.53f;
static constexpr float LEFT_OFFSET_IB     = 1.56f;
static constexpr float RIGHT_OFFSET_IA    = 1.54f;
static constexpr float RIGHT_OFFSET_IB    = 1.52f;

// Current-loop defaults
static constexpr float CURR_Q_P_DEFAULT   = 2.0f;
static constexpr float CURR_Q_I_DEFAULT   = 50.0f;
static constexpr float CURR_Q_LIMIT_A     = 2.0f;
static constexpr float CURR_LPF_TF_DEFAULT= 0.005f;

// PID velocity defaults (if/when you use velocity mode)
static constexpr uint16_t VEL_RAMP        = 1000;
static constexpr float VEL_P_DEFAULT      = 0.1f;
static constexpr float VEL_I_DEFAULT      = 1.0f;
static constexpr float VEL_D_DEFAULT      = 0.0f;