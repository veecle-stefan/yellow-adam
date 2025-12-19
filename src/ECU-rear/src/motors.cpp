#include <Arduino.h>
#include <SimpleFOC.h>
#include "motors.h"

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==============================
// Global configuration
// ==============================

constexpr int   MOTOR_PP             = 15;

constexpr float SUPPLY_VOLTAGE       = 12.0f;
constexpr float VOLTAGE_LIMIT        = 6.0f;
constexpr float PHASE_RESISTANCE     = 1.0f;

constexpr float VEL_LIMIT            = 45.0f;
constexpr float VEL_OUTPUT_RAMP      = 1000.0f;

// Current-loop PI (q & d)
constexpr float CURR_Q_P             = 0.5f;
constexpr float CURR_Q_I             = 50.0f;
constexpr float CURR_Q_LIMIT         = 2.0f;
constexpr float CURR_Q_OUTPUT_RAMP   = 1000.0f;

constexpr float CURR_D_P             = 2.0f;
constexpr float CURR_D_I             = 50.0f;
constexpr float CURR_D_LIMIT         = 2.0f;
constexpr float CURR_D_OUTPUT_RAMP   = 1000.0f;

// Current-loop filters
constexpr float CURR_LPF_TF          = 0.005f;   // 5 ms

// Velocity-loop PI
constexpr float VEL_P                = 0.1f;
constexpr float VEL_I                = 1.0f;
constexpr float VEL_D                = 0.0f;

// Hall / FOC alignment
constexpr float     ZERO_ELECTRIC_ANGLE  = 2.094395f;
constexpr Direction  SENSOR_DIR          = Direction::CCW;

// Current sense hardware
constexpr float SHUNT_RESISTANCE     = 0.01f;    // 10 mΩ
constexpr float SHUNT_GAIN           = 50.0f;    // INA240A2

// Hardcoded current-sense offsets (measured with 12 V present)
constexpr float CS1_OFFSET_IA        = 1.53f;
constexpr float CS1_OFFSET_IB        = 1.56f;
constexpr float CS2_OFFSET_IA        = 1.54f;
constexpr float CS2_OFFSET_IB        = 1.52f;

// Task timing
constexpr uint32_t CONTROL_PERIOD_MS = 1;   // 1 ms -> 1 kHz control loop
constexpr uint32_t COMMS_PERIOD_MS   = 10;  // 10 ms -> 100 Hz comms/monitor

// Task config
constexpr uint32_t CONTROL_TASK_STACK = 4096;
constexpr uint32_t COMMS_TASK_STACK   = 4096;
constexpr UBaseType_t CONTROL_TASK_PRIO = 3;
constexpr UBaseType_t COMMS_TASK_PRIO   = 1;
constexpr BaseType_t  CONTROL_TASK_CORE = 1;  // ESP32 core 1
constexpr BaseType_t  COMMS_TASK_CORE   = 0;  // ESP32 core 0

// ==============================
// Global objects (definitions)
// ==============================

// Hall sensors (U V W pins, PP)
HallSensor sensorL(5, 23, 13, MOTOR_PP);    // motor 1
HallSensor sensorR(18, 19, 15, MOTOR_PP);   // motor 2

// Motors & drivers
BLDCMotor      motorL(MOTOR_PP);
BLDCDriver3PWM driverL(32, 33, 25, 22);

BLDCMotor      motorR(MOTOR_PP);
BLDCDriver3PWM driver2(26, 27, 14, 21);

// Inline current sense
InlineCurrentSense current_senseL(SHUNT_RESISTANCE, SHUNT_GAIN, 39, 36);
InlineCurrentSense current_senseR(SHUNT_RESISTANCE, SHUNT_GAIN, 35, 34);

// Commander
Commander command = Commander(Serial);

// ==============================
// Local ISR glue
// ==============================

static void doA()  { sensorL.handleA(); }
static void doB()  { sensorL.handleB(); }
static void doC()  { sensorL.handleC(); }

static void doA1() { sensorR.handleA(); }
static void doB1() { sensorR.handleB(); }
static void doC1() { sensorR.handleC(); }

// Commander callbacks (still direct motor access for now)
static void doMotorL(char* cmd) { command.motor(&motorL, cmd); }
static void doMotorR(char* cmd) { command.motor(&motorR, cmd); }

// ==============================
// Helpers
// ==============================

static void configureSensor(HallSensor& s, void (*a)(), void (*b)(), void (*c)()) {
  s.pullup = Pullup::USE_INTERN;
  s.init();
  s.enableInterrupts(a, b, c);
}

static void configureDriver(BLDCDriver3PWM& d) {
  d.voltage_power_supply = SUPPLY_VOLTAGE;
  d.init();
}

static void configureCurrentSense(InlineCurrentSense& cs,
                                  BLDCDriver3PWM& driver,
                                  float offset_ia,
                                  float offset_ib) {
  cs.linkDriver(&driver);
  cs.skip_align = true;      // we hardcode alignment/offset
  cs.init();
  cs.offset_ia = offset_ia;
  cs.offset_ib = offset_ib;
}

static void configureMotorCommon(BLDCMotor& m,
                                 HallSensor& s,
                                 BLDCDriver3PWM& d) {
  m.linkSensor(&s);
  m.linkDriver(&d);

  // Motion control settings (still voltage-torque by default)
  m.torque_controller = TorqueControlType::voltage;
  m.controller        = MotionControlType::torque;

  m.voltage_limit     = VOLTAGE_LIMIT;
  m.velocity_limit    = VEL_LIMIT;
  m.phase_resistance  = PHASE_RESISTANCE;

  m.PID_velocity.output_ramp = VEL_OUTPUT_RAMP;

  // Velocity PID
  m.PID_velocity.P    = VEL_P;
  m.PID_velocity.I    = VEL_I;
  m.PID_velocity.D    = VEL_D;

  // Hall / FOC alignment (hardcoded)
  m.zero_electric_angle = ZERO_ELECTRIC_ANGLE;
  m.sensor_direction    = SENSOR_DIR;
}

static void configureCurrentPID(BLDCMotor& m) {
  // q-axis
  m.PID_current_q.P           = CURR_Q_P;
  m.PID_current_q.I           = CURR_Q_I;
  m.PID_current_q.limit       = CURR_Q_LIMIT;
  m.PID_current_q.output_ramp = CURR_Q_OUTPUT_RAMP;

  // d-axis
  m.PID_current_d.P           = CURR_D_P;
  m.PID_current_d.I           = CURR_D_I;
  m.PID_current_d.limit       = CURR_D_LIMIT;
  m.PID_current_d.output_ramp = CURR_D_OUTPUT_RAMP;

  // filters
  m.LPF_current_q.Tf          = CURR_LPF_TF;
  m.LPF_current_d.Tf          = CURR_LPF_TF;
}

// ==============================
// FreeRTOS tasks
// ==============================

static void controlTask(void* pvParameters) {
  // Periodic control loop using vTaskDelayUntil
  const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Core FOC loop – keep this *clean*, no Serial
    motorL.loopFOC();
    motorR.loopFOC();

    motorL.move();
    motorR.move();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

static void commsTask(void* pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(COMMS_PERIOD_MS);
  TickType_t lastWakeTime = xTaskGetTickCount();

  for (;;) {
    // Monitoring & commander – OK to touch Serial here
    motorL.monitor();
    motorR.monitor();
    command.run();

    vTaskDelayUntil(&lastWakeTime, period);
  }
}

// ==============================
// Public API
// ==============================

void setupMotors() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  // --- Sensors ---
  configureSensor(sensorL, doA,  doB,  doC);
  configureSensor(sensorR, doA1, doB1, doC1);

  // --- Drivers ---
  configureDriver(driverL);
  configureDriver(driver2);

  // --- Motors (link sensor+driver & common settings) ---
  configureMotorCommon(motorL, sensorL, driverL);
  configureMotorCommon(motorR, sensorR, driver2);

  // --- Current sense & FOC current PID ---
  configureCurrentSense(current_senseL, driverL, CS1_OFFSET_IA, CS1_OFFSET_IB);
  motorL.linkCurrentSense(&current_senseL);
  configureCurrentPID(motorL);

  configureCurrentSense(current_senseR, driver2, CS2_OFFSET_IA, CS2_OFFSET_IB);
  motorR.linkCurrentSense(&current_senseR);
  configureCurrentPID(motorR);

  // --- Init FOC ---
  motorL.init();
  motorR.init();
  motorL.initFOC();
  motorR.initFOC();

  motorL.enable();
  // motorR.enable();   // enable when you actually want it active

  // --- Commander setup ---
  command.add('A', doMotorL, "motorL");
  command.add('B', doMotorR, "motorR");

  // --- Monitoring ---
  motorL.useMonitoring(Serial);
  motorR.useMonitoring(Serial);
  motorL.monitor_downsample = 100;
  motorR.monitor_downsample = 100;
  motorL.monitor_variables  = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motorR.monitor_variables  = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;

  Serial.println(F("Motors ready."));
  Serial.println(F("Use 'A'/'B' commands via serial Commander."));

  // --- Create RTOS tasks ---
  xTaskCreatePinnedToCore(
      controlTask,
      "controlTask",
      CONTROL_TASK_STACK,
      nullptr,
      CONTROL_TASK_PRIO,
      nullptr,
      CONTROL_TASK_CORE
  );

  xTaskCreatePinnedToCore(
      commsTask,
      "commsTask",
      COMMS_TASK_STACK,
      nullptr,
      COMMS_TASK_PRIO,
      nullptr,
      COMMS_TASK_CORE
  );
}

