#pragma once

#include <Arduino.h>
#include <optional>

#include "axle.h"
#include "lights.h"
#include "rcinput.h"

namespace DriveConfig {
    static constexpr int16_t DeadBand = 50;

    // Brakes / tap logic
    namespace Brakes {
        static constexpr uint16_t DetectThreshold   = 50;    // throttle <= => braking
        static constexpr uint32_t TapTime           = 200;   // ms
        static constexpr uint32_t DoubleTapTime     = 1000;  // ms
        static constexpr uint16_t AntiReversingSpeed = 100;  // speed unit from feedback
    }

    // Safety thresholds (scaled!)
    namespace Controller {
        static constexpr int16_t VoltageOff  = 3300; // 33.00 V
        static constexpr int16_t VoltageWarn = 3500; // 35.00 V
        static constexpr int16_t TempOff     = 600;  // 60.0 C
        static constexpr int16_t TempWarn    = 500;  // 50.0 C
    }

    namespace TV { // Torque Vectoring
        static constexpr float SteerTorqueFront = 30.f;
        static constexpr float SteerTorqueRear = 50.f;
    }
}

enum Gear {
    N = 0,
    D = 1,
    R = 2
};

struct VehicleState {
    bool loBeam = false;
    bool hiBeam = false;
    bool hazards = false;
    bool indicatorsL = false;
    bool indicatorsR = false;
    Gear currGear = Gear::N;

    bool fake = true;
};

struct DriveTrainStatus {
  uint32_t ts_ms = 0;

  int16_t throttle = 0;
  int16_t steering = 0;
  int16_t aux      = 0;
  bool    userDetected = false;

  // last applied torques
  int16_t tq_fl = 0, tq_fr = 0, tq_rl = 0, tq_rr = 0;

  // motor feedback (last known good)
  int16_t curr_fl = 0, curr_fr = 0, curr_rl = 0, curr_rr = 0;
  int16_t vel_fl = 0, vel_fr = 0, vel_rl = 0, vel_rr = 0;
  uint16_t voltage_front = 0, voltage_rear = 0; // 35.00V => 3500 etc
  uint16_t temp_front    = 0, temp_rear    = 0; // 50.0C => 500 etc
  bool haveFront = false;
  bool haveRear  = false;

  VehicleState state;
};

class DriveTrain
{
public:
    static constexpr uint16_t ControlPeriodMs = 20; // 50 Hz

    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);
    bool GetLatestStatus(DriveTrainStatus& out) const;
    void Shutdown();

protected:
    // ----- Types -----
    using MotorStates = std::optional<Axle::HistoryFrame>;
    using UserInput = RCinput::UserInput;
    
    struct UserCmd {
        int16_t throttle = 0;
        int16_t steering = 0;
        int16_t aux      = 0;
        bool    detected = false;
        bool    doubleTap = false;
    };

    struct TickContext {
        uint32_t    nowMs = 0;
        UserCmd     user{};

        MotorStates currFront;
        MotorStates currRear;
        MotorStates lastFront;
        MotorStates lastRear;
    };

    struct Torques {
        int16_t fl = 0, fr = 0, rl = 0, rr = 0;
    };

    struct TickDecision {
        Torques torques{};
        Axle::RemoteCommand cmd = Axle::RemoteCommand::CmdNOP;

        // “Intent” for outputs besides motors
        bool failSafe = false;
        bool brakeLight = false;
        bool reverseLight = false;
        bool tailLight = false;
    };

    // ----- Members -----
    QueueHandle_t statusQueue = NULL;
    TaskHandle_t processTask = NULL;
    RCinput ch1;
    RCinput ch2;
    RCinput ch3;
    Axle&    axleF;
    Axle&    axleR;
    Lights&  lights;

    MotorStates lastFrontFb;
    MotorStates lastRearFb;

    // ----- Background control task -----
    void ControlTask();

    // ----- Pipeline -----
    TickContext  BuildContext(uint32_t nowMs);
    TickDecision ComputeDecision(const TickContext& ctx, VehicleState& state);
    void         ApplyDecision(const TickDecision& dec, VehicleState& state);
    void         CheckGear(const TickContext& ctx, VehicleState& state);

    // ----- Helpers (small, single-purpose) -----
    void PublishStatus(const TickContext& ctx, const TickDecision& dec, const VehicleState& state);

    static UserCmd     ReadUserCmd(UserInput ch1, UserInput ch2, UserInput ch3, uint32_t nowMs);
    static Axle::RemoteCommand ControllerSafety(const TickContext& ctx);
    static void        ComputeLights(const TickContext& ctx, TickDecision& dec,  VehicleState& state);

    static Torques     TorqueVectoring(const TickContext& ctx, const VehicleState& state); // no out-params
};