#pragma once

#include <Arduino.h>
#include <optional>

#include "axle.h"
#include "lights.h"
#include "rcinput.h"

namespace DriveConfig {
    static constexpr int16_t DeadBand = 50;
    static constexpr uint32_t MaxUserIdleBeforeShutdown = 600000; // 600s = 10min

    // Brakes / tap logic
    namespace Brakes {
        static constexpr uint16_t DetectThreshold   = 50;    // throttle <= => braking
        static constexpr uint32_t TapTime           = 300;   // ms
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
        // Maximum allowed change of steering-related torque per control tick,
        // expressed as a fraction of MaxOutputLimit.
        // Limits how fast torque steer / yaw assist can ramp to avoid mechanical shocks.
        static constexpr float MaxTorquePerTick = 0.3f;

        // Front axle steering torque gain at standstill / very low speed.
        // Used to overcome rack centering spring and static friction.
        static constexpr float SteerTorqueLowFactor = 1.f;

        // Front axle steering torque gain at higher vehicle speeds.
        // Reduced on purpose to avoid twitchy steering and oscillations.
        static constexpr float SteerTorqueHighFactor = 0.5f;

        // Speed at which SteerTorqueHighFactor is fully reached.
        // Below this, steering gain interpolates linearly between Low and High factors.
        static constexpr float SteerTorqueHighSpeed = 50.f;

        // Rear axle yaw assist fade-in based on vehicle speed.
        // Below RearFadeSpeed0: rear yaw assist disabled (standstill / noise region).
        static constexpr float RearFadeSpeed0 = 5.f;

        // Above RearFadeSpeed1: rear yaw assist fully enabled.
        // Between Speed0 and Speed1, effect ramps in linearly.
        static constexpr float RearFadeSpeed1 = 25.f;

        // Rear axle yaw assist fade-in based on longitudinal driver intent (throttle or brake).
        // Expressed as fraction of MaxOutputLimit.
        // Below this: rear yaw assist disabled to avoid jitter at zero throttle.
        static constexpr float RearFadeThrottle0 = 0.05f;

        // Above this throttle/brake magnitude: rear yaw assist fully enabled.
        // Between Throttle0 and Throttle1, effect ramps in linearly.
        static constexpr float RearFadeThrottle1 = 0.3f;

        // Maximum differential steering torque applied on the front axle.
        // This is the primary steering actuator (torque steer).
        static constexpr float SteerTorqueFront = 200.f;

        // Maximum yaw-assist differential torque applied on the rear axle.
        // This stabilizes / assists turning but is NOT a steering actuator.
        static constexpr float SteerTorqueRear = 200.f;

        // Absolute per-wheel torque limit sent to motor controllers.
        // All torque commands are clamped to [-MaxOutputLimit .. +MaxOutputLimit].
        static constexpr float MaxOutputLimit = 400.f;

        // -----------------------------------------------------------------------------
        // ABS / ASR – very simple reactive slip control
        //
        // This is NOT a predictive model and does not use wheel acceleration.
        // It purely reacts to speed deviations between wheels *in the same tick*.
        // The goal is robustness and stability, not optimal traction.
        // -----------------------------------------------------------------------------

        // Relative speed deviation threshold to detect slip.
        // If a wheel deviates more than this fraction from the reference wheel speed,
        // it is considered slipping.
        //
        // Example: 0.20 = 20%
        // - ASR: driven wheel spins > +20% faster than others under positive torque
        // - ABS: wheel slows > -20% compared to others under negative torque
        static constexpr float SlipRatio = 0.20f;

        // Torque reduction factor applied when slip is detected on a wheel.
        // The current requested torque for that wheel is multiplied by this value.
        // Repeated slip over consecutive ticks compounds the reduction.
        static constexpr float SlipDownFactor = 0.70f;

        // Minimum allowed torque scaling due to slip control.
        // Prevents torque from being reduced to zero permanently
        // and ensures the wheel can recover traction.
        static constexpr float SlipMinScale = 0.25f;

        // Recovery rate per control tick when no slip is detected.
        // The per-wheel torque scaling factor is increased by this amount towards 1.0
        // every tick without slip, enabling smooth reapplication of torque.
        static constexpr float SlipRecoverPerTick = 0.10f;

        // Minimum reference speed required to evaluate slip logic.
        // Below this speed, slip detection is disabled to avoid noise-triggered
        // ABS/ASR behavior at standstill or very low speeds.
        static constexpr float SlipSpeedEps = 10.f;

        // torque distribution between front and rear under acceleration and braking
        static constexpr float DriveFrontShareLow  = 0.55f;
        static constexpr float DriveFrontShareHigh = 0.30f;
        static constexpr float BrakeFrontShareLow  = 0.60f;
        static constexpr float BrakeFrontShareHigh = 0.80f;
        static constexpr float BiasHighThrottle    = 0.6f;   // |throttle|/maxT where high share is reached
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
    uint32_t lastUserInput = 0;
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
        bool    someInput = false;
        bool    auxSign   = false;
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
        bool loBeam = false;
        bool hiBeam = false;
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
    static Axle::RemoteCommand ControllerSafety(const TickContext& ctx, const VehicleState& state);
    static void        ComputeLights(const TickContext& ctx, TickDecision& dec,  VehicleState& state);

    static Torques     TorqueVectoring(const TickContext& ctx, const VehicleState& state); // no out-params
};