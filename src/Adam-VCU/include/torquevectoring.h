#pragma once
#include <Arduino.h>
#include <optional>
#include "axle.h"
#include "rcinput.h"
#include "driveparams.h"

class TorqueVectoring
{
public:
    // ----- Types -----
    using MotorStates = std::optional<Axle::HistoryFrame>;
    using UserInput = RCinput::UserInput;

    enum Gear : uint8_t
    {
        N = 0,
        D = 1,
        R = 2
    };

    struct UserCmd {
        int16_t throttle = 0;
        int16_t steering = 0;
        int16_t aux      = 0;
        bool    detected = false;
        bool    someInput = false;
        bool    auxPressed = false;
        bool    doubleTap = false;
    };

    struct TickContext {
        uint32_t    nowMs = 0;
        UserCmd     user{};

        MotorStates currFront;
        MotorStates currRear;
        MotorStates lastFront;
        MotorStates lastRear;

        DriveParams* params;
    };

    struct Torques {
        int16_t fl = 0, fr = 0, rl = 0, rr = 0;
    };

    Torques Compute(const TickContext &ctx, const Gear& currGear); 

protected:
    struct SenseData
    {
        // Signed wheel speeds (from feedback)
        bool ok[4] = {false, false, false, false};
        float w[4] = {0, 0, 0, 0};    // signed
        float wAbs[4] = {0, 0, 0, 0}; // abs
        float vehicleSpeedAbs = 0.f;

        // Per-wheel torque envelope for THIS tick
        float capPos[4] = {0, 0, 0, 0}; // max allowed positive torque
        float capNeg[4] = {0, 0, 0, 0}; // min allowed negative torque (<=0)
    };

    SenseData Sense(const TickContext &ctx, Gear currGear);

    float m_vehicleSpeedAbs = 0.f; // persisted across ticks
    bool m_capsInit;
	float m_capPos[4];
	float m_capNeg[4];
};
