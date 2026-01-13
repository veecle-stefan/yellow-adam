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

};
