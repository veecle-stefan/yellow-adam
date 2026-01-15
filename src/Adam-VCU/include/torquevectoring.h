#pragma once
#include <Arduino.h>
#include <optional>
#include "axle.h"
#include "rcinput.h"
#include "driveparams.h"

class TorqueVectoring
{
public:
    using MotorStates = std::optional<Axle::HistoryFrame>;

    enum Gear : uint8_t { N=0, D=1, R=2 };

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

        DriveParams* params = nullptr;
    };

    struct Torques { int16_t fl=0, fr=0, rl=0, rr=0; };

    Torques Compute(const TickContext& ctx, const Gear& currGear);

protected:
    enum Wheel : uint8_t { FL=0, FR=1, RL=2, RR=3 };

    struct SenseData
    {
        bool  ok[4]   = {false,false,false,false};
        float w[4]    = {0,0,0,0};    // signed wheel speeds
        float wAbs[4] = {0,0,0,0};    // abs wheel speeds
        float vehicleSpeedAbs = 0.f;

        // Per-wheel torque envelope for THIS tick
        float capPos[4] = {0,0,0,0};  // max allowed + torque
        float capNeg[4] = {0,0,0,0};  // min allowed - torque (<=0)
    };

    // ---- Stage 0: sensing + caps update
    SenseData Sense(const TickContext& ctx, Gear currGear);


    struct PairSolve {
        float Tc=0.f, Td=0.f;
        float TcMin=0.f, TcMax=0.f;
        float L=0.f, R=0.f;
    };

    // Solve L=Tc+Td, R=Tc-Td with per-wheel caps; priority = keep Td (trajectory)
    static PairSolve SolvePairCaps(float TcReq, float TdReq,
                                   float capNegL, float capPosL,
                                   float capNegR, float capPosR);

    static float AxleSpeedAbs(const SenseData& s, Wheel L, Wheel R);
    static int   BestMotionWheel(const SenseData& s, float vRefAbs);

    // ---- Persistent state
    float m_vehicleSpeedAbs = 0.f;

    bool  m_capsInit = false;
    float m_capPos[4] = {0,0,0,0};
    float m_capNeg[4] = {0,0,0,0};

    // optional: rate limit memory for final wheel outputs
    float m_lastWheelCmd[4] = {0,0,0,0};
};