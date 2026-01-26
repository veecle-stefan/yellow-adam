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

    struct HoldFlags
    {
        bool hFL = false;
        bool hFR = false;
        bool hRL = false;
        bool hRR = false;
    };

    struct Torques { int16_t fl=0, fr=0, rl=0, rr=0;
        int16_t vehicleSpeedAbs; 
        HoldFlags hold;
    };

    Torques Compute(const TickContext& ctx, const Gear& currGear);

protected:
    enum Wheel : uint8_t { FL=0, FR=1, RL=2, RR=3 };

    struct SenseData
    {
        bool  ok[4]   = {false,false,false,false};
        // Signed wheel speeds in MOTOR coordinates:
        //  >0 means motor spins "forward" (as defined by controller),
        //  <0 means motor spins "reverse".
        float w[4]    = {0,0,0,0};
        float wAbs[4] = {0,0,0,0};    // abs wheel speeds
        float vehicleSpeedAbs = 0.f;
        float cmdAtReading[4] = {0,0,0,0};
        float motionSign = 0.f; // positive=forward motion of wheels
        float gearDir = 0.f; // D=+1, R=-1, N=0
        // sign of motion expressed "along gear" (i.e. +1 means moving in gear direction)
        float motionSignAlongGear = 0.f; // = motionSign * gearDir when motionSign!=0

        // Per-wheel torque envelope for THIS tick
        float capFwd[4] = {0,0,0,0};  // max allowed + torque
        float capRev[4] = {0,0,0,0};  // min allowed - torque (<=0)
    };

    // ---- Stage 0: sensing + caps update
    SenseData Sense(const TickContext& ctx, Gear currGear);

    // ---- Sub-stages of Sense()
    void ReadWheelSpeeds(const TickContext& ctx, SenseData& sd);
    void ReadAxleSpeeds(const std::optional<Axle::HistoryFrame>& fb, SenseData& sd, int indexShift);
    void EstimateVehicleMotion(const TickContext& ctx, SenseData& sd, struct SenseContext& sc);
    void UpdateSlipEnvelopes(const TickContext& ctx, SenseData& sd, const SenseContext& sc);
    void ApplySpeedLimiter(const TickContext& ctx, Gear currGear, SenseData& sd);

    // ---- Trajectory intent computation
    struct TrajectoryIntent {
        float Tc_total = 0.f; // Total longitudinal torque request (gear space)
        float TcF = 0.f;      // Front common torque request (gear space)
        float TcR = 0.f;      // Rear common torque request (gear space)
        float TdF = 0.f;      // Front differential torque request
        float TdR = 0.f;      // Rear differential torque request
    };

    TrajectoryIntent ComputeTrajectoryIntent(const TickContext& ctx, const SenseData& sense, Gear currGear);

    // ---- Wheel torque solving
    struct WheelTorques {
        float fl = 0.f, fr = 0.f, rl = 0.f, rr = 0.f;
    };

    WheelTorques SolveWheelTorques(const TickContext& ctx, const SenseData& sense, const TrajectoryIntent& intent);


    struct PairSolve {
        float Tc=0.f, Td=0.f;
        float TcMin=0.f, TcMax=0.f;
        float L=0.f, R=0.f;
    };

    // Solve L=Tc+Td, R=Tc-Td with per-wheel caps; priority = keep Td (trajectory)
    static PairSolve SolvePairCaps(float TcReq, float TdReq,
                                   float capNegL, float capPosL,
                                   float capNegR, float capPosR,
                                    float k);

    static void ApplyBrakeFadeCaps(const TickContext &ctx, SenseData &sd);

    static HoldFlags ComputeHoldFlags(
        const TickContext &ctx,
        const SenseData &sense);

    static float AxleSpeedAbs(const SenseData &s, Wheel L, Wheel R);

    // ---- Persistent state
    float m_vehicleSpeedAbs = 0.f;

    bool  m_capsInit = false;
    float m_capFwd[4] = {0,0,0,0};
    float m_capRev[4] = {0,0,0,0};

    // rate limit for front/rear steering
    float m_lastTdF = 0.f;
    float m_lastTdR = 0.f;
};