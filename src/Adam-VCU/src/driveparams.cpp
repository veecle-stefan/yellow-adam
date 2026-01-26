// driveParams.cpp
#include "driveparams.h"

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace
{
    static inline float clampf(float v, float lo, float hi)
    {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    }

    // Pointer-to-member table: type-safe, fast, perfect for float-only TV params.
    struct TVParamEntry
    {
        const char *key;                   // human-readable name (for UI/listing)
        float DriveParams::TVParams::*mem; // pointer to member inside DriveParams::TVParams
        float lo;                          // clamp lower bound
        float hi;                          // clamp upper bound
    };

    // IMPORTANT:
    // The index in this table is the ParamId used by Tuning::SetByID().
    // Keep this order stable once you start using IDs in tools/UI.
    // driveparams.cpp (or wherever your tuning table lives)
    static constexpr TVParamEntry kTVParams[] = {

        // =========================================================================
        // Output / safety limits (profile)
        // =========================================================================
        {"maxTorqueDrive", &DriveParams::TVParams::maxTorqueDrive, 0.0f, 1000.0f},
        {"maxTorqueBrake", &DriveParams::TVParams::maxTorqueBrake, 0.0f, 1000.0f},

        {"maxSpeedFwd", &DriveParams::TVParams::maxSpeedFwd, 0.0f, 2000.0f},
        {"maxSpeedRev", &DriveParams::TVParams::maxSpeedRev, 0.0f, 2000.0f},
        {"SpeedLimiterFadeBand", &DriveParams::TVParams::SpeedLimiterFadeBand, 0.0f, 500.0f},

        // =========================================================================
        // Steering torques
        // =========================================================================
        // NOTE: now absolute torque-per-tick, not a fraction.
        {"MaxTorquePerTick", &DriveParams::TVParams::MaxTorquePerTick, 0.0f, 500.0f},

        {"SteerTorqueFront", &DriveParams::TVParams::SteerTorqueFront, 0.0f, 1000.0f},
        {"SteerTorqueRear", &DriveParams::TVParams::SteerTorqueRear, 0.0f, 1000.0f},

        {"SteerTorqueLowFactor", &DriveParams::TVParams::SteerTorqueLowFactor, 0.0f, 2.0f},
        {"SteerTorqueHighFactor", &DriveParams::TVParams::SteerTorqueHighFactor, 0.0f, 2.0f},
        {"SteerTorqueHighSpeed", &DriveParams::TVParams::SteerTorqueHighSpeed, 0.0f, 2000.0f},

        {"RearFadeSpeed0", &DriveParams::TVParams::RearFadeSpeed0, 0.0f, 2000.0f},
        {"RearFadeSpeed1", &DriveParams::TVParams::RearFadeSpeed1, 0.0f, 2000.0f},
        {"RearFadeTorque0", &DriveParams::TVParams::RearFadeTorque0, 0.0f, 1000.0f},
        {"RearFadeTorque1", &DriveParams::TVParams::RearFadeTorque1, 0.0f, 1000.0f},

        // =========================================================================
        // Traction / ABS
        // =========================================================================
        {"SlipRatio", &DriveParams::TVParams::SlipRatio, 0.0f, 1.0f},
        {"SlipDownFactor", &DriveParams::TVParams::SlipDownFactor, 0.0f, 1.0f},

        // These are absolute torque units now (good). Keep them <= max torque ranges.
        {"SlipMinTorque", &DriveParams::TVParams::SlipMinTorque, 0.0f, 500.0f},
        {"SlipRecoverTorquePerTick", &DriveParams::TVParams::SlipRecoverTorquePerTick, 0.0f, 500.0f},

        {"WheelMinRPM", &DriveParams::TVParams::WheelMinRPM, 0.0f, 500.0f},
        {"TractionCorrectionASR", &DriveParams::TVParams::TractionCorrectionASR, 0.0f, 1.0f},
        {"TractionCorrectionABS", &DriveParams::TVParams::TractionCorrectionABS, 0.0f, 1.0f},

        {"maxRealisticAccel", &DriveParams::TVParams::maxRealisticAccel, 0.0f, 500.0f},
        {"maxRealisticDecel", &DriveParams::TVParams::maxRealisticDecel, 0.0f, 500.0f},

        // =========================================================================
        // Front/rear bias
        // =========================================================================
        {"DriveFrontShareLow", &DriveParams::TVParams::DriveFrontShareLow, 0.0f, 1.0f},
        {"DriveFrontShareHigh", &DriveParams::TVParams::DriveFrontShareHigh, 0.0f, 1.0f},
        {"BrakeFrontShareLow", &DriveParams::TVParams::BrakeFrontShareLow, 0.0f, 1.0f},
        {"BrakeFrontShareHigh", &DriveParams::TVParams::BrakeFrontShareHigh, 0.0f, 1.0f},

        {"FrontRearBiasFullTorqueDrive", &DriveParams::TVParams::FrontRearBiasFullTorqueDrive, 0.0f, 1000.0f},
        {"FrontRearBiasFullTorqueBrake", &DriveParams::TVParams::FrontRearBiasFullTorqueBrake, 0.0f, 1000.0f},

        // =========================================================================
        // Braking near standstill
        // =========================================================================
        {"AntiReversingSpeed", &DriveParams::TVParams::AntiReversingSpeed, 0.0f, 500.0f},
    };

    static constexpr uint16_t kTVParamCount =
        static_cast<uint16_t>(sizeof(kTVParams) / sizeof(kTVParams[0]));
} // namespace

bool Tuning::SetByID(DriveParams::TVParams* tv, uint16_t id, float value)
{
    if (!tv) return false;

    if (id >= kTVParamCount) return false;

    const auto &e = kTVParams[id];
    if ((value < e.lo) || (value > e.hi)) {
        return false;
    }
    
    tv->*(e.mem) = value;
    return true;
}

uint16_t Tuning::Count()
{
    return kTVParamCount;
}

const char *Tuning::Key(uint16_t id)
{
    if (id >= kTVParamCount)
        return nullptr;
    return kTVParams[id].key;
}

bool Tuning::GetByID(const DriveParams::TVParams &tv, uint16_t id, float &outValue)
{
    if (id >= kTVParamCount)
        return false;
    outValue = tv.*(kTVParams[id].mem);
    return true;
}

bool Tuning::GetLimitsByID(uint16_t id, float &outLo, float &outHi)
{
    if (id >= kTVParamCount)
        return false;
    outLo = kTVParams[id].lo;
    outHi = kTVParams[id].hi;
    return true;
}

bool Tuning::IdByKey(const char *key, uint16_t &outId)
{
    if (!key)
        return false;
    for (uint16_t i = 0; i < kTVParamCount; ++i)
    {
        if (std::strcmp(kTVParams[i].key, key) == 0)
        {
            outId = i;
            return true;
        }
    }
    return false;
}