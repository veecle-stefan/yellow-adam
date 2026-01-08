// driveParams.cpp
#include "driveParams.h"

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
    static constexpr TVParamEntry kTVParams[] = {
        // ----- Torque vectoring / steering gains -----
        {"MaxTorquePerTick", &DriveParams::TVParams::MaxTorquePerTick, 0.0f, 1.0f},
        {"SteerTorqueLowFactor", &DriveParams::TVParams::SteerTorqueLowFactor, 0.0f, 1.0f},
        {"SteerTorqueHighFactor", &DriveParams::TVParams::SteerTorqueHighFactor, 0.0f, 1.0f},
        {"SteerTorqueHighSpeed", &DriveParams::TVParams::SteerTorqueHighSpeed, 0.0f, 500.0f},

        {"RearFadeSpeed0", &DriveParams::TVParams::RearFadeSpeed0, 0.0f, 500.0f},
        {"RearFadeSpeed1", &DriveParams::TVParams::RearFadeSpeed1, 0.0f, 500.0f},
        {"RearFadeThrottle0", &DriveParams::TVParams::RearFadeThrottle0, 0.0f, 1.0f},
        {"RearFadeThrottle1", &DriveParams::TVParams::RearFadeThrottle1, 0.0f, 1.0f},

        {"SteerTorqueFront", &DriveParams::TVParams::SteerTorqueFront, 0.0f, 1000.0f},
        {"SteerTorqueRear", &DriveParams::TVParams::SteerTorqueRear, 0.0f, 1000.0f},

        // ----- ABS/ASR slip control -----
        {"SlipRatio", &DriveParams::TVParams::SlipRatio, 0.0f, 1.0f},
        {"SlipDownFactor", &DriveParams::TVParams::SlipDownFactor, 0.0f, 1.0f},
        {"SlipMinScale", &DriveParams::TVParams::SlipMinScale, 0.0f, 1.0f},
        {"SlipRecoverPerTick", &DriveParams::TVParams::SlipRecoverPerTick, 0.0f, 1.0f},
        {"SlipSpeedEps", &DriveParams::TVParams::SlipSpeedEps, 0.0f, 500.0f},
        {"SlipTorqueEps", &DriveParams::TVParams::SlipTorqueEps, 0.0f, 1.0f},

        // ----- Front/rear bias -----
        {"DriveFrontShareLow", &DriveParams::TVParams::DriveFrontShareLow, 0.0f, 1.0f},
        {"DriveFrontShareHigh", &DriveParams::TVParams::DriveFrontShareHigh, 0.0f, 1.0f},
        {"BrakeFrontShareLow", &DriveParams::TVParams::BrakeFrontShareLow, 0.0f, 1.0f},
        {"BrakeFrontShareHigh", &DriveParams::TVParams::BrakeFrontShareHigh, 0.0f, 1.0f},
        {"BiasHighThrottle", &DriveParams::TVParams::BiasHighThrottle, 0.0f, 5.0f},
    };

    static constexpr uint16_t kTVParamCount =
        static_cast<uint16_t>(sizeof(kTVParams) / sizeof(kTVParams[0]));
} // namespace

bool Tuning::SetByID(DriveParams::TVParams* tv, uint16_t id, float value)
{
    if (!tv) return false;

    if (id >= kTVParamCount) return false;

    const auto &e = kTVParams[id];
    value = clampf(value, e.lo, e.hi);
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