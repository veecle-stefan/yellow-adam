#pragma once
#include <cstdint>
#include <math.h>

// DriveParams:
// A single, explicit container for all drive algorithm parameters.
// - Defaults are centralized here
// - Can be instantiated and tuned live without scattering values into VehicleState.
struct DriveParams
{

    // Speed limiter fade band (soft region to avoid harsh clamp).
    // Default: 30.0
    float SpeedLimiterFadeBand = 30.f;

    // -----------------------------------------------------------------------------
    // Brakes / tap logic
    // -----------------------------------------------------------------------------
    struct BrakesParams
    {
        // "Anti-reversing" speed threshold (feedback speed units).
        // Used to fade braking torque to zero as |speed| approaches 0 to avoid reversing.
        // Default: 100 (speed units from feedback)
        float AntiReversingSpeed = 100.f;
    } Brakes;

    // -----------------------------------------------------------------------------
    // Safety thresholds (scaled!)
    // -----------------------------------------------------------------------------
    struct ControllerSafetyParams
    {
        // Battery voltage hard-off threshold.
        // Default: 3300 => 33.00 V
        int16_t VoltageOff = 3300;

        // Battery voltage warning threshold.
        // Default: 3500 => 35.00 V
        int16_t VoltageWarn = 3500;

        // Board temperature hard-off threshold.
        // Default: 600 => 60.0°C
        int16_t TempOff = 600;

        // Board temperature warning threshold.
        // Default: 500 => 50.0°C
        int16_t TempWarn = 500;
    } Controller;

    // -----------------------------------------------------------------------------
    // Torque Vectoring / steering / traction control
    // -----------------------------------------------------------------------------
    struct TVParams
    {
        // Maximum allowed change of steering-related torque per control tick,
        // expressed as a fraction of MaxOutputLimit.
        // Limits how fast torque steer / yaw assist can ramp to avoid mechanical shocks.
        // Default: 0.3
        float MaxTorquePerTick = 0.5f;

        // Front axle steering torque gain at standstill / very low speed.
        // Used to overcome rack centering spring and static friction.
        // Default: 1.0
        float SteerTorqueLowFactor = 1.f;

        // Front axle steering torque gain at higher vehicle speeds.
        // Reduced on purpose to avoid twitchy steering and oscillations.
        // Default: 0.5
        float SteerTorqueHighFactor = 0.7f;

        // Speed at which SteerTorqueHighFactor is fully reached.
        // Below this, steering gain interpolates linearly between Low and High factors.
        // Default: 50.0
        float SteerTorqueHighSpeed = 30.f;

        // Rear axle yaw assist fade-in based on vehicle speed.
        // Below RearFadeSpeed0: rear yaw assist disabled (standstill / noise region).
        // Default: 15.0
        float RearFadeSpeed0 = 15.f;

        // Above RearFadeSpeed1: rear yaw assist fully enabled.
        // Between Speed0 and Speed1, effect ramps in linearly.
        // Default: 30.0
        float RearFadeSpeed1 = 30.f;

        // Rear axle yaw assist fade-in based on longitudinal driver intent (throttle or brake),
        // expressed as fraction of MaxOutputLimit.
        // Below this: rear yaw assist disabled to avoid jitter at zero throttle.
        // Default: 0.05
        float RearFadeThrottle0 = 0.05f;

        // Above this throttle/brake magnitude: rear yaw assist fully enabled.
        // Between Throttle0 and Throttle1, effect ramps in linearly.
        // Default: 0.30
        float RearFadeThrottle1 = 0.3f;

        // Maximum differential steering torque applied on the front axle.
        // Primary steering actuator (torque steer) for your rack.
        // Default: 250.0
        float SteerTorqueFront = 220.f;

        // Maximum yaw-assist differential torque applied on the rear axle.
        // Stabilizes / assists turning but is NOT a steering actuator.
        // Default: 200.0
        float SteerTorqueRear = 260.f;

        // -------------------------------------------------------------------------
        // ABS / ASR – very simple reactive slip control (per-wheel scaling)
        // -------------------------------------------------------------------------

        // Relative speed deviation threshold to detect slip. Example: 0.20 = 20%.
        // ASR: wheel spins > +SlipRatio faster than reference under positive torque.
        // ABS: wheel slows > -SlipRatio slower than reference under negative torque.
        // Default: 0.20
        float SlipRatio = 0.20f;

        // Multiplicative torque reduction applied when slip is detected.
        // Repeated slip compounds (e.g. 0.7^n).
        // Default: 0.70
        float SlipDownFactor = 0.70f;

        // Lower bound for slip scaling (never reduce below this).
        // Prevents permanent "torque = 0" and allows recovery.
        // Default: 0.25
        float SlipMinScale = 0.25f;

        // Recovery rate per tick when no slip is detected (towards 1.0).
        // Default: 0.10
        float SlipRecoverPerTick = 0.20f;

        // Minimum reference speed to activate slip detection (noise floor).
        // Default: 10.0
        float SlipSpeedEps = 20.f;

        // Deadband for slip detection based on requested torque magnitude.
        // Ignore slip logic if |torqueCmd| < SlipTorqueEps * MaxOutputLimit.
        // Helps avoid ABS/ASR fighting small steering differentials/noise.
        // Default: 0.1
        float SlipTorqueEps = 0.1f;

        // -------------------------------------------------------------------------
        // Axle torque bias (load transfer compensation)
        // -------------------------------------------------------------------------

        // Under acceleration: front share ramps from Low -> High as |throttle| increases.
        // Default: 0.55 -> 0.30 (more rear at hard accel)
        float DriveFrontShareLow = 0.55f;
        float DriveFrontShareHigh = 0.30f;

        // Under braking: front share ramps from Low -> High as |brake| increases.
        // Default: 0.60 -> 0.80 (more front at hard brake)
        float BrakeFrontShareLow = 0.60f;
        float BrakeFrontShareHigh = 0.80f;

        // Normalization point for the ramp above (fraction of max torque).
        // a = clamp(|throttle| / (BiasHighThrottle * MaxOutputLimit), 0..1)
        // Default: 0.6
        float BiasHighThrottle = 0.6f;

    } TV;

    // Convenience: constexpr-like default instance (still mutable at runtime).
    static DriveParams Defaults() { return DriveParams{}; }
};

// -----------------------------------------------------------------------------
// Tuning (TV only)
// -----------------------------------------------------------------------------
// Thin utility for live tuning. Uses stable numeric IDs (table index).
// This is queue-friendly and avoids passing strings across tasks.
//
class Tuning
{
public:
    // Set TV param by numeric id (table index). Clamps inside.
    // Returns false if id is out of range.
    static bool SetByID(DriveParams::TVParams* tv, uint16_t id, float value);

    // Returns number of tunable TV params (IDs range: [0..Count-1]).
    static uint16_t Count();

    // Returns the human-readable key for a param id, or nullptr if invalid.
    static const char *Key(uint16_t id);

    // Reads the current value of a param by id. Returns false if invalid id.
    static bool GetByID(const DriveParams::TVParams &tv, uint16_t id, float &outValue);

    // Reads min/max limits for a param id. Returns false if invalid id.
    static bool GetLimitsByID(uint16_t id, float &outLo, float &outHi);

    // Maps a key string to an id (so WS handler can convert key->id before queueing).
    static bool IdByKey(const char *key, uint16_t &outId);
};