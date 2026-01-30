#pragma once
#include <cstdint>
#include <math.h>

// DriveParams:
// A single, explicit container for all drive algorithm parameters.
// - Defaults are centralized here
// - Can be instantiated and tuned live without scattering values into VehicleState.
struct DriveParams
{

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
        // =========================================================================
        // Output / safety limits (profile)
        // =========================================================================

        // Absolute max commanded drive torque (+) per wheel (before envelopes).
        // Units: torque command units.
        float maxTorqueDrive = 250.f;

        // Absolute max commanded brake torque magnitude (|−|) per wheel (before envelopes).
        // Units: torque command units. (Negative torques are braking.)
        float maxTorqueBrake = 600.f;

        // Max allowed vehicle speed in forward / reverse (wheel-speed units).
        float maxSpeedFwd = 100.f;
        float maxSpeedRev = 60.f;

        // Soft fade band below maxSpeed* where speed limiting starts (wheel-speed units).
        // Example: maxSpeedRev=60, FadeBand=30 => start fading at 30..60.
        float SpeedLimiterFadeBand = 30.f;

        // =========================================================================
        // Steering torques (front torque-steer + rear yaw assist)
        // =========================================================================

        // Max allowed change (delta) of steering-related differential torque per tick.
        // Units: torque command units per control tick.
        float MaxTorquePerTick = 50.f;

        // Front axle differential torque limit (primary steering actuator).
        // Convention (your codebase): s>0 (right) => more torque on FL, opposing on FR.
        float SteerTorqueFront = 220.f;

        // Rear axle differential torque limit (yaw assist, not primary steering).
        float SteerTorqueRear = 260.f;

        // Steering gain at very low speed (rack stiction / centering spring).
        float SteerTorqueLowFactor = 1.f;

        // Steering gain at higher speed (reduce twitchiness).
        float SteerTorqueHighFactor = 0.7f;

        // Vehicle speed where HighFactor is fully reached (wheel-speed units).
        float SteerTorqueHighSpeed = 30.f;

        // Rear yaw assist fade-in by speed: below Speed0 off, above Speed1 full.
        float RearFadeSpeed0 = 15.f;
        float RearFadeSpeed1 = 30.f;

        // Rear yaw assist fade-in by longitudinal DRIVE torque request magnitude.
        // Units: torque command units (applied to |Tc_rear_req| / |Tc_total_req| logic depending on your implementation).
        float RearFadeTorque0 = 50.f;
        float RearFadeTorque1 = 150.f;

        // =========================================================================
        // Traction / ABS (per-wheel envelopes)
        // =========================================================================

        // Relative speed deviation threshold for slip detection (ratio).
        // Drive slip: wheel > vRef*(1+SlipRatio) under + torque.
        // Brake slip: wheel < vRef*(1−SlipRatio) under − torque.
        float SlipRatio = 0.20f;

        // Multiplicative envelope tightening when slip is detected.
        // capPos *= SlipDownFactor, capNeg *= SlipDownFactor (brake moves toward 0).
        float SlipDownFactor = 0.70f;

        // Minimum magnitude the envelope is allowed to shrink to.
        // Units: torque command units. (capPos >= SlipMinTorque, capNeg <= −SlipMinTorque)
        float SlipMinTorque = 50.f;

        // Absolute recovery added per tick when NOT slipping (toward live max limits).
        // Units: torque command units per tick.
        float SlipRecoverTorquePerTick = 50.f;

        // Minimum reference speed needed to use ratio-based slip detection (noise floor).
        // Units: wheel-speed units.
        float WheelMinRPM = 20.f;

        // Traction correction factor under ASR (acceleration slip).
        // 0 = equal torque L/R (conservative), 1 = full shift to gripping wheel (max thrust).
        // Higher values give more acceleration but may cause yaw when traction returns.
        float TractionCorrectionASR = 0.75f;

        // Traction correction factor under ABS (braking slip).
        // 0 = equal braking L/R (stable), 1 = full shift to gripping wheel (max decel).
        // Recommend keeping low (0..0.3) for directional stability under braking.
        float TractionCorrectionABS = 0.25f;

        // Maximum plausible change in the estimated vehicle speed per tick.
        // Units: wheel-speed units per tick. Used to reject outliers (“teleporting”).
        float maxRealisticAccel = 30.f;
        float maxRealisticDecel = 40.f;

        // =========================================================================
        // Front/rear torque bias (load transfer compensation)
        // =========================================================================

        // Front axle share under DRIVE: ramps from Low -> High as |Tc_total_req| increases.
        // (Low torque = more front, high torque = more rear) depending on chosen defaults.
        float DriveFrontShareLow = 0.55f;
        float DriveFrontShareHigh = 0.30f;

        // Front axle share under BRAKE: ramps from Low -> High as |Tc_total_req| increases.
        float BrakeFrontShareLow = 0.60f;
        float BrakeFrontShareHigh = 0.80f;

        // Torque magnitude where the front/rear share reaches the “High” setting (uBias=1).
        // Units: torque command units. Separate knobs because drive/brake ranges differ a lot.
        float FrontRearBiasFullTorqueDrive = 150.f;
        float FrontRearBiasFullTorqueBrake = 300.f;

        // =========================================================================
        // Braking near standstill (anti-reversing)
        // =========================================================================

        // Fade braking torque to zero as |wheel speed| approaches 0 to avoid reversing a stopped wheel.
        // Units: wheel-speed units. (Used by brakeScale = |w|/AntiReversingSpeed, clamped 0..1)
        float AntiReversingSpeed = 60.f;
        float AntiReversingHoldSpeed = 5.f;
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