#include "torquevectoring.h"

struct WheelSpeed
{
    float absSpeed;
    int index;
};

static inline void sortSmallN(WheelSpeed *x, int n)
{
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (x[j].absSpeed < x[i].absSpeed)
                std::swap(x[i], x[j]);
}

static inline float rateLimit(float target, float last, float maxDelta)
{
    const float d = target - last;
    if (d > maxDelta)
        return last + maxDelta;
    if (d < -maxDelta)
        return last - maxDelta;
    return target;
}

// Helper: sign of a wheel speed in motor coords
static inline int wheelSign(float w, float vEps)
{
    if (w > vEps)  return +1;
    if (w < -vEps) return -1;
    return 0;
}

// Helper: tighten the "assist" side of the torque envelope for a wheel
static inline void tightenAssistCap(
    float cmdAtReading, float assistSign, float minT, float downFactor,
    float& capFwd, float& capRev)
{
    const float tSlip = std::fabs(cmdAtReading);
    const float target = std::max(minT, tSlip * downFactor);
    if (assistSign > 0.f)
        capFwd = std::min(capFwd, target);
    else
        capRev = std::max(capRev, -target);
}

// Helper: tighten the "oppose" side of the torque envelope for a wheel
static inline void tightenOpposeCap(
    float cmdAtReading, float assistSign, float minT, float downFactor,
    float& capFwd, float& capRev)
{
    const float tSlip = std::fabs(cmdAtReading);
    const float target = std::max(minT, tSlip * downFactor);
    if (assistSign > 0.f)
        capRev = std::max(capRev, -target);
    else
        capFwd = std::min(capFwd, target);
}

// Helper: compute speed limiter cap based on speed
static inline float capFromSpeed(float vForCap, float vStart, float vMax, float maxTorque)
{
    if (vForCap <= vStart) return maxTorque;
    if (vForCap >= vMax)   return 0.f;
    const float u = (vForCap - vStart) / (vMax - vStart);
    return maxTorque * (1.f - std::clamp(u, 0.f, 1.f));
}

// Helper: compute front steering differential request
static inline float computeFrontSteeringDiff(
    float steering, float vFrontAbs,
    float steerTorqueHighSpeed, float steerTorqueLowFactor,
    float steerTorqueHighFactor, float steerTorqueFront)
{
    float uf = (steerTorqueHighSpeed > 1.f) ? (vFrontAbs / steerTorqueHighSpeed) : 1.f;
    uf = std::clamp(uf, 0.f, 1.f);
    const float kSteer = std::lerp(steerTorqueLowFactor, steerTorqueHighFactor, uf);

    // non-linear (quadratic) shaping: softer near center, still reaches max at full stick
    const float sAbs = std::fabs(steering);
    const float shaped = sAbs * sAbs; // use sAbs^2
    return std::copysign(shaped * steerTorqueFront * kSteer, steering);
}

// Helper: compute rear yaw assist differential request
static inline float computeRearYawAssist(
    float steering, float absS, float vRearAbs, float rearEffAbs,
    float rearFadeSpeed0, float rearFadeSpeed1,
    float rearFadeTorque0, float rearFadeTorque1,
    float steerTorqueRear)
{
    float us = 1.f;
    if (rearFadeSpeed1 > rearFadeSpeed0) {
        us = (vRearAbs - rearFadeSpeed0) / (rearFadeSpeed1 - rearFadeSpeed0);
        us = std::clamp(us, 0.f, 1.f);
    }

    float uLong = 1.f;
    if (rearFadeTorque1 > rearFadeTorque0) {
        uLong = (rearEffAbs - rearFadeTorque0) / (rearFadeTorque1 - rearFadeTorque0);
        uLong = std::clamp(uLong, 0.f, 1.f);
    }
    // conservative product: both speed and rear-longitudinal conditions must be met
    const float fadeRear = us * uLong;   // both in [0..1]
    const float oppMag = absS * steerTorqueRear;
    return std::copysign(oppMag * fadeRear, steering);
}

// ============================================================================
// SenseContext: Internal working data for Sense() sub-methods
// - Not part of the public Sense() output (SenseData)
// - Lives only within Sense() scope
// ============================================================================
struct SenseContext
{
    // Motion intent derived from user input
    bool accel = false;
    bool brake = false;
    
    // Sorted wheel speeds (ascending by absSpeed)
    WheelSpeed sorted[4] = {};
    uint8_t numValid = 0;
};

// ============================================================================
// TorqueVectoring::ReadAxleSpeeds()
// - Reads wheel speeds for a single axle from motor feedback into SenseData
// - Populates: ok[], w[], wAbs[], cmdAtReading[] for the given index pair
// ============================================================================
void TorqueVectoring::ReadAxleSpeeds(const std::optional<Axle::HistoryFrame>& fb, SenseData& sd, int indexShift)
{
    if (!fb) {
        return;
    }

    sd.ok[indexShift] = true;
    sd.ok[indexShift + 1] = true;

    sd.w[indexShift] = static_cast<float>(fb->sample.speedL_meas);
    sd.w[indexShift + 1] = static_cast<float>(fb->sample.speedR_meas);
    sd.wAbs[indexShift] = fabs(sd.w[indexShift]);
    sd.wAbs[indexShift + 1] = fabs(sd.w[indexShift + 1]);
    sd.cmdAtReading[indexShift] = static_cast<float>(fb->sample.cmdL);
    sd.cmdAtReading[indexShift + 1] = static_cast<float>(fb->sample.cmdR);
}

// ============================================================================
// TorqueVectoring::ReadWheelSpeeds()
// - Reads wheel speeds from motor feedback into SenseData
// - Populates: ok[], w[], wAbs[], cmdAtReading[]
// ============================================================================
void TorqueVectoring::ReadWheelSpeeds(const TickContext& ctx, SenseData& sd)
{
    ReadAxleSpeeds(ctx.currFront, sd, 0);
    ReadAxleSpeeds(ctx.currRear, sd, 2);
}

// ============================================================================
// TorqueVectoring::EstimateVehicleMotion()
// - Computes a robust vehicleSpeedAbs reference
//   * accel: use 2nd smallest abs wheel speed  (reject fast spin outliers)
//   * brake: use 2nd largest  abs wheel speed  (reject locked wheel outliers)
//   * coast: use median abs wheel speed
// - Persists m_vehicleSpeedAbs and rejects physically impossible jumps
// - Populates sc.sorted[], sc.numValid for downstream use
// ============================================================================
void TorqueVectoring::EstimateVehicleMotion(const TickContext& ctx, SenseData& sd, SenseContext& sc)
{
    // Collect valid abs speeds for robust statistics WITH indices
    sc.numValid = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (sd.ok[i])
        {
            sc.sorted[sc.numValid].absSpeed = sd.wAbs[i];
            sc.sorted[sc.numValid].index = i;
            ++sc.numValid;
        }
    }

    sortSmallN(sc.sorted, sc.numValid);

    const uint8_t n = sc.numValid;
    const float vEps = ctx.params->TV.WheelMinRPM;

    // Robust measured reference speed v_ref_meas (abs) + motionSign (motor coords)
    float v_ref_meas = 0.f;
    float motionSign = 0.f; // -1,0,+1
    int refWheelIdx = -1;

    if (n == 0)
    {
        v_ref_meas = 0.f;
        refWheelIdx = -1;
    }
    else
    {
        if (sc.accel)
        {
            // 2nd smallest (reject fast spinner); for n==1 fallback to smallest
            const int pick = (n >= 2) ? 1 : 0;
            v_ref_meas = sc.sorted[pick].absSpeed;
            refWheelIdx = sc.sorted[pick].index;
        }
        else if (sc.brake)
        {
            // 2nd largest (reject locked); for n==1 fallback to largest
            const int pick = (n >= 2) ? (n - 2) : (n - 1);
            v_ref_meas = sc.sorted[pick].absSpeed;
            refWheelIdx = sc.sorted[pick].index;
        }
        else
        {
            // Coasting: median abs speed
            if (n == 1)
            {
                v_ref_meas = sc.sorted[0].absSpeed;
                refWheelIdx = sc.sorted[0].index;
            }
            else if (n == 2)
            {
                v_ref_meas = 0.5f * (sc.sorted[0].absSpeed + sc.sorted[1].absSpeed);

                // SIGN: only if both wheels agree, else unknown
                const int s0 = wheelSign(sd.w[sc.sorted[0].index], vEps);
                const int s1 = wheelSign(sd.w[sc.sorted[1].index], vEps);
                if (s0 != 0 && s0 == s1)
                    refWheelIdx = sc.sorted[0].index; // any of them is fine; they agree
                else
                    refWheelIdx = -1; // force fallback
            }
            else if (n == 3)
            {
                v_ref_meas = sc.sorted[1].absSpeed;
                refWheelIdx = sc.sorted[1].index;
            }
            else // n == 4
            {
                v_ref_meas = 0.5f * (sc.sorted[1].absSpeed + sc.sorted[2].absSpeed);

                // SIGN: middle-two must agree, else unknown
                const int s1 = wheelSign(sd.w[sc.sorted[1].index], vEps);
                const int s2 = wheelSign(sd.w[sc.sorted[2].index], vEps);
                if (s1 != 0 && s1 == s2)
                    refWheelIdx = sc.sorted[1].index; // any of the two is fine
                else
                    refWheelIdx = -1; // force fallback
            }
        }
    }

    // Determine motionSign from chosen reference wheel (if any)
    if (refWheelIdx >= 0 && v_ref_meas > vEps)
    {
        const float wref = sd.w[refWheelIdx];
        motionSign = (wref > 0.f) ? +1.f : -1.f;
    }
    else
    {
        motionSign = 0.f; // unknown / standstill / disagreement -> fallback later
    }

    // --- Standstill handling ---
    // If we're basically standing still, keep a stable 0 reference and avoid "invented" speed.

    if (v_ref_meas < vEps)
    {
        if (sc.accel)
        {
            // At standstill under acceleration: trust the slow end hard.
            if (n > 0)
                v_ref_meas = sc.sorted[0].absSpeed; // smallest abs speed
            // motionSign stays as computed (may be 0) -> fallback logic downstream
        }
        else // if braking or coasting
        {
            v_ref_meas = 0.f;
            motionSign = 0.f; // explicit: at standstill braking, don't claim a direction
        }
    }

    // Rate-limit vehicle speed changes to reject physically impossible jumps
    const float dv = v_ref_meas - m_vehicleSpeedAbs;
    if (dv > ctx.params->TV.maxRealisticAccel)
    {
        m_vehicleSpeedAbs += ctx.params->TV.maxRealisticAccel;
    }
    else if (dv < -ctx.params->TV.maxRealisticDecel)
    {
        m_vehicleSpeedAbs -= ctx.params->TV.maxRealisticDecel;
    }
    else
    {
        m_vehicleSpeedAbs = v_ref_meas;
    }

    // Output for this tick
    sd.vehicleSpeedAbs = m_vehicleSpeedAbs;
    sd.motionSign = motionSign; // motor-coord sign; downstream can fallback if 0
    sd.motionSignAlongGear = (fabs(sd.motionSign) > 0.5f) ? (sd.motionSign * sd.gearDir) : 0.f;
}

// ============================================================================
// TorqueVectoring::UpdateSlipEnvelopes()
// - ASR/ABS torque envelopes per wheel (persistent, updated each tick)
// - Recovers envelopes towards hard limits each tick
// - Detects slip conditions and tightens envelopes accordingly
// - Outputs: sd.capFwd[], sd.capRev[]
// ============================================================================
void TorqueVectoring::UpdateSlipEnvelopes(const TickContext& ctx, SenseData& sd, const SenseContext& sc)
{
    const auto& TV = ctx.params->TV;
    const float vRef = m_vehicleSpeedAbs;
    const float vEps = TV.WheelMinRPM;
    const float minT = TV.SlipMinTorque;
    const float hard = std::max(TV.maxTorqueDrive, TV.maxTorqueBrake);

    // Initialize caps on first run
    if (!m_capsInit)
    {
        for (int i = 0; i < 4; i++)
        {
            m_capFwd[i] = +hard;
            m_capRev[i] = -hard;
        }
        m_capsInit = true;
    }

    // Recover towards +/- hard
    for (int i = 0; i < 4; i++)
    {
        m_capFwd[i] = std::min(m_capFwd[i] + TV.SlipRecoverTorquePerTick, +hard);
        m_capFwd[i] = std::max(m_capFwd[i], minT);

        m_capRev[i] = std::max(m_capRev[i] - TV.SlipRecoverTorquePerTick, -hard);
        m_capRev[i] = std::min(m_capRev[i], -minT); // keep negative
    }

    // Anchor wheel: at least one wheel is basically not moving.
    // This is the condition under which "standstill slip" is meaningful.
    const bool hasAnchor = (sc.numValid > 0) && (sc.sorted[0].absSpeed < vEps);
    const bool moving = (std::fabs(sd.motionSign) > 0.5f);
    const float assistSign = moving ? sd.motionSign : sd.gearDir;
    
    // Slip detection & envelope tightening
    for (int i = 0; i < 4; ++i)
    {
        const float wi = sd.wAbs[i];
        const float ci = sd.cmdAtReading[i];

        // drive = torque that helps motion in gear direction
        // propulsive/assist torque = helps motion (or helps intended direction at standstill)
        const bool cmdAssistStandstill = (ci * sd.gearDir) > +minT;
        const float effMotionSign = moving ? sd.motionSign : sd.gearDir; // fallback
        const bool cmdAssistMoving = (ci * effMotionSign) > +minT;
        const bool cmdOpposeMoving = (ci * effMotionSign) < -minT;
        bool slipAccelerate = false;
        bool slipOppose = false;

        if (vRef < vEps)
        {
            // Standstill:
            // slip detection only makes sense if we have an anchor wheel.
            // Otherwise (all wheels already moving a bit), don't "invent" slip decisions here.
            if (hasAnchor)
            {
                // ASR at standstill: if we command meaningful drive torque and this wheel moves -> free-spin
                if (cmdAssistStandstill && wi > vEps)
                    slipAccelerate = true;

                // ABS at standstill is not meaningful for your use-case (vehicleSpeed~0).
                // Prevent tightening brake envelopes just because a wheel is still spinning down.
                // If you want a "wheel spin-down brake limiter" later, do it explicitly elsewhere.
                slipOppose = false;
            }
            else
            {
                // No anchor => ambiguous whether we're truly at standstill.
                // Skip standstill tightening and let the moving-state ratio logic handle it once vRef rises.
                slipAccelerate = false;
                slipOppose = false;
            }
        }
        else
        {
            // braking/opposing torque = opposes motion (only meaningful if moving)
            const float hi = vRef * (1.f + TV.SlipRatio);
            const float lo = vRef * (1.f - TV.SlipRatio);

            // ASR: wheel too fast under positive commanded torque
            if (cmdAssistMoving && wi > hi)
                slipAccelerate = true;

            // ABS: wheel too slow under negative commanded torque
            if (cmdOpposeMoving && wi < lo)
                slipOppose = true;
        }

        if (slipAccelerate)
            tightenAssistCap(sd.cmdAtReading[i], assistSign, minT, TV.SlipDownFactor, m_capFwd[i], m_capRev[i]);
        if (slipOppose)
            tightenOpposeCap(sd.cmdAtReading[i], assistSign, minT, TV.SlipDownFactor, m_capFwd[i], m_capRev[i]);
    }

    // Output for this tick (clamped to CURRENT live limits)
    for (int i = 0; i < 4; ++i)
    {
        sd.capFwd[i] = std::clamp(m_capFwd[i], minT, +hard);
        sd.capRev[i] = std::clamp(m_capRev[i], -hard, -minT);
    }
}

// ============================================================================
// TorqueVectoring::ApplyBrakeFadeCaps()
// - Tightens brake-side envelope when wheel is near standstill
// - Prevents braking torque from reversing a stopped wheel
// - Works in motor coordinates: w > 0 → brake is capRev, w < 0 → brake is capFwd
// - At true standstill (|w| < vEps), clamps brake side to ZERO (not minT)
// ============================================================================
void TorqueVectoring::ApplyBrakeFadeCaps(const TickContext &ctx, SenseData &sd)
{
    const auto &TV = ctx.params->TV;
    const float vFade = TV.AntiReversingSpeed;

    if (vFade <= 1.f)
        return;

    const float vEps = TV.WheelMinRPM;

    for (int i = 0; i < 4; ++i)
    {
        const float wi = sd.wAbs[i];
        const float ws = sd.w[i];

        // First: if wheel is spinning OPPOSITE to gear direction, block the accelerating side
        // This is a hard safety rule, independent of fade logic
        if (ws * sd.gearDir < -vEps)
        {
            // Wheel going "wrong way" relative to gear
            if (ws < 0.f)
            {
                // Wheel backward in D: block negative torque (would accelerate backward)
                sd.capRev[i] = 0.f;
            }
            else
            {
                // Wheel forward in R: block positive torque (would accelerate forward)
                sd.capFwd[i] = 0.f;
            }
            // Also fade the brake side if near standstill
        }

        if (wi >= vFade)
            continue;

        const float fadeScale = wi / vFade;

        if (ws > vEps)
        {
            sd.capRev[i] = sd.capRev[i] * fadeScale;
        }
        else if (ws < -vEps)
        {
            sd.capFwd[i] = sd.capFwd[i] * fadeScale;
        }
        else
        {
            if (sd.gearDir > 0.f)
                sd.capRev[i] = 0.f;
            else
                sd.capFwd[i] = 0.f;
        }
    }
}

TorqueVectoring::SenseData TorqueVectoring::Sense(const TickContext &ctx, Gear currGear)
{
    SenseData sd{};
    sd.gearDir = (currGear == Gear::D) ? +1.f : -1.f;

    // Initialize internal working context
    SenseContext sc{};
    sc.accel = (ctx.user.throttle > 0);
    sc.brake = (ctx.user.throttle < 0);

    // === STAGE 1: Read Wheel Speeds ===
    ReadWheelSpeeds(ctx, sd);

    // === STAGE 2: Estimate Vehicle Motion ===
    EstimateVehicleMotion(ctx, sd, sc);

    // === STAGE 3: Update Slip Envelopes ===
    UpdateSlipEnvelopes(ctx, sd, sc);

    // === STAGE 4: Apply Speed Limiter ===
    ApplySpeedLimiter(ctx, currGear, sd);

    // === STAGE 5: Apply Brake Fade Caps ===
    if (sc.brake)
        ApplyBrakeFadeCaps(ctx, sd);

    return sd;
}

// ============================================================================
// TorqueVectoring::ApplySpeedLimiter()
// - Speed limiter as an envelope modifier (NOT throttle scaling)
// - Prevents overspeed without oscillating
// - Tightens sd.capFwd[i] (drive torque) when near/above speed limit
// - Does NOT touch braking envelopes (capRev)
// ============================================================================
void TorqueVectoring::ApplySpeedLimiter(const TickContext& ctx, Gear currGear, SenseData& sd)
{
    const auto& TV = ctx.params->TV;
    const float band = TV.SpeedLimiterFadeBand;
    if (band <= 1e-3f)
        return;

    const float vRef = sd.vehicleSpeedAbs;
    const float vMax = (currGear == Gear::D) ? TV.maxSpeedFwd : TV.maxSpeedRev;
    const float vStart = vMax - band;
    const float hiOutlier = vRef * (1.f + TV.SlipRatio);
    
    for (int i = 0; i < 4; ++i)
    {
        // Use vehicle reference unless this wheel is clearly an outlier near the cap.
        float vForCap = vRef;

        const float wi = sd.wAbs[i];

        // Only treat wheel speed as authoritative when:
        //  1) we're moving (vRef above noise floor)
        //  2) the wheel is significantly faster than the vehicle (outlier)
        //  3) we're in/near the limiter band (otherwise don't mess with it)
        
        if (vRef > TV.WheelMinRPM)
        {
            if (wi > hiOutlier && wi > vStart)
                vForCap = wi;
        }

        // Tightens the propulsion-side envelope (sign = gearDir)
        // Does not touch the opposite-side envelope (used for braking / opposing torque)
        const float capTarget = capFromSpeed(vForCap, vStart, vMax, TV.maxTorqueDrive);

        if (sd.gearDir > 0.f)
        {
            sd.capFwd[i] = std::min(sd.capFwd[i], capTarget);
            sd.capFwd[i] = std::max(sd.capFwd[i], TV.SlipMinTorque);
        }
        else
        {
            sd.capRev[i] = std::max(sd.capRev[i], -capTarget);
            sd.capRev[i] = std::min(sd.capRev[i], -TV.SlipMinTorque);
        }
    }
}

TorqueVectoring::PairSolve TorqueVectoring::SolvePairCaps(
    float TcReq, float TdReq,
    float capNegL, float capPosL,
    float capNegR, float capPosR,
    float k) // 0 = no correction, 1 = full traction correction
{
    // Traction-correcting Td: shifts torque toward the wheel with more headroom
    // Positive when L has more headroom (capPosL > capPosR) → adds to L, subtracts from R
    const float TdTraction = 0.5f * (capPosL - capPosR);

    // Blend between driver request and traction correction
    const float TdTarget = TdReq + k * TdTraction;

    // Feasible Td range
    const float TdMin = 0.5f * (capNegL - capPosR);
    const float TdMax = 0.5f * (capPosL - capNegR);

    float Td = std::clamp(TdTarget, TdMin, TdMax);

    // Given that Td, find feasible Tc range
    const float TcMin = std::max(capNegL - Td, capNegR + Td);
    const float TcMax = std::min(capPosL - Td, capPosR + Td);

    float Tc = std::clamp(TcReq, TcMin, TcMax);

    PairSolve o{};
    o.Tc = Tc;
    o.Td = Td;
    o.TcMin = TcMin;
    o.TcMax = TcMax;
    o.L = Tc + Td;
    o.R = Tc - Td;
    return o;
}

float TorqueVectoring::AxleSpeedAbs(const SenseData& s, Wheel L, Wheel R)
{
    const bool okL = s.ok[L], okR = s.ok[R];
    if (okL && okR) return 0.5f * (s.wAbs[L] + s.wAbs[R]);
    if (okL) return s.wAbs[L];
    if (okR) return s.wAbs[R];
    return 0.f;
}


// -----------------
// ComputeTrajectoryIntent()
// -----------------
// Computes the driver's trajectory intent:
// - Longitudinal torque request split between front/rear axles
// - Front steering differential
// - Rear yaw assist differential
// - Rate limits applied to differentials
TorqueVectoring::TrajectoryIntent TorqueVectoring::ComputeTrajectoryIntent(
    const TickContext& ctx, const SenseData& sense, Gear currGear)
{
    const auto& TV = ctx.params->TV;

    // === Normalize user inputs ===
    const float s = static_cast<float>(ctx.user.steering) / 1000.0f;   // convention: -left, +right
    const float absS = std::fabs(s);

    const float maxDrive = TV.maxTorqueDrive;
    const float maxBrake = TV.maxTorqueBrake;

    // Requested magnitude in "torque units"
    const float maxT = (ctx.user.throttle >= 0) ? maxDrive : maxBrake;
    const float throttleInput = static_cast<float>(ctx.user.throttle) * maxT / 1000.0f;

    // === Longitudinal intent ===
    float Tc_total_req = 0.f;

    if (throttleInput >= 0.f) {
        // Drive in gear direction
        Tc_total_req = throttleInput;
    }
    else
    {
        const float bmag = -throttleInput;

        float msAlongGear = sense.motionSignAlongGear;
        if (std::fabs(msAlongGear) < 0.5f)
        {
            // At standstill: no active brake torque needed
            // HoldFlags will handle holding the vehicle in place
            Tc_total_req = 0.f;
        }
        else
        {
            // Moving: brake opposes motion
            Tc_total_req = (-msAlongGear) * bmag;
        }
    }

    // === Front/rear bias ===
    const float TcAbs = std::fabs(Tc_total_req);

    // Pick ramp end-point depending on drive vs brake
    const float full = (Tc_total_req >= 0.f)
                           ? TV.FrontRearBiasFullTorqueDrive
                           : TV.FrontRearBiasFullTorqueBrake;

    // uBias in [0..1]
    float uBias = (full > 1e-6f) ? (TcAbs / full) : 1.f;
    uBias = std::clamp(uBias, 0.f, 1.f);

    float frontShare = 0.5f;
    if (Tc_total_req >= 0.f)
        frontShare = std::lerp(TV.DriveFrontShareLow, TV.DriveFrontShareHigh, uBias);
    else
        frontShare = std::lerp(TV.BrakeFrontShareLow, TV.BrakeFrontShareHigh, uBias);

    frontShare = std::clamp(frontShare, 0.05f, 0.95f);

    float TcF_req = Tc_total_req * frontShare;
    float TcR_req = Tc_total_req * (1.f - frontShare);

    // === Front steering differential ===
    const float vFrontAbs = AxleSpeedAbs(sense, FL, FR);
    float TdF_req = computeFrontSteeringDiff(
        s, vFrontAbs,
        TV.SteerTorqueHighSpeed, TV.SteerTorqueLowFactor,
        TV.SteerTorqueHighFactor, TV.SteerTorqueFront);

    // === Rear yaw assist differential (only in D) ===
    float TdR_req = 0.f;
    if (currGear == Gear::D && absS > 0.001f)
    {
        const float vRearAbs = AxleSpeedAbs(sense, RL, RR);
        TdR_req = computeRearYawAssist(
            s, absS, vRearAbs, std::fabs(TcR_req),
            TV.RearFadeSpeed0, TV.RearFadeSpeed1,
            TV.RearFadeTorque0, TV.RearFadeTorque1,
            TV.SteerTorqueRear);
    }

    // === Rate limit steering/yaw differentials ===
    TdF_req = rateLimit(TdF_req, m_lastTdF, TV.MaxTorquePerTick);
    m_lastTdF = TdF_req;

    TdR_req = rateLimit(TdR_req, m_lastTdR, TV.MaxTorquePerTick);
    m_lastTdR = TdR_req;

    return { Tc_total_req, TcF_req, TcR_req, TdF_req, TdR_req };
}


// -----------------
// SolveWheelTorques()
// -----------------
// Solves for individual wheel torques under slip envelope constraints:
// - Converts trajectory intent to motor coordinates
// - Solves axle pairs prioritizing differential (steering) over common (longitudinal)
// - Rebalances longitudinal torque across axles when one hits its cap
// - Applies hard clamp to final values
TorqueVectoring::WheelTorques TorqueVectoring::SolveWheelTorques(
    const TickContext &ctx, const SenseData &sense, const TrajectoryIntent &intent)
{
    const auto &TV = ctx.params->TV;

    // Convert to motor coordinates
    const float TcF_req_m = sense.gearDir * intent.TcF;
    const float TcR_req_m = sense.gearDir * intent.TcR;

    // Td must NOT be flipped (otherwise steering mirrors in reverse)
    const float TdF_req_m = intent.TdF;
    const float TdR_req_m = intent.TdR;

    // Select traction correction factor based on accel vs brake
    const float kFront = (TcF_req_m * sense.gearDir > 0)
                             ? TV.TractionCorrectionASR
                             : TV.TractionCorrectionABS;
    const float kRear = (TcR_req_m * sense.gearDir > 0)
                            ? TV.TractionCorrectionASR
                            : TV.TractionCorrectionABS;

    // Solve axle pairs under caps
    PairSolve front = SolvePairCaps(
        TcF_req_m, TdF_req_m,
        sense.capRev[FL], sense.capFwd[FL],
        sense.capRev[FR], sense.capFwd[FR],
        kFront);

    PairSolve rear = SolvePairCaps(
        TcR_req_m, TdR_req_m,
        sense.capRev[RL], sense.capFwd[RL],
        sense.capRev[RR], sense.capFwd[RR],
        kRear);

    // No cross-axle rebalancing.
    // Rationale:
    // - Traction limits mean the surface can't support more force
    // - Shifting torque to another axle likely just causes that axle to slip too
    // - Understeer (reduced yaw moment) is the safe failure mode
    // - Each axle's SolvePairCaps already preserves Td (steering) over Tc (thrust)

    // Hard clamp (motor coords)
    const float hard = std::max(TV.maxTorqueDrive, TV.maxTorqueBrake);

    WheelTorques out;
    out.fl = std::clamp(front.L, -hard, +hard);
    out.fr = std::clamp(front.R, -hard, +hard);
    out.rl = std::clamp(rear.L, -hard, +hard);
    out.rr = std::clamp(rear.R, -hard, +hard);
    return out;
}

// ============================================================================
// TorqueVectoring::ComputeHoldFlags()
// - Computes per-wheel hold flags
// - Hold = vehicle AND wheel both near standstill while braking
// ============================================================================
TorqueVectoring::HoldFlags TorqueVectoring::ComputeHoldFlags(
    const TickContext &ctx,
    const SenseData &sense)
{
    const auto &TV = ctx.params->TV;
    const float vHold = TV.AntiReversingHoldSpeed;
    const bool braking = (ctx.user.throttle < 0);

    HoldFlags hold = {};

    if (braking)
    {
        hold.hFL = (sense.vehicleSpeedAbs < vHold && sense.wAbs[FL] < vHold);
        hold.hFR = (sense.vehicleSpeedAbs < vHold && sense.wAbs[FR] < vHold);
        hold.hRL = (sense.vehicleSpeedAbs < vHold && sense.wAbs[RL] < vHold);
        hold.hRR = (sense.vehicleSpeedAbs < vHold && sense.wAbs[RR] < vHold);
    }

    return hold;
}

TorqueVectoring::Torques TorqueVectoring::Compute(const TickContext &ctx, const Gear &currGear)
{
    if (currGear == Gear::N)
        return {};

    // === SENSE ===
    const SenseData sense = Sense(ctx, currGear);

    // === TRAJECTORY INTENT ===
    const TrajectoryIntent intent = ComputeTrajectoryIntent(ctx, sense, currGear);

    // === SOLVE CONSTRAINTS ===
    WheelTorques torques = SolveWheelTorques(ctx, sense, intent);

    // === HOLD FLAGS ===
    const HoldFlags hold = ComputeHoldFlags(ctx, sense);

    // === OUTPUT ===
    return {
        static_cast<int16_t>(torques.fl),
        static_cast<int16_t>(torques.fr),
        static_cast<int16_t>(torques.rl),
        static_cast<int16_t>(torques.rr),
        static_cast<int16_t>(sense.vehicleSpeedAbs),
        hold};
}

