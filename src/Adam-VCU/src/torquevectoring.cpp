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
    const float vEps = ctx.params->TV.SlipSpeedEps;

    // Helper: sign of a specific wheel in MOTOR coords
    auto wheelSign = [&](int idx) -> int
    {
        if (idx < 0)
            return 0;
        const float w = sd.w[idx];
        if (w > vEps)
            return +1;
        if (w < -vEps)
            return -1;
        return 0;
    };

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
                const int s0 = wheelSign(sc.sorted[0].index);
                const int s1 = wheelSign(sc.sorted[1].index);
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
                const int s1 = wheelSign(sc.sorted[1].index);
                const int s2 = wheelSign(sc.sorted[2].index);
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

    if (m_vehicleSpeedAbs < vEps)
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
    const float vRef = m_vehicleSpeedAbs;
    const float vEps = ctx.params->TV.SlipSpeedEps;
    const float slipRatio = ctx.params->TV.SlipRatio;
    const float down = ctx.params->TV.SlipDownFactor;
    const float recover = ctx.params->TV.SlipRecoverTorquePerTick;
    const float minT = ctx.params->TV.SlipMinTorque;
    const float hard = std::max(ctx.params->TV.maxTorqueDrive, ctx.params->TV.maxTorqueBrake);

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
        m_capFwd[i] = std::min(m_capFwd[i] + recover, +hard);
        m_capFwd[i] = std::max(m_capFwd[i], minT);

        m_capRev[i] = std::max(m_capRev[i] - recover, -hard);
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
            const float hi = vRef * (1.f + slipRatio);
            const float lo = vRef * (1.f - slipRatio);

            // ASR: wheel too fast under positive commanded torque
            if (cmdAssistMoving && wi > hi)
                slipAccelerate = true;

            // ABS: wheel too slow under negative commanded torque
            if (cmdOpposeMoving && wi < lo)
                slipOppose = true;
        }

        auto tightenAssist = [&](int i)
        {
            if (assistSign > 0.f)
            {
                m_capFwd[i] *= down;
                m_capFwd[i] = std::max(m_capFwd[i], minT);
            }
            else
            {
                m_capRev[i] *= down;
                m_capRev[i] = std::min(m_capRev[i], -minT);
            }
        };

        auto tightenOppose = [&](int i)
        {
            // opposeSign = -assistSign
            if (assistSign > 0.f)
            {
                m_capRev[i] *= down;
                m_capRev[i] = std::min(m_capRev[i], -minT);
            }
            else
            {
                m_capFwd[i] *= down;
                m_capFwd[i] = std::max(m_capFwd[i], minT);
            }
        };

        if (slipAccelerate)
            tightenAssist(i);
        if (slipOppose)
            tightenOppose(i);
    }

    // Output for this tick (clamped to CURRENT live limits)
    for (int i = 0; i < 4; ++i)
    {
        sd.capFwd[i] = std::clamp(m_capFwd[i], minT, +hard);
        sd.capRev[i] = std::clamp(m_capRev[i], -hard, -minT);
    }
}

// ============================================================================
// TorqueVectoring::Sense()
// - Orchestrates sensing sub-stages
// - Returns SenseData for use by Compute()
// ============================================================================
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
    const float band = ctx.params->TV.SpeedLimiterFadeBand;
    if (band <= 1e-3f)
        return;

    const float vRef = sd.vehicleSpeedAbs;
    const float vMax = (currGear == Gear::D) ? ctx.params->TV.maxSpeedFwd : ctx.params->TV.maxSpeedRev;
    const float vStart = vMax - band;
    const float vEps = ctx.params->TV.SlipSpeedEps;
    const float minT = ctx.params->TV.SlipMinTorque;

    // Outlier threshold uses existing ASR parameter (no new params)
    const float hiOutlier = vRef * (1.f + ctx.params->TV.SlipRatio);

    auto capFromSpeed = [&](float vForCap) -> float
    {
        if (vForCap <= vStart) return ctx.params->TV.maxTorqueDrive;
        if (vForCap >= vMax)   return 0.f;

        const float u = (vForCap - vStart) / band; // 0..1
        return ctx.params->TV.maxTorqueDrive * (1.f - std::clamp(u, 0.f, 1.f));
    };

    for (int i = 0; i < 4; ++i)
    {
        // Use vehicle reference unless this wheel is clearly an outlier near the cap.
        float vForCap = vRef;

        const float wi = sd.wAbs[i];

        // Only treat wheel speed as authoritative when:
        //  1) we're moving (vRef above noise floor)
        //  2) the wheel is significantly faster than the vehicle (outlier)
        //  3) we're in/near the limiter band (otherwise don't mess with it)
        if (vRef > vEps)
        {
            if (wi > hiOutlier && wi > vStart)
                vForCap = wi;
        }

        // Tightens the propulsion-side envelope (sign = gearDir)
        // Does not touch the opposite-side envelope (used for braking / opposing torque)
        const float capTarget = capFromSpeed(vForCap);

        if (sd.gearDir > 0.f)
        {
            sd.capFwd[i] = std::min(sd.capFwd[i], capTarget);
            sd.capFwd[i] = std::max(sd.capFwd[i], minT);
        }
        else
        {
            sd.capRev[i] = std::max(sd.capRev[i], -capTarget);
            sd.capRev[i] = std::min(sd.capRev[i], -minT);
        }
    }
}


TorqueVectoring::PairSolve TorqueVectoring::SolvePairCaps(
    float TcReq, float TdReq,
    float capNegL, float capPosL,
    float capNegR, float capPosR)
{
    // Feasible Td range so that Tc interval is non-empty
    const float TdMin = 0.5f * (capNegL - capPosR);
    const float TdMax = 0.5f * (capPosL - capNegR);

    float Td = std::clamp(TdReq, TdMin, TdMax);

    const float TcMin = std::max(capNegL - Td, capNegR + Td);
    const float TcMax = std::min(capPosL - Td, capPosR + Td);

    float Tc = std::clamp(TcReq, TcMin, TcMax);

    PairSolve o{};
    o.Tc = Tc; o.Td = Td;
    o.TcMin = TcMin; o.TcMax = TcMax;
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
// Compute()
// -----------------
TorqueVectoring::Torques TorqueVectoring::Compute(const TickContext& ctx, const Gear& currGear)
{
    Torques out{};
    if (currGear == Gear::N || !ctx.params) return out;

    const auto& TV = ctx.params->TV;

    // ========= Stage 0: Sense (vehicle speed + caps) =========
    const SenseData sense = Sense(ctx, currGear);
    out.vehicleSpeedAbs = sense.vehicleSpeedAbs;
    const float vRefAbs = sense.vehicleSpeedAbs;

    // ========= Stage 1: Normalize user inputs =========
    const float s = static_cast<float>(ctx.user.steering) / 1000.0f;   // convention: -left, +right
    const float absS = std::fabs(s);

    const float maxDrive = TV.maxTorqueDrive;
    const float maxBrake = TV.maxTorqueBrake;

    // requested magnitude in "torque units"
    const float maxT = (ctx.user.throttle >= 0) ? maxDrive : maxBrake;
    const float throttleInput = static_cast<float>(ctx.user.throttle) * maxT / 1000.0f;

    // ========= Stage 2: Trajectory intent (Tc_total + TdF + TdR) =========
    // Longitudinal intent:
    float Tc_total_req = 0.f;

    if (throttleInput >= 0.f) {
        // drive in gear direction
        Tc_total_req = throttleInput;
    }
    else
    {
        const float bmag = -throttleInput; // positive magnitude


        float brakeScale = 1.f;
        if (TV.AntiReversingSpeed > 1.f)
            brakeScale = std::clamp(vRefAbs / TV.AntiReversingSpeed, 0.f, 1.f);

        float msAlongGear = sense.motionSignAlongGear;
        if (std::fabs(msAlongGear) < 0.5f)
            msAlongGear = +1.f; // fallback: assume "moving along gear"
        Tc_total_req = (-msAlongGear) * bmag * brakeScale;
    }

    // front/rear bias based on |Tc_total_req| (absolute torque ramp)
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

    // front steering differential (primary steering actuator)
    {
        const float vFrontAbs = AxleSpeedAbs(sense, FL, FR);
        float uf = (TV.SteerTorqueHighSpeed > 1.f) ? (vFrontAbs / TV.SteerTorqueHighSpeed) : 1.f;
        uf = std::clamp(uf, 0.f, 1.f);

        // SIGN: s<0 => Td<0 => FR gets more torque
        //       s>0 => Td>0 => FL gets more torque
        // Pair convention: FL = Tc + Td, FR = Tc - Td
        // NOTE: No clamping here; caps solver will clip if needed.
        //       Optional rate limit later.
        //       For now: pure intent.
        // (If you later add steering authority fade, do it here.)
        // ----
        // TdF_req:
    }
    float TdF_req = 0.f;
    {
        const float vFrontAbs = AxleSpeedAbs(sense, FL, FR);
        float uf = (TV.SteerTorqueHighSpeed > 1.f) ? (vFrontAbs / TV.SteerTorqueHighSpeed) : 1.f;
        uf = std::clamp(uf, 0.f, 1.f);
        const float kSteer = std::lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);
        TdF_req = s * TV.SteerTorqueFront * kSteer;
    }

    // rear yaw assist differential (only in D)
    float TdR_req = 0.f;
    if (currGear == Gear::D && absS > 0.001f)
    {
        const float vRearAbs = AxleSpeedAbs(sense, RL, RR);

        float us = 1.f;
        if (TV.RearFadeSpeed1 > TV.RearFadeSpeed0) {
            us = (vRearAbs - TV.RearFadeSpeed0) / (TV.RearFadeSpeed1 - TV.RearFadeSpeed0);
            us = std::clamp(us, 0.f, 1.f);
        }

        const float rearEffAbs = std::fabs(TcR_req);
        const float t0 = TV.RearFadeTorque0;
        const float t1 = TV.RearFadeTorque1;

        float uLong = 1.f;
        if (t1 > t0) {
            uLong = (rearEffAbs - t0) / (t1 - t0);
            uLong = std::clamp(uLong, 0.f, 1.f);
        }

        // TODO: Could be a params.TV configuration
        const float s0 = 0.05f;
        const float s1 = 0.25f;
        float uSteer = (absS - s0) / (s1 - s0);
        uSteer = std::clamp(uSteer, 0.f, 1.f);

        const float fadeRear = us * std::max(uLong, uSteer);

        const float oppMag = absS * TV.SteerTorqueRear;

        // SIGN: s>0 => TdR>0 => RR reduced / can go negative
        //       s<0 => TdR<0 => RL reduced / can go negative
        TdR_req = (s > 0.f) ? (+oppMag) : (-oppMag);
        TdR_req *= fadeRear;
    }

    // ========= Stage 2b: Rate limit steering/yaw differentials ONLY =========
    {
        const float maxD = TV.MaxTorquePerTick; // absolute torque per tick

        // Front steering diff always limited
        TdF_req = rateLimit(TdF_req, m_lastTdF, maxD);
        m_lastTdF = TdF_req;

        // Rear yaw assist diff limited too (even if it's 0 in R)
        TdR_req = rateLimit(TdR_req, m_lastTdR, maxD);
        m_lastTdR = TdR_req;
    }

    // ========= Stage 3: Solve axle pairs under caps (keep Td) =========
    const float TcF_req_m = sense.gearDir * TcF_req;
    const float TcR_req_m = sense.gearDir * TcR_req;

    // Td must NOT be flipped (otherwise steering mirrors in reverse)
    const float TdF_req_m = TdF_req;
    const float TdR_req_m = TdR_req;

    PairSolve front = SolvePairCaps(
        TcF_req_m, TdF_req_m,
        sense.capRev[FL], sense.capFwd[FL],
        sense.capRev[FR], sense.capFwd[FR]);

    PairSolve rear = SolvePairCaps(
        TcR_req_m, TdR_req_m,
        sense.capRev[RL], sense.capFwd[RL],
        sense.capRev[RR], sense.capFwd[RR]);


    // ========= Stage 4: Preserve trajectory as much as possible by reallocating Tc =========
    // Keep Td fixed; move missing longitudinal torque into any axle that still has Tc headroom.
    const float Tc_total_req_m = sense.gearDir * Tc_total_req;
    const float Tc_done_m = front.Tc + rear.Tc;
    float need = Tc_total_req_m - Tc_done_m;

    if (std::fabs(need) > 1e-3f)
    {
        auto headroom = [](float Tc, float TcMin, float TcMax, float need) -> float {
            if (need > 0.f) return (TcMax - Tc);
            if (need < 0.f) return (Tc - TcMin);
            return 0.f;
        };

        const float availF = headroom(front.Tc, front.TcMin, front.TcMax, need);
        const float availR = headroom(rear.Tc,  rear.TcMin,  rear.TcMax,  need);
        const float sum = availF + availR;

        if (sum > 1e-6f)
        {
            const float dF = need * (availF / sum);
            const float dR = need * (availR / sum);

            front.Tc = std::clamp(front.Tc + dF, front.TcMin, front.TcMax);
            rear.Tc  = std::clamp(rear.Tc  + dR, rear.TcMin,  rear.TcMax);

            front.L = front.Tc + front.Td;
            front.R = front.Tc - front.Td;
            rear.L  = rear.Tc  + rear.Td;
            rear.R  = rear.Tc  - rear.Td;
        }
        // else: nowhere to put it; accept deficit
    }

    // --- wheel torques in MOTOR coords ---
    float fl_m = front.L;
    float fr_m = front.R;
    float rl_m = rear.L;
    float rr_m = rear.R;

    // hard clamp (motor coords)
    const float hard = std::max(maxDrive, maxBrake);
    fl_m = std::clamp(fl_m, -hard, +hard);
    fr_m = std::clamp(fr_m, -hard, +hard);
    rl_m = std::clamp(rl_m, -hard, +hard);
    rr_m = std::clamp(rr_m, -hard, +hard);

    out.fl = (int16_t)fl_m;
    out.fr = (int16_t)fr_m;
    out.rl = (int16_t)rl_m;
    out.rr = (int16_t)rr_m;
    return out;
}