#include "torquevectoring.h"


auto sortSmallN = [](float *x, int n)
{
    for (int i = 0; i < n; ++i)
        for (int j = i + 1; j < n; ++j)
            if (x[j] < x[i])
            {
                float t = x[i];
                x[i] = x[j];
                x[j] = t;
            }
};

auto median = [](float *sorted, int n) -> float
{
    if (n <= 0)
        return 0.f;
    if (n == 1)
        return sorted[0];
    if (n == 2)
        return 0.5f * (sorted[0] + sorted[1]);
    if (n == 3)
        return sorted[1];
    return 0.5f * (sorted[1] + sorted[2]); // n=4
};

// ============================================================================
// TorqueVectoring::Sense()
// - Computes wheel speeds (abs + signed)
// - Computes a robust vehicleSpeedAbs reference
//   * accel: use 2nd smallest abs wheel speed  (reject fast spin outliers)
//   * brake: use 2nd largest  abs wheel speed  (reject locked wheel outliers)
//   * coast: use median abs wheel speed
// - Persists m_vehicleSpeedAbs and rejects physically impossible jumps
//   (hard reject, NOT "slow converge" into nonsense)
// ============================================================================
TorqueVectoring::SenseData TorqueVectoring::Sense(const TickContext &ctx, Gear currGear)
{
    SenseData sd{};

    // ----- read wheel speeds (signed) -----
    auto getSpeed = [](const std::optional<Axle::HistoryFrame> &fb, bool left) -> float
    {
        // caller guarantees fb has value if passed in, but keep it safe
        if (!fb)
            return 0.f;
        return left ? static_cast<float>(fb->sample.speedL_meas) : static_cast<float>(fb->sample.speedR_meas);
    };

    sd.ok[0] = sd.ok[1] = ctx.currFront.has_value();
    sd.ok[2] = sd.ok[3] = ctx.currRear.has_value();

    sd.w[0] = getSpeed(ctx.currFront, true);
    sd.w[1] = getSpeed(ctx.currFront, false);
    sd.w[2] = getSpeed(ctx.currRear, true);
    sd.w[3] = getSpeed(ctx.currRear, false);
    
    for (int i = 0; i < 4; i++) {
        sd.wAbs[i] = fabsf(sd.w[i]);
    }

    // Collect valid abs speeds for robust statistics
    float validWheels[4];
    float sorted[4];
    uint8_t n = 0;
    for (int i = 0; i < 4; ++i) {
        if (sd.ok[i]) {
            validWheels[n] = sorted[n] = sd.wAbs[i];
            n++;
        }
    }
    sortSmallN(sorted, n);

    // Decide motion intent from requested throttleInput sign
    const bool accel = (ctx.user.throttle > 0);
    const bool brake = (ctx.user.throttle < 0);
    const bool coast = (!accel && !brake);

    // Robust measured reference speed v_ref_meas (abs)
    float v_ref_meas = 0.f;
    if (n == 0)
    {
        v_ref_meas = 0.f;
    }
    else
    {

        if (accel)
        {
            // Use 2nd smallest to ignore a single (or even two) fast spinners.
            // For n==1 fallback to smallest.
            v_ref_meas = (n >= 2) ? sorted[1] : sorted[0];
        }
        else if (brake)
        {
            // Use 2nd largest to ignore a single locked wheel.
            v_ref_meas = (n >= 2) ? sorted[n - 2] : sorted[n - 1];
        }
        else
        {
            // Coasting: median is a good compromise
            v_ref_meas = median(sorted, n);
        }
    }

    // --- Standstill handling ---
    // If we're basically standing still, we still want a stable 0 reference and
    // do not want a single free-spinning wheel to "create" vehicle speed.
    const float eps = ctx.params->TV.SlipSpeedEps;

    if (m_vehicleSpeedAbs < eps)
    {
        if (accel)
        {
            // At standstill under acceleration: trust the slow end hard.
            // If any wheel is truly at ~0, vehicle speed should remain ~0.
            // Use smallest abs speed.
            if (n > 0)
            {
                v_ref_meas = sorted[0];
            }
        }
        else if (brake)
        {
            // At standstill under braking: vehicle speed should be 0 (avoid reverse logic contamination).
            v_ref_meas = 0.f;
        }
        else
        {
            // Coast at standstill: keep 0
            v_ref_meas = 0.f;
        }
    }

    const float dv = v_ref_meas - m_vehicleSpeedAbs;

    const bool plausible =
        (dv <= ctx.params->TV.maxRealisticAccel) &&
        (dv >= -ctx.params->TV.maxRealisticDecel);

    if (plausible)
    {
        // Accept measurement (no smoothing; we already did robust selection)
        m_vehicleSpeedAbs = v_ref_meas;
    }
    else
    {
        // Reject outlier: keep previous vehicle speed estimate
        // do NOT converge toward a known-bad value
    }

    // Output for this tick
    sd.vehicleSpeedAbs = m_vehicleSpeedAbs;

    // now ABS/ASR envelope based on last wheel speeds
    // =========================
    // ASR/ABS torque envelopes per wheel (persistent, updated each tick)
    // =========================
    const float maxDrive = static_cast<float>(ctx.params->TV.maxPowerDrive);
    const float maxBrake = static_cast<float>(ctx.params->TV.maxPowerBrake); // magnitude

    if (!m_capsInit)
    {
        for (int i = 0; i < 4; ++i)
        {
            m_capPos[i] = maxDrive;
            m_capNeg[i] = -maxBrake;
        }
        m_capsInit = true;
    }

    // Pull last issued torque commands from feedback (same sample as speed)
    auto getLastTorque = [](const std::optional<Axle::HistoryFrame> &fb, bool left) -> float
    {
        if (!fb)
            return 0.f;
        return left ? static_cast<float>(fb->sample.cmdL)
                    : static_cast<float>(fb->sample.cmdR);
    };

    const float cmd[4] = {
        getLastTorque(ctx.currFront, true),
        getLastTorque(ctx.currFront, false),
        getLastTorque(ctx.currRear, true),
        getLastTorque(ctx.currRear, false),
    };

    const float vRef = m_vehicleSpeedAbs;
    const float vEps = ctx.params->TV.SlipSpeedEps;
    const float slipRatio = ctx.params->TV.SlipRatio;
    const float down = ctx.params->TV.SlipDownFactor;
    const float recover = ctx.params->TV.SlipRecoverTorquePerTick;
    const float minT = ctx.params->TV.SlipMinTorque;

    // Recovery every tick (toward current live maxDrive/maxBrake)
    for (int i = 0; i < 4; ++i)
    {
        m_capPos[i] += recover;
        if (m_capPos[i] > maxDrive)
            m_capPos[i] = maxDrive;
        if (m_capPos[i] < minT)
            m_capPos[i] = minT;

        m_capNeg[i] -= recover ;
        if (m_capNeg[i] < -maxBrake)
            m_capNeg[i] = -maxBrake;
        if (m_capNeg[i] > -minT)
            m_capNeg[i] = -minT;
    }

    // Anchor wheel: at least one wheel is basically not moving.
    // This is the condition under which "standstill slip" is meaningful.
    const bool hasAnchor = (n > 0) && (sorted[0] < vEps);

    // Slip detection & envelope tightening
    for (int i = 0; i < 4; ++i)
    {
        const float wi = sd.wAbs[i];
        const float ci = cmd[i];

        bool slipDrive = false;
        bool slipBrake = false;

        if (vRef < vEps)
        {
            // Standstill slip detection only makes sense if we have an anchor wheel.
            // Otherwise (all wheels already moving a bit), don't "invent" slip decisions here.
            if (hasAnchor)
            {
                // ASR at standstill: if we command meaningful drive torque and this wheel moves -> free-spin
                if (ci > +minT && wi > vEps)
                    slipDrive = true;

                // ABS at standstill is not meaningful for your use-case (vehicleSpeed~0).
                // Prevent tightening brake envelopes just because a wheel is still spinning down.
                // If you want a "wheel spin-down brake limiter" later, do it explicitly elsewhere.
                slipBrake = false;
            }
            else
            {
                // No anchor => ambiguous whether we're truly at standstill.
                // Skip standstill tightening and let the moving-state ratio logic handle it once vRef rises.
                slipDrive = false;
                slipBrake = false;
            }
        }
        else
        {
            const float hi = vRef * (1.f + slipRatio);
            const float lo = vRef * (1.f - slipRatio);

            // ASR: wheel too fast under positive commanded torque
            if (ci > +minT && wi > hi)
                slipDrive = true;

            // ABS: wheel too slow under negative commanded torque
            if (ci < -minT && wi < lo)
                slipBrake = true;
        }

        if (slipDrive)
        {
            m_capPos[i] *= down;
            if (m_capPos[i] < minT)
                m_capPos[i] = minT;
        }
        if (slipBrake)
        {
            m_capNeg[i] *= down; // negative -> closer to 0 => less braking
            if (m_capNeg[i] > -minT)
                m_capNeg[i] = -minT;
        }
    }

    // Output for this tick (clamped to CURRENT live limits)
    for (int i = 0; i < 4; ++i)
    {
        sd.capPos[i] = std::clamp(m_capPos[i], minT, maxDrive);
        sd.capNeg[i] = std::clamp(m_capNeg[i], -maxBrake, -minT);
    }

    // =========================================================================
    // Speed limiter as an envelope modifier (NOT throttle scaling)
    //
    // Goal:
    //  - Prevent overspeed without oscillating (no "hit limit -> drop torque -> recover -> repeat")
    //  - Do NOT kill cornering at the reverse speed cap:
    //      * Use robust vehicle speed as the primary reference
    //      * Only apply per-wheel speed capping if a wheel is an outlier (spinner)
    //
    // Effect:
    //  - Tightens sd.capPos[i] (drive torque) when near/above speed limit.
    //  - Does NOT touch braking envelopes (capNeg), because braking is how you slow down.
    // =========================================================================

    const float band = ctx.params->TV.SpeedLimiterFadeBand;
    if (band > 1e-3f)
    {
        const float vRef = sd.vehicleSpeedAbs;

        const float vMax   = (currGear == Gear::D) ? ctx.params->TV.maxSpeedFwd : ctx.params->TV.maxSpeedRev;
        const float vStart = vMax - band;

        // Outlier threshold uses existing ASR parameter (no new params)
        const float hiOutlier = vRef * (1.f + ctx.params->TV.SlipRatio);
        const float vEps = ctx.params->TV.SlipSpeedEps;

        auto capFromSpeed = [&](float vForCap) -> float
        {
            if (vForCap <= vStart) return maxDrive;
            if (vForCap >= vMax)   return 0.f;

            const float u = (vForCap - vStart) / band; // 0..1
            return maxDrive * (1.f - std::clamp(u, 0.f, 1.f));
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

            // Tighten positive envelope only (drive torque)
            const float capTarget = capFromSpeed(vForCap);

            if (sd.capPos[i] > capTarget)
                sd.capPos[i] = capTarget;

            // Keep invariant: capPos should never go below SlipMinTorque
            // (otherwise you could "brick" drive authority at the speed cap)
            if (sd.capPos[i] < minT)
                sd.capPos[i] = minT;
        }
    }


    return sd;
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

int TorqueVectoring::BestMotionWheel(const SenseData& s, float vRefAbs)
{
    int best = -1;
    float bestErr = 1e9f;
    for (int i = 0; i < 4; ++i) {
        if (!s.ok[i]) continue;
        const float err = std::fabs(s.wAbs[i] - vRefAbs);
        if (err < bestErr) { bestErr = err; best = i; }
    }
    return best;
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
    const float vRefAbs = sense.vehicleSpeedAbs;

    // ========= Stage 1: Normalize user inputs =========
    const float s = static_cast<float>(ctx.user.steering) / 1000.0f;   // convention: -left, +right
    const float absS = std::fabs(s);

    const float maxDrive = static_cast<float>(TV.maxPowerDrive);
    const float maxBrake = static_cast<float>(TV.maxPowerBrake);

    // requested magnitude in "torque units"
    const float maxT = (ctx.user.throttle >= 0) ? maxDrive : maxBrake;
    const float throttleInput = static_cast<float>(ctx.user.throttle) * maxT / 1000.0f;

    const float gearSign = (currGear == Gear::D) ? +1.0f : -1.0f;

    // ========= Stage 2: Trajectory intent (Tc_total + TdF + TdR) =========
    // Longitudinal intent:
    float Tc_total_req = 0.f;

    if (throttleInput >= 0.f) {
        // drive in gear direction
        Tc_total_req = gearSign * throttleInput;
    } else {
        // braking opposes actual motion direction (use best wheel)
        const float bmag = -throttleInput;

        int best = BestMotionWheel(sense, vRefAbs);
        float motionSign = gearSign;
        if (best >= 0) {
            motionSign = (sense.w[best] > 0.f) ? +1.f : (sense.w[best] < 0.f) ? -1.f : gearSign;
        }

        // anti-reversing fade near standstill
        float brakeScale = 1.f;
        if (TV.AntiReversingSpeed > 1.f)
            brakeScale = std::clamp(vRefAbs / TV.AntiReversingSpeed, 0.f, 1.f);

        Tc_total_req = (-motionSign) * bmag * brakeScale;
    }

    // front/rear bias based on |Tc_total|
    const float TcAbs = std::fabs(Tc_total_req);
    const float normMax = (Tc_total_req >= 0.f) ? maxDrive : maxBrake;

    float uBias = (TV.BiasHighThrottle * normMax > 1e-6f) ? (TcAbs / (TV.BiasHighThrottle * normMax)) : 1.f;
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

        const float kSteer = std::lerp(TV.SteerTorqueLowFactor, TV.SteerTorqueHighFactor, uf);

        // SIGN: s<0 => Td<0 => FR gets more torque  ✅
        //       s>0 => Td>0 => FL gets more torque  ✅
        // Pair convention: FL = Tc + Td, FR = Tc - Td
        // (matches your old code)
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
        const float t0 = TV.RearFadeThrottle0 * normMax;
        const float t1 = TV.RearFadeThrottle1 * normMax;

        float uLong = 1.f;
        if (t1 > t0) {
            uLong = (rearEffAbs - t0) / (t1 - t0);
            uLong = std::clamp(uLong, 0.f, 1.f);
        }

        // no new params: keep your old steering fade constants
        const float s0 = 0.05f;
        const float s1 = 0.25f;
        float uSteer = (absS - s0) / (s1 - s0);
        uSteer = std::clamp(uSteer, 0.f, 1.f);

        const float fadeRear = us * std::max(uLong, uSteer);

        const float oppMag = absS * static_cast<float>(TV.SteerTorqueRear);

        // SIGN: s>0 => TdR>0 => RR reduced / can go negative ✅
        //       s<0 => TdR<0 => RL reduced / can go negative ✅
        TdR_req = (s > 0.f) ? (+oppMag) : (-oppMag);
        TdR_req *= fadeRear;
    }

    // ========= Stage 3: Solve axle pairs under caps (keep Td) =========
    PairSolve front = SolvePairCaps(
        TcF_req, TdF_req,
        sense.capNeg[FL], sense.capPos[FL],
        sense.capNeg[FR], sense.capPos[FR]
    );

    PairSolve rear  = SolvePairCaps(
        TcR_req, TdR_req,
        sense.capNeg[RL], sense.capPos[RL],
        sense.capNeg[RR], sense.capPos[RR]
    );

    // ========= Stage 4: Preserve trajectory as much as possible by reallocating Tc =========
    // Keep Td fixed; move missing longitudinal torque into any axle that still has Tc headroom.
    const float Tc_done = front.Tc + rear.Tc;
    float need = Tc_total_req - Tc_done;

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

    float wheel[4] = { front.L, front.R, rear.L, rear.R };

    // ========= Stage 5: Rate limit final wheel commands 
    {

        for (int i = 0; i < 4; ++i)
        {
            const float d = wheel[i] - m_lastWheelCmd[i];
            if (d >  TV.MaxTorquePerTick) wheel[i] = m_lastWheelCmd[i] + TV.MaxTorquePerTick;
            if (d < -TV.MaxTorquePerTick) wheel[i] = m_lastWheelCmd[i] - TV.MaxTorquePerTick;
            m_lastWheelCmd[i] = wheel[i];
        }
    }

    // ========= Stage 6: Final clamp to caps + physical limits =========
    for (int i = 0; i < 4; ++i)
    {
        wheel[i] = std::clamp(wheel[i], sense.capNeg[i], sense.capPos[i]);
        wheel[i] = std::clamp(wheel[i], -maxBrake, +maxDrive);
    }

    out.fl = static_cast<int16_t>(wheel[FL]);
    out.fr = static_cast<int16_t>(wheel[FR]);
    out.rl = static_cast<int16_t>(wheel[RL]);
    out.rr = static_cast<int16_t>(wheel[RR]);

    return out;
}