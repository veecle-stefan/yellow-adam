#include "torquevectoring.h"

auto clampF = [](float v, float lo, float hi) -> float
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
};


auto signOf = [](int16_t v) -> float
{ return (v > 0) ? +1.f : (v < 0) ? -1.f
                                  : 0.f; };

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
    return sorted[n / 2]; // n=3 -> x[1], n=4 -> x[2]
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
    int n = 0;
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
    const float recover = ctx.params->TV.SlipRecoverPerTick;
    const float minT = ctx.params->TV.SlipMinTorque;

    // Recovery every tick (toward current live maxDrive/maxBrake)
    for (int i = 0; i < 4; ++i)
    {
        m_capPos[i] += recover * maxDrive;
        if (m_capPos[i] > maxDrive)
            m_capPos[i] = maxDrive;
        if (m_capPos[i] < minT)
            m_capPos[i] = minT;

        m_capNeg[i] -= recover * maxBrake;
        if (m_capNeg[i] < -maxBrake)
            m_capNeg[i] = -maxBrake;
        if (m_capNeg[i] > -minT)
            m_capNeg[i] = -minT;
    }

    // Slip detection & envelope tightening
    for (int i = 0; i < 4; ++i)
    {
        const float wi = sd.wAbs[i];
        const float ci = cmd[i];

        bool slipDrive = false;
        bool slipBrake = false;

        if (vRef < vEps)
        {
            // Standstill special-case:
            // if we command meaningful torque but wheel moves -> it's slip/free-spin
            if (ci > +minT && wi > vEps)
                slipDrive = true;
            if (ci < -minT && wi > vEps)
                slipBrake = true; // “ABS at standstill” -> treat as loss of control
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
        sd.capPos[i] = clampF(m_capPos[i], minT, maxDrive);
        sd.capNeg[i] = clampF(m_capNeg[i], -maxBrake, -minT);
    }
    return sd;
}

TorqueVectoring::Torques TorqueVectoring::Compute(const TickContext &ctx, const Gear &currGear)
{
    Torques t{};

    if (currGear == Gear::N)
    {
        return t;
    }
    bool allowRearYawAssist = (currGear == Gear::D);
    const float throttleInput = static_cast<float>(ctx.user.throttle) * ((ctx.user.throttle >= 0) ? ctx.params->TV.maxPowerDrive : ctx.params->TV.maxPowerBrake) / 1000.0f; // [-maxBrakeT..+maxDriveT] approx

    float s = static_cast<float>(ctx.user.steering) / 1000.0f; // [-1..1]

    const SenseData sense = Sense(ctx, currGear);

   

    return t;
}