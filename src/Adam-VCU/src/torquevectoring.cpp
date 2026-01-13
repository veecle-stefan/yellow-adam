#include "torquevectoring.h"


TorqueVectoring::Torques TorqueVectoring::Compute(const TickContext &ctx, const Gear &currGear)
{
    Torques t{};

    if (currGear == Gear::N)
    {
        return t;
    }
    bool allowRearYawAssist = (currGear == Gear::D);

    const float maxT = (ctx.user.throttle > 0) ? static_cast<float>(ctx.params->TV.maxPowerDrive) : static_cast<float>(ctx.params->TV.maxPowerBrake);

    const float throttleInput =
        static_cast<float>(ctx.user.throttle) * maxT / 1000.0f; // [-maxT..+maxT]

    float s = static_cast<float>(ctx.user.steering) / 1000.0f; // [-1..1]
    if (s > 1.0f)
        s = 1.0f;
    if (s < -1.0f)
        s = -1.0f;

    const float absS = (s >= 0.f) ? s : -s;

    // D: forward => + command, R: forward => - command
    const float gearSign = (currGear == Gear::D) ? +1.0f : -1.0f;

    // --- CURRENT speed only (do not use last*) ---
    auto getSpeed = [](const std::optional<Axle::HistoryFrame> &fb, bool left) -> std::optional<int16_t>
    {
        if (!fb)
            return std::nullopt;

        // Replace with your real speed fields.
        return left ? std::optional<int16_t>(fb->sample.speedL_meas)
                    : std::optional<int16_t>(fb->sample.speedR_meas);
    };

    const auto sp_fl = getSpeed(ctx.currFront, true);
    const auto sp_fr = getSpeed(ctx.currFront, false);
    const auto sp_rl = getSpeed(ctx.currRear, true);
    const auto sp_rr = getSpeed(ctx.currRear, false);

    auto lerp = [](float a, float b, float u) -> float
    { return a + (b - a) * u; };

    auto signOf = [](int16_t v) -> float
    { return (v > 0) ? +1.f : (v < 0) ? -1.f
                                      : 0.f; };

    auto median_of = [&](float *arr, int n) -> float
    {
        // n is small (<=3). simple sort.
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (arr[j] < arr[i])
                {
                    float x = arr[i];
                    arr[i] = arr[j];
                    arr[j] = x;
                }

        if (n == 1)
            return arr[0];
        if (n == 2)
            return 0.5f * (arr[0] + arr[1]);
        return arr[1]; // fine for n==3 or n==4 here
    };

    // Precompute abs speeds
    float v[4];
    bool ok[4];
    v[0] = sp_fl ? static_cast<float>(abs(*sp_fl)) : 0.f;
    ok[0] = sp_fl.has_value();
    v[1] = sp_fr ? static_cast<float>(abs(*sp_fr)) : 0.f;
    ok[1] = sp_fr.has_value();
    v[2] = sp_rl ? static_cast<float>(abs(*sp_rl)) : 0.f;
    ok[2] = sp_rl.has_value();
    v[3] = sp_rr ? static_cast<float>(abs(*sp_rr)) : 0.f;
    ok[3] = sp_rr.has_value();

    auto median_wheelspeeds = [&](float *arr) -> float
    {
        float tmp[4];
        int n = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (!ok[i])
                continue;
            tmp[n++] = v[i];
        }
        if (n == 0)
            return 0.f;
        return median_of(tmp, n);
    };

    float vehicleSpeed = median_wheelspeeds(v);

    // limit to the max speed by fading out near max speed
    float allowedMaxSpeed = (currGear == Gear::D) ? ctx.params->TV.maxSpeedFwd : ctx.params->TV.maxSpeedRev;
    float start = allowedMaxSpeed - ctx.params->TV.SpeedLimiterFadeBand;

    // scale in [0..1]
    float spdFade = (vehicleSpeed - start) / ctx.params->TV.SpeedLimiterFadeBand;
    if (spdFade < 0.f)
        spdFade = 0.f;
    if (spdFade > 1.f)
        spdFade = 1.f;

    const float throttle = (throttleInput > 0.f && vehicleSpeed >= start)
                               ? (throttleInput * (1.f - spdFade))
                               : throttleInput;
    // Anti-reversing fade: scale braking torque to zero as |speed| -> 0
    const float vFadeDen = ctx.params->TV.AntiReversingSpeed;

    auto brakeScale = [&](std::optional<int16_t> sp) -> float
    {
        if (!sp || vFadeDen <= 1.0f)
            return 1.0f; // if speed missing: keep braking
        float a = static_cast<float>(abs(*sp)) / vFadeDen;
        if (a > 1.f)
            a = 1.f;
        if (a < 0.f)
            a = 0.f;
        return a;
    };

    // Base wheel command:
    // throttle > 0: acceleration in gear direction
    // throttle < 0: braking opposing actual wheel rotation (or expected gear direction if speed unknown)
    auto baseWheelCmd = [&](std::optional<int16_t> sp) -> float
    {
        if (throttle >= 0.0f)
        {
            return gearSign * throttle;
        }

        const float bmag = -throttle;                         // positive magnitude
        const float motionSign = sp ? signOf(*sp) : gearSign; // fallback: expected motion
        return (-motionSign) * bmag * brakeScale(sp);
    };

    // --- helpers ---
    auto clampF = [](float v, float lo, float hi) -> float
    {
        if (v < lo)
            return lo;
        if (v > hi)
            return hi;
        return v;
    };

    auto clampToI16 = [&](float v) -> int16_t
    {
        v = clampF(v, -maxT, +maxT);
        return static_cast<int16_t>(v);
    };

    struct PairOut
    {
        float l;
        float r;
    };
    auto allocate_pair = [&](float Tc, float Td) -> PairOut
    {
        // 1) differential cannot exceed max
        Td = clampF(Td, -maxT, +maxT);

        // 2) common must leave headroom for differential:
        // need |Tc+Td|<=maxT and |Tc-Td|<=maxT  => Tc in [-maxT+|Td|, +maxT-|Td|]
        const float head = (Td >= 0.f) ? Td : -Td;
        const float TcMax = +maxT - head;
        const float TcMin = -maxT + head;
        Tc = clampF(Tc, TcMin, TcMax);

        return {Tc + Td, Tc - Td};
    };

    auto rateLimit = [&](float desired, float &last, float maxDeltaPerTick) -> float
    {
        const float delta = desired - last;
        float limited = desired;
        if (delta > maxDeltaPerTick)
            limited = last + maxDeltaPerTick;
        if (delta < -maxDeltaPerTick)
            limited = last - maxDeltaPerTick;
        last = limited;
        return limited;
    };

    // =========================
    // (2) Front/Rear bias for longitudinal torque (load transfer compensation)
    // =========================
    const float Tc_front_raw = 0.5f * (baseWheelCmd(sp_fl) + baseWheelCmd(sp_fr));
    const float Tc_rear_raw = 0.5f * (baseWheelCmd(sp_rl) + baseWheelCmd(sp_rr));

    // Total longitudinal effort across both axles (sum, not average).
    // If frontShare==0.5 this reproduces the old behavior:
    // Tc_front ~= Tc_front_raw, Tc_rear ~= Tc_rear_raw.
    const float Tc_total = Tc_front_raw + Tc_rear_raw;

    const float TcAbs = (Tc_total >= 0.f) ? Tc_total : -Tc_total;
    float a = TcAbs / (ctx.params->TV.BiasHighThrottle * maxT);
    a = clampF(a, 0.f, 1.f);

    float frontShare = 0.5f;

    // IMPORTANT: decide drive vs brake based on Tc_total (actual requested longitudinal torque),
    // not on throttle input sign. This makes the behavior consistent in reverse gear as well.
    if (Tc_total >= 0.f)
    {
        frontShare = lerp(ctx.params->TV.DriveFrontShareLow, ctx.params->TV.DriveFrontShareHigh, a);
    }
    else
    {
        frontShare = lerp(ctx.params->TV.BrakeFrontShareLow, ctx.params->TV.BrakeFrontShareHigh, a);
    }

    frontShare = clampF(frontShare, 0.05f, 0.95f);

    float Tc_front = Tc_total * frontShare;
    float Tc_rear = Tc_total * (1.f - frontShare);

    // =========================
    // Rear axle: yaw assist (smoothly fades out near standstill / near zero throttle)
    // =========================
    float vRear = 0.f;
    if (sp_rl && sp_rr)
        vRear = 0.5f * (static_cast<float>(abs(*sp_rl)) + static_cast<float>(abs(*sp_rr)));
    else if (sp_rl)
        vRear = static_cast<float>(abs(*sp_rl));
    else if (sp_rr)
        vRear = static_cast<float>(abs(*sp_rr));
    else
        vRear = 0.f;

    const float v0 = ctx.params->TV.RearFadeSpeed0;
    const float v1 = ctx.params->TV.RearFadeSpeed1;
    float us = (v1 > v0) ? ((vRear - v0) / (v1 - v0)) : 1.f;
    us = clampF(us, 0.f, 1.f);

    // Longitudinal authority (traction/brake)
    const float rearEffAbs = (Tc_rear >= 0.f) ? Tc_rear : -Tc_rear;
    const float t0 = ctx.params->TV.RearFadeThrottle0 * maxT;
    const float t1 = ctx.params->TV.RearFadeThrottle1 * maxT;
    float uLong = (t1 > t0) ? ((rearEffAbs - t0) / (t1 - t0)) : 1.f;
    uLong = clampF(uLong, 0.f, 1.f);

    // Steering intent (coast yaw assist): enable smoothly with steering magnitude
    // (Pick a small deadband to avoid noise around zero steering.)
    // TODO: don't hardcode here, use DriveParams instead
    const float s0 = 0.05f; // steering where assist starts (~5%)
    const float s1 = 0.25f; // steering where assist is fully enabled (~25%)
    float uSteer = (absS - s0) / (s1 - s0);
    uSteer = clampF(uSteer, 0.f, 1.f);

    // Combine: allow yaw assist if either condition is strong
    const float ut = (uLong > uSteer) ? uLong : uSteer;

    const float fadeRear = us * ut;

    float Td_rear = 0.f;

    if (allowRearYawAssist && (absS > 0.001f))
    {
        const float oppMag = absS * static_cast<float>(ctx.params->TV.SteerTorqueRear);

        auto motionSign = [&](std::optional<int16_t> sp) -> float
        {
            return sp ? signOf(*sp) : gearSign;
        };

        // Convention: rl = Tc + Td, rr = Tc - Td
        if (s > 0.0f)
        {
            // turning right => right is inner => make RR oppose motion
            Td_rear = motionSign(sp_rr) * oppMag;
        }
        else
        {
            // turning left => left is inner => make RL oppose motion
            Td_rear = -motionSign(sp_rl) * oppMag;
        }

        Td_rear *= fadeRear;

        static float lastTdRear = 0.f;
        const float maxTdRearDeltaPerTick = ctx.params->TV.MaxTorquePerTick * maxT;
        Td_rear = rateLimit(Td_rear, lastTdRear, maxTdRearDeltaPerTick);
    }

    const auto rp = allocate_pair(Tc_rear, Td_rear);
    float rl = rp.l;
    float rr = rp.r;

    // =========================
    // Front axle: steering actuator -> preserve differential by reducing common-mode if needed
    // =========================
    float vFront = 0.f;
    if (sp_fl && sp_fr)
        vFront = 0.5f * (static_cast<float>(abs(*sp_fl)) + static_cast<float>(abs(*sp_fr)));
    else if (sp_fl)
        vFront = static_cast<float>(abs(*sp_fl));
    else if (sp_fr)
        vFront = static_cast<float>(abs(*sp_fr));
    else
        vFront = 0.f;

    float u = vFront / ctx.params->TV.SteerTorqueHighSpeed;
    u = clampF(u, 0.f, 1.f);
    const float k = lerp(ctx.params->TV.SteerTorqueLowFactor, ctx.params->TV.SteerTorqueHighFactor, u);

    float Td_front = s * static_cast<float>(ctx.params->TV.SteerTorqueFront) * k;

    static float lastTdFront = 0.f;
    const float maxTdDeltaPerTick = ctx.params->TV.MaxTorquePerTick * maxT;
    Td_front = rateLimit(Td_front, lastTdFront, maxTdDeltaPerTick);

    const auto fp = allocate_pair(Tc_front, Td_front);
    float fl = fp.l;
    float fr = fp.r;

    // =========================
    // (1) ABS/ASR: wheel-specific reference (exclude wheel under test)
    // =========================
    static float slipScale_fl = 1.f, slipScale_fr = 1.f, slipScale_rl = 1.f, slipScale_rr = 1.f;

    auto recover = [&](float &sc)
    {
        sc += ctx.params->TV.SlipRecoverPerTick;
        if (sc > 1.f)
            sc = 1.f;
    };

    // Default: recover scales every tick (unless we detect slip on that wheel)
    recover(slipScale_fl);
    recover(slipScale_fr);
    recover(slipScale_rl);
    recover(slipScale_rr);

    auto vRefExcluding = [&](int excludeIdx) -> float
    {
        float tmp[3];
        int n = 0;
        for (int i = 0; i < 4; ++i)
        {
            if (i == excludeIdx)
                continue;
            if (!ok[i])
                continue;
            tmp[n++] = v[i];
            if (n == 3)
                break;
        }
        if (n == 0)
            return 0.f;
        return median_of(tmp, n);
    };

    auto applySlipLogic = [&](int idx, float wheelTorqueCmd, float &sc)
    {
        const float vRefLocal = vRefExcluding(idx);
        if (vRefLocal < ctx.params->TV.SlipSpeedEps)
            return; // too slow / too noisy -> do nothing

        const float hi = vRefLocal * (1.f + ctx.params->TV.SlipRatio);
        const float lo = vRefLocal * (1.f - ctx.params->TV.SlipRatio);

        bool slip = false;
        const float te = ctx.params->TV.SlipTorqueEps * maxT;

        if (wheelTorqueCmd > te)
        {
            // ASR: wheel spins faster than the other wheels under positive torque
            if (v[idx] > hi)
                slip = true;
        }
        else if (wheelTorqueCmd < -te)
        {
            // ABS: wheel becomes much slower than the other wheels under negative torque
            if (v[idx] < lo)
                slip = true;
        }

        if (slip)
        {
            sc *= ctx.params->TV.SlipDownFactor;
            if (sc < ctx.params->TV.SlipMinScale)
                sc = ctx.params->TV.SlipMinScale;
        }
    };

    applySlipLogic(0, fl, slipScale_fl);
    applySlipLogic(1, fr, slipScale_fr);
    applySlipLogic(2, rl, slipScale_rl);
    applySlipLogic(3, rr, slipScale_rr);

    fl *= slipScale_fl;
    fr *= slipScale_fr;
    rl *= slipScale_rl;
    rr *= slipScale_rr;

    // =========================
    // Final clamp + pack
    // =========================
    t.fl = clampToI16(fl);
    t.fr = clampToI16(fr);
    t.rl = clampToI16(rl);
    t.rr = clampToI16(rr);

    return t;
}