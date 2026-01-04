#include "drivetrain.h"
#include "swconfig.h"

// ----- Constructor -----
DriveTrain::DriveTrain(Axle& axleF, Axle& axleR, Lights& lights)
    : ch1(HWConfig::Pins::PPM::Channel1, DriveConfig::DeadBand)
    , ch2(HWConfig::Pins::PPM::Channel2, DriveConfig::DeadBand)
    , ch3(HWConfig::Pins::PPM::Channel3, DriveConfig::DeadBand)
    , axleF(axleF)
    , axleR(axleR)
    , lights(lights)
{
    ch1.Begin();
    ch2.Begin();
    ch3.Begin();

    // create size-1 queue for status updates
    statusQueue = xQueueCreate(1, sizeof(DriveTrainStatus));
    configASSERT(statusQueue);

    // Start background control task
    xTaskCreatePinnedToCore(
        [](void* pvParameters) {
            static_cast<DriveTrain*>(pvParameters)->ControlTask();
            vTaskDelete(nullptr);
        },
        "DriveTrain",
        SWConfig::Tasks::MinStakSize,
        this,
        SWConfig::Tasks::PrioHigh,
        &processTask,
        SWConfig::CoreAffinity::CoreApp
    );
}

void DriveTrain::Shutdown()
{
    this->axleF.Shutdown();
    this->axleR.Shutdown();
    vTaskDelete(this->processTask);
    this->ch1.Stop();
    this->ch2.Stop();
    this->ch3.Stop();
}

bool DriveTrain::GetLatestStatus(DriveTrainStatus& out) const
{
  if (!statusQueue) return false;
  return xQueuePeek(statusQueue, &out, 0) == pdTRUE;
}


// Centralised RC reading + double-tap detection.
// Keeps state inside this function instead of spreading it over the control loop.
DriveTrain::UserCmd DriveTrain::ReadUserCmd(RCinput::UserInput ch1, RCinput::UserInput ch2, RCinput::UserInput ch3, uint32_t nowMs)
{
    // --- double-tap state machine ---
    static uint32_t firstTap = 0;
    static uint32_t lastTap  = 0;
    static bool     isTapping = false;
    static uint8_t  tapCount  = 0;

    UserCmd u{};

    if (!ch1 || !ch2 || !ch3) {
        // the value is not valid, user/signal not present or too old
        u.throttle = 0; // safe defaults
        u.steering = 0;
        u.aux = 0;
        u.detected = false;

        // reset tap detection
        firstTap = lastTap = 0;
        isTapping = false;
        tapCount = 0;

    } else {
        // signals valid -> plug them in
        u.throttle = *ch1;
        u.steering = *ch3;
        u.aux      = *ch2;
        u.detected = true;

        u.someInput = (abs(u.throttle) > DriveConfig::DeadBand) || (abs(u.steering) > DriveConfig::DeadBand);
    }

    
    const bool braking = (u.throttle < -DriveConfig::Brakes::DetectThreshold);

    if (braking) {
        if (!isTapping) {
            isTapping = true;
            lastTap = nowMs;
            if (tapCount == 0) firstTap = lastTap;
        }
    } else if (isTapping) {
        // released
        if ((nowMs - lastTap) < DriveConfig::Brakes::TapTime) {
            tapCount++;
            if ((nowMs - firstTap) < DriveConfig::Brakes::DoubleTapTime) {
                if (tapCount == 2) {
                    u.doubleTap = true;
                    tapCount = 0;
                }
            }
        } else {
            tapCount = 0;
        }
        isTapping = false;
    }

    return u;
}

DriveTrain::TickContext DriveTrain::BuildContext(uint32_t nowMs)
{
    TickContext ctx{};
    ctx.nowMs = nowMs;

    // carry previous for this tick
    ctx.lastFront = lastFrontFb;
    ctx.lastRear  = lastRearFb;

    // current snapshots (may be empty if no feedback arrived)
    ctx.currFront = axleF.GetLatestFeedback(ctx.nowMs);
    ctx.currRear  = axleR.GetLatestFeedback(ctx.nowMs);

    // user input
    ctx.user = ReadUserCmd(*ch1, *ch2, *ch3, nowMs);

    return ctx;
}

Axle::RemoteCommand DriveTrain::ControllerSafety(const TickContext& ctx, const VehicleState& state)
{
    Axle::RemoteCommand cmd = Axle::RemoteCommand::CmdNOP;

    // 1. Check for global timeout
    if (ctx.nowMs - state.lastUserInput > DriveConfig::MaxUserIdleBeforeShutdown) {
        cmd = Axle::RemoteCommand::CmdPowerOff;
    }

    // 2. Check for motor controller emergency
    if (ctx.currFront && ctx.currRear) {
        const auto& f = ctx.currFront->sample;
        const auto& r = ctx.currRear->sample;

        if ((f.batVoltage < DriveConfig::Controller::VoltageOff) ||
            (r.batVoltage < DriveConfig::Controller::VoltageOff)) {
            cmd = Axle::RemoteCommand::CmdPowerOff;
        } else if ((f.boardTemp >= DriveConfig::Controller::TempOff) ||
                   (r.boardTemp >= DriveConfig::Controller::TempOff)) {
            cmd = Axle::RemoteCommand::CmdPowerOff;
        } else if ((f.batVoltage < DriveConfig::Controller::VoltageWarn) ||
                   (r.batVoltage < DriveConfig::Controller::VoltageWarn) ||
                   (f.boardTemp >= DriveConfig::Controller::TempWarn) ||
                   (r.boardTemp >= DriveConfig::Controller::TempWarn)) {
            cmd = Axle::RemoteCommand::CmdBeep;
        }
    } else {
        // no feedback -> weird, at least beep!
        cmd = Axle::RemoteCommand::CmdBeep;
    }

    return cmd;
}

void DriveTrain::ComputeLights(const TickContext& ctx, TickDecision& dec, VehicleState& state)
{
    if (!ctx.user.detected) {
        dec.failSafe = true;
        return;
    }

    dec.failSafe   = false;
    dec.brakeLight = (ctx.user.throttle <= -DriveConfig::Brakes::DetectThreshold);
    dec.tailLight = (state.currGear != Gear::N);
    dec.loBeam = (state.currGear == Gear::D);
    dec.reverseLight = (state.currGear == Gear::R);
}

DriveTrain::Torques DriveTrain::TorqueVectoring(const TickContext& ctx, const VehicleState& state)
{
    Torques t{};

    if (state.currGear == Gear::N) {
        return t;
    }

    const float maxT = static_cast<float>(DriveConfig::TV::MaxOutputLimit);

    const float throttle =
        static_cast<float>(ctx.user.throttle) * maxT / 1000.0f; // [-maxT..+maxT]

    float s = static_cast<float>(ctx.user.steering) / 1000.0f;  // [-1..1]
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    const float absS = (s >= 0.f) ? s : -s;

    // D: forward => + command, R: forward => - command
    const float gearSign = (state.currGear == Gear::D) ? +1.0f : -1.0f;

    // --- CURRENT speed only (do not use last*) ---
    auto getSpeed = [](const std::optional<Axle::HistoryFrame>& fb, bool left) -> std::optional<int16_t> {
        if (!fb) return std::nullopt;

        // Replace with your real speed fields.
        return left ? std::optional<int16_t>(fb->sample.speedL_meas)
                    : std::optional<int16_t>(fb->sample.speedR_meas);
    };

    const auto sp_fl = getSpeed(ctx.currFront, true);
    const auto sp_fr = getSpeed(ctx.currFront, false);
    const auto sp_rl = getSpeed(ctx.currRear,  true);
    const auto sp_rr = getSpeed(ctx.currRear,  false);

    auto signOf = [](int16_t v) -> float { return (v > 0) ? +1.f : (v < 0) ? -1.f : 0.f; };

    // Anti-reversing fade: scale braking torque to zero as |speed| -> 0
    const float vFadeDen = static_cast<float>(DriveConfig::Brakes::AntiReversingSpeed);

    auto brakeScale = [&](std::optional<int16_t> sp) -> float {
        if (!sp || vFadeDen <= 1.0f) return 1.0f; // if speed missing: keep braking (your call)
        float a = static_cast<float>(abs(*sp)) / vFadeDen;
        if (a > 1.f) a = 1.f;
        if (a < 0.f) a = 0.f;
        return a;
    };

    // Base wheel command:
    // throttle > 0: acceleration in gear direction
    // throttle < 0: braking opposing actual wheel rotation (or expected gear direction if speed unknown)
    auto baseWheelCmd = [&](std::optional<int16_t> sp) -> float {
        if (throttle >= 0.0f) {
            return gearSign * throttle;
        }

        const float bmag = -throttle; // positive magnitude
        const float motionSign = sp ? signOf(*sp) : gearSign; // fallback: expected motion
        return (-motionSign) * bmag * brakeScale(sp);
    };

    // --- helpers ---
    auto clampF = [](float v, float lo, float hi) -> float {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    };

    auto clampToI16 = [&](float v) -> int16_t {
        v = clampF(v, -maxT, +maxT);
        return static_cast<int16_t>(v);
    };

    auto lerp = [](float a, float b, float u) -> float { return a + (b - a) * u; };

    struct PairOut { float l; float r; };
    auto allocate_pair = [&](float Tc, float Td) -> PairOut {
        // 1) differential cannot exceed max
        Td = clampF(Td, -maxT, +maxT);

        // 2) common must leave headroom for differential:
        // need |Tc+Td|<=maxT and |Tc-Td|<=maxT  => Tc in [-maxT+|Td|, +maxT-|Td|]
        const float head = (Td >= 0.f) ? Td : -Td;
        const float TcMax = +maxT - head;
        const float TcMin = -maxT + head;
        Tc = clampF(Tc, TcMin, TcMax);

        return { Tc + Td, Tc - Td };
    };

    auto rateLimit = [&](float desired, float& last, float maxDeltaPerTick) -> float {
        const float delta = desired - last;
        float limited = desired;
        if (delta >  maxDeltaPerTick) limited = last + maxDeltaPerTick;
        if (delta < -maxDeltaPerTick) limited = last - maxDeltaPerTick;
        last = limited;
        return limited;
    };

    // =========================
    // Rear axle: traction + yaw assist (smoothly fades out near standstill / near zero throttle)
    // =========================

    float vRear = 0.f;
    if (sp_rl && sp_rr) vRear = 0.5f * (static_cast<float>(abs(*sp_rl)) + static_cast<float>(abs(*sp_rr)));
    else if (sp_rl)     vRear = static_cast<float>(abs(*sp_rl));
    else if (sp_rr)     vRear = static_cast<float>(abs(*sp_rr));
    else                vRear = 0.f;

    const float Tc_rear = 0.5f * (baseWheelCmd(sp_rl) + baseWheelCmd(sp_rr));

    const float v0 = static_cast<float>(DriveConfig::TV::RearFadeSpeed0);
    const float v1 = static_cast<float>(DriveConfig::TV::RearFadeSpeed1);
    float us = (v1 > v0) ? ((vRear - v0) / (v1 - v0)) : 1.f;
    us = clampF(us, 0.f, 1.f);

    const float thrAbs = (throttle >= 0.f) ? throttle : -throttle;
    const float t0 = static_cast<float>(DriveConfig::TV::RearFadeThrottle0) * maxT;
    const float t1 = static_cast<float>(DriveConfig::TV::RearFadeThrottle1) * maxT;
    float ut = (t1 > t0) ? ((thrAbs - t0) / (t1 - t0)) : 1.f;
    ut = clampF(ut, 0.f, 1.f);

    const float fadeRear = us * ut;

    float Td_rear = 0.f;

    if (absS > 0.001f) {
        const float oppMag = absS * static_cast<float>(DriveConfig::TV::SteerTorqueRear);

        auto motionSign = [&](std::optional<int16_t> sp) -> float {
            return sp ? signOf(*sp) : gearSign;
        };

        if (s > 0.0f) {
            Td_rear = motionSign(sp_rr) * oppMag;
        } else {
            Td_rear = -motionSign(sp_rl) * oppMag;
        }

        Td_rear *= fadeRear;

        static float lastTdRear = 0.f;
        const float maxTdRearDeltaPerTick = DriveConfig::TV::MaxTorquePerTick * maxT;
        Td_rear = rateLimit(Td_rear, lastTdRear, maxTdRearDeltaPerTick);
    }

    const auto rp = allocate_pair(Tc_rear, Td_rear);
    float rl = rp.l;
    float rr = rp.r;

    // =========================
    // Front axle: steering is an actuator -> preserve differential by reducing common-mode if needed
    // =========================

    const float Tc_front = 0.5f * (baseWheelCmd(sp_fl) + baseWheelCmd(sp_fr));

    float vFront = 0.f;
    if (sp_fl && sp_fr) vFront = 0.5f * (static_cast<float>(abs(*sp_fl)) + static_cast<float>(abs(*sp_fr)));
    else if (sp_fl)     vFront = static_cast<float>(abs(*sp_fl));
    else if (sp_fr)     vFront = static_cast<float>(abs(*sp_fr));
    else                vFront = 0.f;

    float u = vFront / DriveConfig::TV::SteerTorqueHighSpeed;
    u = clampF(u, 0.f, 1.f);
    const float k = lerp(DriveConfig::TV::SteerTorqueLowFactor, DriveConfig::TV::SteerTorqueHighFactor, u);

    float Td_front = s * static_cast<float>(DriveConfig::TV::SteerTorqueFront) * k;

    static float lastTdFront = 0.f;
    const float maxTdDeltaPerTick = DriveConfig::TV::MaxTorquePerTick * maxT;
    Td_front = rateLimit(Td_front, lastTdFront, maxTdDeltaPerTick);

    const auto fp = allocate_pair(Tc_front, Td_front);
    float fl = fp.l;
    float fr = fp.r;

    // =========================
    // ABS/ASR: very simple per-wheel slip limiter (post-mixing, pre-clamp)
    // =========================
    // Persistent per-wheel scaling (0..1). Static is OK since you only have one drivetrain.
    static float slipScale_fl = 1.f, slipScale_fr = 1.f, slipScale_rl = 1.f, slipScale_rr = 1.f;

    auto recover = [&](float& sc) {
        sc += DriveConfig::TV::SlipRecoverPerTick;
        if (sc > 1.f) sc = 1.f;
    };

    // Build a reference speed from available wheels (median-ish via min/max removal for 4 values)
    // Use ABS speed to avoid sign issues.
    float v[4];
    bool  ok[4];
    v[0] = sp_fl ? static_cast<float>(abs(*sp_fl)) : 0.f; ok[0] = sp_fl.has_value();
    v[1] = sp_fr ? static_cast<float>(abs(*sp_fr)) : 0.f; ok[1] = sp_fr.has_value();
    v[2] = sp_rl ? static_cast<float>(abs(*sp_rl)) : 0.f; ok[2] = sp_rl.has_value();
    v[3] = sp_rr ? static_cast<float>(abs(*sp_rr)) : 0.f; ok[3] = sp_rr.has_value();

    // Compute reference as average of middle values among valid speeds (robust-ish).
    // If <2 valid speeds, skip slip detection.
    int n = 0;
    float tmp[4];
    for (int i = 0; i < 4; ++i) if (ok[i]) tmp[n++] = v[i];

    float vRef = 0.f;
    if (n >= 2) {
        // simple sort (n <= 4)
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (tmp[j] < tmp[i]) { float x = tmp[i]; tmp[i] = tmp[j]; tmp[j] = x; }

        if (n == 2) vRef = 0.5f * (tmp[0] + tmp[1]);
        else if (n == 3) vRef = tmp[1];
        else /*n==4*/ vRef = 0.5f * (tmp[1] + tmp[2]);
    }

    // Default: recover scales every tick (unless we detect slip on that wheel)
    recover(slipScale_fl);
    recover(slipScale_fr);
    recover(slipScale_rl);
    recover(slipScale_rr);

    auto applySlipLogic = [&](float wheelAbsSpeed, float vRefLocal, float wheelTorqueCmd, float& sc) {
        if (vRefLocal < DriveConfig::TV::SlipSpeedEps) return; // too slow / too noisy -> do nothing

        const float hi = vRefLocal * (1.f + DriveConfig::TV::SlipRatio);
        const float lo = vRefLocal * (1.f - DriveConfig::TV::SlipRatio);

        bool slip = false;

        if (wheelTorqueCmd > 0.f) {
            // ASR: driven wheel spins faster than others under positive torque
            if (wheelAbsSpeed > hi) slip = true;
        } else if (wheelTorqueCmd < 0.f) {
            // ABS: wheel locks (much slower than others) under negative torque
            if (wheelAbsSpeed < lo) slip = true;
        }

        if (slip) {
            sc *= DriveConfig::TV::SlipDownFactor;
            if (sc < DriveConfig::TV::SlipMinScale) sc = DriveConfig::TV::SlipMinScale;
        }
    };

    // Evaluate slip based on *current request* (best you can do without last torque)
    applySlipLogic(v[0], vRef, fl, slipScale_fl);
    applySlipLogic(v[1], vRef, fr, slipScale_fr);
    applySlipLogic(v[2], vRef, rl, slipScale_rl);
    applySlipLogic(v[3], vRef, rr, slipScale_rr);

    // Apply scaling (reduces both accel and brake magnitudes)
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

void DriveTrain::CheckGear(const TickContext& ctx, VehicleState& state)
{
    if (!ctx.user.detected) {
        // reset to neutral
        state.currGear = Gear::N;
        return;
    }
    // check for double-tap on the brake while not moving
    if (ctx.user.doubleTap && ctx.currFront && (ctx.currFront->sample.speedL_meas == 0) && (ctx.currFront->sample.speedR_meas == 0)) {
        if (state.currGear == Gear::D) {
            state.currGear = Gear::R;
        } else {
            // from neutral or reverse -> goto Drive
            state.currGear = Gear::D;
        }
    }
}

DriveTrain::TickDecision DriveTrain::ComputeDecision(const TickContext& ctx, VehicleState& state)
{
    TickDecision dec{};

    // 1) Update last user input timestamp
    if (ctx.user.someInput) {
        state.lastUserInput = ctx.nowMs;
    }

    // 2) gear
    CheckGear(ctx, state);

    // 3) torques
    dec.torques = TorqueVectoring(ctx, state);

    // 4) safety command
    dec.cmd = ControllerSafety(ctx, state);

    // 5) lights intent
    ComputeLights(ctx, dec, state);

    return dec;
}

void DriveTrain::ApplyDecision(const TickDecision& dec, VehicleState& state)
{
    // Motors
    axleF.Send(dec.torques.fl, dec.torques.fr, dec.cmd);
    axleR.Send(dec.torques.rl, dec.torques.rr, dec.cmd);


    // Lights
    if (dec.failSafe) {
        if (!state.hazards) {
            lights.SetHeadlight(Lights::HeadLightState::Off);
            lights.SetReverseLight(false);
            lights.SetBrakeLight(false);
            lights.SetTailLight(false);
            lights.SetIndicators(true, true, true, true);
            state.hazards = true;
        }
        return;
    }

    if (state.hazards) {
        lights.SetHeadlight(Lights::HeadLightState::DRL);
        lights.SetIndicators(false, false, false, false);
        state.hazards = false;
    }

    lights.SetTailLight(dec.tailLight);
    lights.SetBrakeLight(dec.brakeLight);
    lights.SetHeadlight(dec.hiBeam ? Lights::HeadLightState::High : dec.loBeam ? Lights::HeadLightState::Dipped : Lights::HeadLightState::DRL);
    lights.SetReverseLight(dec.reverseLight);
}

void DriveTrain::PublishStatus(const TickContext& ctx, const TickDecision& dec, const VehicleState& state)
{
  DriveTrainStatus st{};
  st.ts_ms = ctx.nowMs;

  st.throttle = ctx.user.throttle;
  st.steering = ctx.user.steering;
  st.aux      = ctx.user.aux;
  st.userDetected = ctx.user.detected;

  st.tq_fl = dec.torques.fl; st.tq_fr = dec.torques.fr;
  st.tq_rl = dec.torques.rl; st.tq_rr = dec.torques.rr;

  st.state = state;

  // Pull from last known feedback (what you already store)
  if (lastFrontFb) {
    st.haveFront = true;
    st.curr_fl = lastFrontFb->sample.currL_meas;
    st.curr_fr = lastFrontFb->sample.currR_meas;
    st.vel_fl  = lastFrontFb->sample.speedL_meas;
    st.vel_fr  = lastFrontFb->sample.speedR_meas;
    st.voltage_front = lastFrontFb->sample.batVoltage;
    st.temp_front    = lastFrontFb->sample.boardTemp;
  }
  if (lastRearFb) {
    st.haveRear = true;
    st.curr_rl = lastRearFb->sample.currL_meas;
    st.curr_rr = lastRearFb->sample.currR_meas;
    st.vel_rl  = lastRearFb->sample.speedL_meas;
    st.vel_rr  = lastRearFb->sample.speedR_meas;
    st.voltage_rear = lastRearFb->sample.batVoltage;
    st.temp_rear    = lastRearFb->sample.boardTemp;
  }

  xQueueOverwrite(statusQueue, &st);
}

void DriveTrain::ControlTask()
{
    const TickType_t period       = pdMS_TO_TICKS(ControlPeriodMs);
    TickType_t       lastWakeTime = xTaskGetTickCount();
    VehicleState     state; // only thing carried between loop iterations

    for (;;) {
        const uint32_t nowMs = millis();

        TickContext  ctx = BuildContext(nowMs);
        TickDecision dec = ComputeDecision(ctx, state);
        ApplyDecision(dec, state);

        PublishStatus(ctx, dec, state);

        // roll “last” forward for next tick
        lastFrontFb = ctx.currFront; // also overwrite last if currFront is nullopt 
        lastRearFb  = ctx.currRear; // so it's never too old in terms of time

        vTaskDelayUntil(&lastWakeTime, period);
    }
}