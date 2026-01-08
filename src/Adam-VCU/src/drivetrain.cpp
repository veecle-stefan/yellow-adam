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
    extCmdQueue = xQueueCreate(10, sizeof(CommandItem));
    configASSERT(extCmdQueue);

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

void DriveTrain::SendCommand(const CommandItem* cmd)
{
    xQueueSend(extCmdQueue, cmd, portMAX_DELAY);
}

void DriveTrain::SendGear(Gear newGear)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::SetGear;
    cmd.p1.u8 = newGear;
    SendCommand(&cmd);
}

void DriveTrain::SendIndicators(bool left, bool right)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::SetIndicators;
    cmd.p1.onOff = left;
    cmd.p2.onOff = right;
    SendCommand(&cmd);
}

void DriveTrain::SendPowerLimit(uint16_t maxThrottle, uint16_t maxSpeedFwd, uint16_t maxSpeedRev)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::SetPowerLimit;
    cmd.p1.u16 = maxThrottle;
    cmd.p2.u16 = maxSpeedFwd;
    cmd.p3.u16 = maxSpeedRev;
    SendCommand(&cmd);
}

void DriveTrain::SendExternalControl(bool enable)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::EnableExternalControl;
    cmd.p1.onOff = enable;
    SendCommand(&cmd);
}

void DriveTrain::SendPowerOff()
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::PowerOff;
    SendCommand(&cmd);
}

void DriveTrain::SendHeadlight(uint8_t mode, bool on)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::SetHeadlight;
    cmd.p1.u8 = mode;
    cmd.p2.onOff = on;
    SendCommand(&cmd);
}

void DriveTrain::SendSteer(int16_t throttle, int16_t steer)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::Steer;
    cmd.p1.i16 = throttle;
    cmd.p2.i16 = steer;
    SendCommand(&cmd);
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
    static bool lastAuxSign = false;

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
        bool auxSign = u.aux > DriveConfig::DeadBand;
        if (auxSign != lastAuxSign) {
            u.auxPressed = true; // only this tick
        }
        lastAuxSign = auxSign;
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

uint8_t DriveTrain::ControllerSafety(const TickContext& ctx, const VehicleState& state)
{
    enum class Severity : uint8_t { Ok=0, Warn=1, Off=2 };

    auto sevMax = [](Severity a, Severity b) -> Severity { return (static_cast<uint8_t>(a) > static_cast<uint8_t>(b)) ? a : b; };

    auto evalOne = [&](const std::optional<Axle::HistoryFrame>& fb) -> Severity {
        if (!fb) return Severity::Ok; // treat missing feedback as "no data" (handled separately for beep)

        const auto& s = fb->sample;

        // Hard shutdown conditions
        if (s.batVoltage < DriveConfig::Controller::VoltageOff) return Severity::Off;
        if (s.boardTemp  >= DriveConfig::Controller::TempOff)   return Severity::Off;

        // Warning conditions
        if (s.batVoltage < DriveConfig::Controller::VoltageWarn) return Severity::Warn;
        if (s.boardTemp  >= DriveConfig::Controller::TempWarn)   return Severity::Warn;

        return Severity::Ok;
    };

    // 0) Highest priority: explicit poweroff request
    if (state.reqPowerOff) {
        return Axle::RemoteCommand::CmdPowerOff;
    }

    // 1) Idle-based shutdown (hard)
    const uint32_t timeUserIdle = ctx.nowMs - state.lastUserInput;
    if (timeUserIdle > DriveConfig::MaxUserIdleBeforeShutdown) {
        return Axle::RemoteCommand::CmdPowerOff;
    }

    // 2) Evaluate each controller independently (even if one is missing)
    Severity sev = Severity::Ok;
    sev = sevMax(sev, evalOne(ctx.currFront));
    sev = sevMax(sev, evalOne(ctx.currRear));

    if (sev == Severity::Off) {
        return Axle::RemoteCommand::CmdPowerOff;
    }

    // 3) Decide warning/beep behavior
    //    We separate "warning severity" from "how to beep" so nothing can override poweroff.
    uint8_t cmd = Axle::RemoteCommand::CmdNOP;

    // 3a) User-idle warning beep pattern (soft)
    if (timeUserIdle > DriveConfig::MaxUserIdleWarn) {
        const uint32_t t = timeUserIdle / 100;
        if (t % 10 == 0) {
            cmd = Axle::RemoteCommand::CmdBeep;
        } else if (t % 10 == 5) {
            cmd = Axle::RemoteCommand::CmdBeep + (uint8_t)10;
        }
    }

    // 3b) Controller warning beep (voltage/temp warn) — only if we haven't already chosen an idle pattern
    if (sev == Severity::Warn && cmd == Axle::RemoteCommand::CmdNOP) {
        cmd = Axle::RemoteCommand::CmdBeep;
    }

    // 3c) Missing feedback policy (optional):
    // If one or both controllers have no feedback, you might want a beep, but DO NOT treat as shutdown.
    const bool frontOk = ctx.currFront.has_value();
    const bool rearOk  = ctx.currRear.has_value();
    if ((!frontOk || !rearOk) && cmd == Axle::RemoteCommand::CmdNOP) {
        // Keep this mild; otherwise you’ll beep constantly on startup.
        cmd = Axle::RemoteCommand::CmdBeep;
    }

    return cmd;
}

void DriveTrain::ComputeLights(const TickContext& ctx, TickDecision& dec, VehicleState& state)
{
    enum class CenterPhase { RightOn, AllOff1, LeftOn, AllOff2 };
    static CenterPhase centerPhase = CenterPhase::RightOn;

    if (!ctx.user.detected) {
        dec.failSafe = true;
        return;
    }

    if (ctx.user.auxPressed) {

        if (ctx.user.steering > 0) {
            // Right toggle
            state.indicatorsL = false;
            state.indicatorsR = !state.indicatorsR;

        } else if (ctx.user.steering < 0) {
            // Left toggle
            state.indicatorsR = false;
            state.indicatorsL = !state.indicatorsL;

        } else {
            // Center: Right, Off, Left, Off
            switch (centerPhase) {
                case CenterPhase::RightOn:
                    state.indicatorsL = false;
                    state.indicatorsR = true;
                    centerPhase = CenterPhase::AllOff1;
                    break;

                case CenterPhase::AllOff1:
                    state.indicatorsL = false;
                    state.indicatorsR = false;
                    centerPhase = CenterPhase::LeftOn;
                    break;

                case CenterPhase::LeftOn:
                    state.indicatorsR = false;
                    state.indicatorsL = true;
                    centerPhase = CenterPhase::AllOff2;
                    break;

                case CenterPhase::AllOff2:
                    state.indicatorsL = false;
                    state.indicatorsR = false;
                    centerPhase = CenterPhase::RightOn;
                    break;
            }
        }
    }

    dec.failSafe   = false;
    state.brakeLight = (ctx.user.throttle <= -DriveConfig::Brakes::DetectThreshold);
    state.tailLight = (state.currGear != Gear::N);
    state.loBeam = (state.currGear == Gear::D);
    state.reverseLight = (state.currGear == Gear::R);
}

DriveTrain::Torques DriveTrain::TorqueVectoring(const TickContext& ctx, VehicleState& state)
{
    Torques t{};

    if (state.currGear == Gear::N) {
        return t;
    }

    const float maxT = static_cast<float>(state.maxPower);

    const float throttleInput =
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

    auto lerp = [](float a, float b, float u) -> float { return a + (b - a) * u; };

    auto signOf = [](int16_t v) -> float { return (v > 0) ? +1.f : (v < 0) ? -1.f : 0.f; };

    auto median_of = [&](float* arr, int n) -> float {
        // n is small (<=3). simple sort.
        for (int i = 0; i < n; ++i)
            for (int j = i + 1; j < n; ++j)
                if (arr[j] < arr[i]) { float x = arr[i]; arr[i] = arr[j]; arr[j] = x; }

        if (n == 1) return arr[0];
        if (n == 2) return 0.5f * (arr[0] + arr[1]);
        return arr[1]; // n==3
    };

    // Precompute abs speeds
    float v[4];
    bool  ok[4];
    v[0] = sp_fl ? static_cast<float>(abs(*sp_fl)) : 0.f; ok[0] = sp_fl.has_value();
    v[1] = sp_fr ? static_cast<float>(abs(*sp_fr)) : 0.f; ok[1] = sp_fr.has_value();
    v[2] = sp_rl ? static_cast<float>(abs(*sp_rl)) : 0.f; ok[2] = sp_rl.has_value();
    v[3] = sp_rr ? static_cast<float>(abs(*sp_rr)) : 0.f; ok[3] = sp_rr.has_value();

    auto median_wheelspeeds = [&](float* arr) -> float {
        float tmp[3];
        int n = 0;
        for (int i = 0; i < 4; ++i) {
            if (!ok[i]) continue;
            tmp[n++] = v[i];
        }
        if (n == 0) return 0.f;
        return median_of(tmp, n);
    };

    
    float vehicleSpeed = median_wheelspeeds(v);
    state.vehicleSpeed = (uint16_t)vehicleSpeed;
    
    // limit to the max speed by fading out near max speed
    float allowedMaxSpeed = (state.currGear == Gear::D) ? state.maxSpeedForward : state.maxSpeedReverse;
    float start = allowedMaxSpeed - DriveConfig::SpeedLimiterFadeBand;

    // scale in [0..1]
    float spdFade = (vehicleSpeed - start) / DriveConfig::SpeedLimiterFadeBand;
    if (spdFade < 0.f) spdFade = 0.f;
    if (spdFade > 1.f) spdFade = 1.f;

    const float throttle = (throttleInput > 0.f && vehicleSpeed >= start)
               ? (throttleInput * (1.f - spdFade))
               : throttleInput;
    // Anti-reversing fade: scale braking torque to zero as |speed| -> 0
    const float vFadeDen = static_cast<float>(DriveConfig::Brakes::AntiReversingSpeed);

    auto brakeScale = [&](std::optional<int16_t> sp) -> float {
        if (!sp || vFadeDen <= 1.0f) return 1.0f; // if speed missing: keep braking
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
    // (2) Front/Rear bias for longitudinal torque (load transfer compensation)
    // =========================
    const float Tc_front_raw = 0.5f * (baseWheelCmd(sp_fl) + baseWheelCmd(sp_fr));
    const float Tc_rear_raw  = 0.5f * (baseWheelCmd(sp_rl) + baseWheelCmd(sp_rr));

    // Total longitudinal effort across both axles (sum, not average).
    // If frontShare==0.5 this reproduces the old behavior:
    // Tc_front ~= Tc_front_raw, Tc_rear ~= Tc_rear_raw.
    const float Tc_total = Tc_front_raw + Tc_rear_raw;

    const float thrAbs = (throttle >= 0.f) ? throttle : -throttle;
    float a = thrAbs / (DriveConfig::TV::BiasHighThrottle * maxT);
    a = clampF(a, 0.f, 1.f);

    float frontShare = 0.5f;
    if (throttle >= 0.f) {
        frontShare = lerp(DriveConfig::TV::DriveFrontShareLow, DriveConfig::TV::DriveFrontShareHigh, a);
    } else {
        frontShare = lerp(DriveConfig::TV::BrakeFrontShareLow, DriveConfig::TV::BrakeFrontShareHigh, a);
    }

    // Safety clamp (optional but good): never starve an axle completely
    frontShare = clampF(frontShare, 0.05f, 0.95f);

    float Tc_front = Tc_total * frontShare;
    float Tc_rear  = Tc_total * (1.f - frontShare);

    // =========================
    // Rear axle: yaw assist (smoothly fades out near standstill / near zero throttle)
    // =========================
    float vRear = 0.f;
    if (sp_rl && sp_rr) vRear = 0.5f * (static_cast<float>(abs(*sp_rl)) + static_cast<float>(abs(*sp_rr)));
    else if (sp_rl)     vRear = static_cast<float>(abs(*sp_rl));
    else if (sp_rr)     vRear = static_cast<float>(abs(*sp_rr));
    else                vRear = 0.f;

    const float v0 = static_cast<float>(DriveConfig::TV::RearFadeSpeed0);
    const float v1 = static_cast<float>(DriveConfig::TV::RearFadeSpeed1);
    float us = (v1 > v0) ? ((vRear - v0) / (v1 - v0)) : 1.f;
    us = clampF(us, 0.f, 1.f);

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

        // Convention: rl = Tc + Td, rr = Tc - Td
        if (s > 0.0f) {
            // turning right => right is inner => make RR oppose motion
            Td_rear = motionSign(sp_rr) * oppMag;
        } else {
            // turning left => left is inner => make RL oppose motion
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
    // Front axle: steering actuator -> preserve differential by reducing common-mode if needed
    // =========================
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
    // (1) ABS/ASR: wheel-specific reference (exclude wheel under test)
    // =========================
    static float slipScale_fl = 1.f, slipScale_fr = 1.f, slipScale_rl = 1.f, slipScale_rr = 1.f;

    auto recover = [&](float& sc) {
        sc += DriveConfig::TV::SlipRecoverPerTick;
        if (sc > 1.f) sc = 1.f;
    };


    // Default: recover scales every tick (unless we detect slip on that wheel)
    recover(slipScale_fl);
    recover(slipScale_fr);
    recover(slipScale_rl);
    recover(slipScale_rr);


    auto vRefExcluding = [&](int excludeIdx) -> float {
        float tmp[3];
        int n = 0;
        for (int i = 0; i < 4; ++i) {
            if (i == excludeIdx) continue;
            if (!ok[i]) continue;
            tmp[n++] = v[i];
            if (n == 3) break;
        }
        if (n == 0) return 0.f;
        return median_of(tmp, n);
    };

    auto applySlipLogic = [&](int idx, float wheelTorqueCmd, float& sc) {
        const float vRefLocal = vRefExcluding(idx);
        if (vRefLocal < DriveConfig::TV::SlipSpeedEps) return; // too slow / too noisy -> do nothing

        const float hi = vRefLocal * (1.f + DriveConfig::TV::SlipRatio);
        const float lo = vRefLocal * (1.f - DriveConfig::TV::SlipRatio);

        bool slip = false;
        const float te = DriveConfig::TV::SlipTorqueEps * maxT;

        if (wheelTorqueCmd > te) {
            // ASR: wheel spins faster than the other wheels under positive torque
            if (v[idx] > hi) slip = true;
        } else if (wheelTorqueCmd < -te) {
            // ABS: wheel becomes much slower than the other wheels under negative torque
            if (v[idx] < lo) slip = true;
        }

        if (slip) {
            sc *= DriveConfig::TV::SlipDownFactor;
            if (sc < DriveConfig::TV::SlipMinScale) sc = DriveConfig::TV::SlipMinScale;
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

    lights.SetTailLight(state.tailLight);
    lights.SetBrakeLight(state.brakeLight);
    lights.SetHeadlight(state.hiBeam ? Lights::HeadLightState::High : state.loBeam ? Lights::HeadLightState::Dipped : Lights::HeadLightState::DRL);
    lights.SetReverseLight(state.reverseLight);
    lights.SetIndicator(Lights::IndicatorPosition::FL, state.indicatorsL);
    lights.SetIndicator(Lights::IndicatorPosition::RL, state.indicatorsL);
    lights.SetIndicator(Lights::IndicatorPosition::FR, state.indicatorsR);
    lights.SetIndicator(Lights::IndicatorPosition::RR, state.indicatorsR);
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

void DriveTrain::ProcessExtCmds(TickContext& ctx, VehicleState& state)
{
    // keep track of timing for external control
    static uint32_t lastExtSteerCmd = 0;

    if (state.externalControl)
    {
        // check for timeout
        if (ctx.nowMs - lastExtSteerCmd > DriveConfig::ExternalTimeout) {
            state.externalControl = false;
            state.lastExtSteering =  state.lastExtThrottle = 0;
        } 
    }
    // check for commands
    CommandItem extCmd;
    while (xQueueReceive(extCmdQueue, &extCmd, 0) == pdTRUE) {
        switch (extCmd.cmd) {
            case DriveCommand::SetGear:
                state.currGear = static_cast<Gear>(extCmd.p1.u8);
                break;
            case DriveCommand::SetIndicators:
                state.indicatorsL = extCmd.p1.onOff;
                state.indicatorsR = extCmd.p2.onOff;
                break;
            case DriveCommand::SetPowerLimit:
                state.maxPower = extCmd.p1.u16;
                state.maxSpeedForward = extCmd.p2.u16;
                state.maxSpeedReverse = extCmd.p3.u16;
                break;
            case DriveCommand::EnableExternalControl:
                state.externalControl = extCmd.p1.onOff;
                lastExtSteerCmd = ctx.nowMs;
                ctx.user.detected = true;
                break;
            case DriveCommand::SetHeadlight:
                {
                    uint8_t mode = extCmd.p1.u8; // 1=drl, 2=low, 3=high
                    bool on = extCmd.p2.onOff;
                    if (mode == 1) state.DRL = on;
                    else if (mode == 2) state.loBeam = on;
                    else if (mode == 3) state.hiBeam = on;
                }
                break;
            case DriveCommand::Steer:
                lastExtSteerCmd = ctx.nowMs;
                state.lastExtThrottle = extCmd.p1.i16;
                state.lastExtSteering = extCmd.p2.i16;
                break;
            case DriveCommand::PowerOff:
                state.reqPowerOff = true;
                break;
        }
    }

    if (state.externalControl) {
        ctx.user.throttle = state.lastExtThrottle;
        ctx.user.steering = state.lastExtSteering;
        // still alive
        ctx.user.detected = true;
        ctx.user.someInput = true;
    }
}

void DriveTrain::ControlTask()
{
    const TickType_t period       = pdMS_TO_TICKS(ControlPeriodMs);
    TickType_t       lastWakeTime = xTaskGetTickCount();
    VehicleState     state; // only thing carried between loop iterations

    for (;;) {
        const uint32_t nowMs = millis();

        TickContext  ctx = BuildContext(nowMs);
        ProcessExtCmds(ctx, state);
        TickDecision dec = ComputeDecision(ctx, state);
        ApplyDecision(dec, state);

        PublishStatus(ctx, dec, state);

        // roll “last” forward for next tick
        lastFrontFb = ctx.currFront; // also overwrite last if currFront is nullopt 
        lastRearFb  = ctx.currRear; // so it's never too old in terms of time

        vTaskDelayUntil(&lastWakeTime, period);
    }
}