#include "drivetrain.h"
#include "driveparams.h"
#include "swconfig.h"

// ----- Constructor -----
DriveTrain::DriveTrain(Axle& axleF, Axle& axleR, Lights& lights)
    : ch1(HWConfig::Pins::PPM::Channel1, SWConfig::InputFiltering::DeadBand)
    , ch2(HWConfig::Pins::PPM::Channel2, SWConfig::InputFiltering::DeadBand)
    , ch3(HWConfig::Pins::PPM::Channel3, SWConfig::InputFiltering::DeadBand)
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

void DriveTrain::SendTuneTV(uint16_t id, float value)
{
    CommandItem cmd;
    cmd.cmd = DriveCommand::TuneTVParam;
    cmd.p1.u16 = id;
    cmd.p2.f16 = value;
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

QueueHandle_t DriveTrain::GetStatusQueue() const
{
  return this->statusQueue;
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
        u.throttle = -100; // safe defaults: light braking
        u.steering = 0; // safe defaults: steer straight
        u.aux = 0;
        u.detected = false;

        // reset tap detection
        firstTap = lastTap = 0;
        isTapping = false;
        tapCount = 0;

        return u;

    } else {
        // signals valid -> plug them in
        u.throttle = *ch1;
        u.steering = *ch3;
        u.aux      = *ch2;
        u.detected = true;

        u.someInput = (abs(u.throttle) > SWConfig::InputFiltering::DeadBand) || (abs(u.steering) > SWConfig::InputFiltering::DeadBand);
        bool auxSign = u.aux > SWConfig::InputFiltering::DeadBand;
        if (auxSign != lastAuxSign) {
            u.auxPressed = true; // only this tick
        }
        lastAuxSign = auxSign;
    }

    
    const bool braking = (u.throttle < -SWConfig::InputFiltering::DeadBand);

    if (braking) {
        if (!isTapping) {
            isTapping = true;
            lastTap = nowMs;
            if (tapCount == 0) firstTap = lastTap;
        }
    } else if (isTapping) {
        // released
        if ((nowMs - lastTap) < SWConfig::UserBehaviour::TapTimeMs) {
            tapCount++;
            if ((nowMs - firstTap) < SWConfig::UserBehaviour::DoubleTapTimeMs)
            {
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
        if (s.batVoltage < ctx.params->Controller.VoltageOff) return Severity::Off;
        if (s.boardTemp >= ctx.params->Controller.TempOff)    return Severity::Off;

        // Warning conditions
        if (s.batVoltage < ctx.params->Controller.VoltageWarn) return Severity::Warn;
        if (s.boardTemp  >= ctx.params->Controller.TempWarn)   return Severity::Warn;

        return Severity::Ok;
    };

    // 0) Highest priority: explicit poweroff request
    if (state.reqPowerOff) {
        return Axle::RemoteCommand::CmdPowerOff;
    }

    // 1) Idle-based shutdown (hard)
    const uint32_t timeUserIdle = ctx.nowMs - state.lastUserInput;
    if (timeUserIdle > SWConfig::UserBehaviour::MaxUserIdleBeforeShutdownMs) {
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
    if (timeUserIdle > SWConfig::UserBehaviour::MaxUserIdleWarnMs) {
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
    state.brakeLight = (ctx.user.throttle <= -SWConfig::InputFiltering::DeadBand);
    state.tailLight = (state.currGear != Gear::N);
    state.loBeam = (state.currGear == Gear::D);
    state.reverseLight = (state.currGear == Gear::R);
}


void DriveTrain::CheckGear(const TickContext& ctx, VehicleState& state)
{
    if (!ctx.user.detected) {
        // reset to neutral after first standstill
        int16_t speed = 0;
        if (ctx.currFront) speed = max(ctx.currFront->sample.speedL_meas, ctx.currFront->sample.speedR_meas);
        if (ctx.currRear) speed = max(speed, max(ctx.currRear->sample.speedL_meas, ctx.currRear->sample.speedR_meas));
        if (speed == 0) {
            state.currGear = Gear::N; // switch to N if standstill or no motor feedback from any controller
        }
        return;
    }
    // check for double-tap on the brake while not moving
    if (ctx.user.doubleTap && state.vehicleSpeed == 0) {
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
    dec.torques = tv.Compute(ctx, state.currGear);
    state.vehicleSpeed = dec.torques.vehicleSpeedAbs;

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
        if (ctx.nowMs - lastExtSteerCmd > SWConfig::UserBehaviour::ExternalTimeoutMs) {
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
                uint16_t IDmaxPower, IDspdFwd, IDspdRev;
                Tuning::IdByKey("maxDrivePower", IDmaxPower);
                Tuning::IdByKey("maxSpeedFwd", IDspdFwd);
                Tuning::IdByKey("maxSpeedRev", IDspdRev);
                Tuning::SetByID(&ctx.params->TV, IDmaxPower, extCmd.p1.u16);
                Tuning::SetByID(&ctx.params->TV, IDspdFwd, extCmd.p2.u16);
                Tuning::SetByID(&ctx.params->TV, IDspdRev, extCmd.p3.u16);
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
            case DriveCommand::TuneTVParam:
                Tuning::SetByID(&ctx.params->TV, extCmd.p1.u16, extCmd.p2.f16); // clamps inside
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
    DriveParams params = DriveParams::Defaults();

    for (;;) {
        const uint32_t nowMs = millis();

        TickContext  ctx = BuildContext(nowMs);
        ctx.params = &params; 
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