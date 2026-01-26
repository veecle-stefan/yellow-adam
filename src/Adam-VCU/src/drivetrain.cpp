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
// Double-tap = two short brake "press->release" cycles within DoubleTapTimeMs.
// Tap is counted on RELEASE, and only if press duration <= TapTimeMs.
// Window starts at the first counted release.
DriveTrain::UserCmd DriveTrain::ReadUserCmd(
    RCinput::UserInput ch1, RCinput::UserInput ch2, RCinput::UserInput ch3, uint32_t nowMs)
{
    struct TapFSM
    {
        bool pressed = false;
        uint32_t pressMs = 0;

        uint8_t taps = 0;
        uint32_t windowStartMs = 0; // time of first counted release
    };
    static TapFSM fsm{};
    static bool lastAuxSign = false;

    UserCmd u{};

    // ---- Input validity / defaults ----
    if (!ch1 || !ch2 || !ch3)
    {
        u.throttle = -100;
        u.steering = 0;
        u.aux = 0;
        u.detected = false;

        // Reset tap FSM on signal loss
        fsm = TapFSM{};
        return u;
    }

    // ---- Populate user input ----
    u.throttle = *ch1;
    u.steering = *ch3;
    u.aux = *ch2;
    u.detected = true;

    u.someInput = (std::abs(u.throttle) > SWConfig::InputFiltering::DeadBand) ||
                  (std::abs(u.steering) > SWConfig::InputFiltering::DeadBand);

    const bool auxSign = (u.aux > SWConfig::InputFiltering::DeadBand);
    if (auxSign != lastAuxSign)
        u.auxPressed = true;
    lastAuxSign = auxSign;

    // ---- Double-tap on brake ----
    const bool brakeDown = (u.throttle < -SWConfig::InputFiltering::DeadBand);

    // Expire an unfinished sequence (even if we never get a clean 2nd tap)
    if (fsm.taps > 0 && (nowMs - fsm.windowStartMs) > SWConfig::UserBehaviour::DoubleTapTimeMs)
    {
        fsm.taps = 0;
        fsm.windowStartMs = 0;
    }

    // Edge detect brake press/release
    if (brakeDown && !fsm.pressed)
    {
        // 'Press' edge
        fsm.pressed = true;
        fsm.pressMs = nowMs;
    }
    else if (!brakeDown && fsm.pressed)
    {
        // 'Release' edge
        fsm.pressed = false;

        const uint32_t heldMs = nowMs - fsm.pressMs;
        if (heldMs <= SWConfig::UserBehaviour::TapTimeMs)
        {
            // Count this tap on release
            if (fsm.taps == 0)
                fsm.windowStartMs = nowMs;
            fsm.taps++;

            if (fsm.taps == 2 &&
                (nowMs - fsm.windowStartMs) <= SWConfig::UserBehaviour::DoubleTapTimeMs)
            {
                u.doubleTap = true;
                fsm.taps = 0;
                fsm.windowStartMs = 0;
            }
        }
        else
        {
            // Long press -> explicitly not part of any sequence
            fsm.taps = 0;
            fsm.windowStartMs = 0;
        }
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

DriveTrain::SafetyDecision DriveTrain::ControllerSafety(const TickContext& ctx, const VehicleState& state, const DriveTrain::Requests r)
{
    // ---- Melody handling (minimal state machine) ----------------------------

    static constexpr uint8_t kIdleWarn[10] = {10, 0, 0, 0, 0, 1, 0, 0, 0, 0};
    static constexpr uint8_t kCtrlWarn[10] = {2, 2, 0, 0, 2, 2, 0, 0, 2, 2};
    static constexpr uint8_t kFail[10] = {15, 15, 0, 1, 1, 1, 1, 1, 1, 1};
    static constexpr uint8_t kSuccess[10] = {1, 1, 0, 0, 5, 5, 5, 0, 0, 0};

    struct Melody
    {
        const uint8_t *tones = nullptr; // points to one of the arrays above
        uint8_t i = 10;                 // 10 == idle / finished
    };
    static Melody mel;
    static uint8_t melSlowdown = 0;

    auto isPlaying = [&]
    { return mel.tones && mel.i < 10; };

    auto startMelody = [&](const uint8_t *tones)
    {
        if (isPlaying())
            return; // do not restart while playing
        mel.tones = tones;
        mel.i = 0;
        melSlowdown = 0;
    };
    // ---- Safety logic --------------------------------------------------------

    enum class Severity : uint8_t
    {
        Ok = 0,
        Warn = 1,
        Off = 2
    };
    SafetyDecision saf{};

    auto evalOne = [&](const std::optional<Axle::HistoryFrame> &fb) -> Severity
    {
        if (!fb)
            return Severity::Ok;
        const auto &s = fb->sample;

        if (s.batVoltage < ctx.params->Controller.VoltageOff ||
            s.boardTemp >= ctx.params->Controller.TempOff)
            return Severity::Off;

        if (s.batVoltage < ctx.params->Controller.VoltageWarn ||
            s.boardTemp >= ctx.params->Controller.TempWarn)
            return Severity::Warn;

        return Severity::Ok;
    };

    // Explicit poweroff request
    if (r.reqPowerOff)
    {
        saf.cmd = Axle::RemoteCommand::CmdPowerOff;
        return saf;
    }

    // Idle-based shutdown
    const uint32_t timeUserIdle = ctx.nowMs - state.lastUserInput;
    if (timeUserIdle > SWConfig::UserBehaviour::MaxUserIdleBeforeShutdownMs)
    {
        saf.cmd = Axle::RemoteCommand::CmdPowerOff;
        return saf;
    }

    // Evaluate both controllers
    Severity sev = std::max(evalOne(ctx.currFront), evalOne(ctx.currRear));
    if (sev == Severity::Off)
    {
        saf.cmd = Axle::RemoteCommand::CmdPowerOff;
        return saf;
    }

    // ---- Decide which melody is requested (priority by overwrite) -----------

    const uint8_t *req = nullptr;

    if (timeUserIdle > SWConfig::UserBehaviour::MaxUserIdleWarnMs)
    {
        req = kIdleWarn;
    }

    if (sev == Severity::Warn || !ctx.currFront || !ctx.currRear)
    {
        req = kCtrlWarn;
    }

    // user request beeps override warnings
    if (r.beep != BeepRequest::NoBeep)
    {
        req = (r.beep == BeepRequest::BeepFailure) ? kFail : kSuccess;
    }

    if (req)
    {
        startMelody(req);
    }

    // ---- Advance melody ------------------------------------------------------

    if (isPlaying())
    {
        saf.warningBeep = mel.tones[mel.i];
        if (++melSlowdown == 5)
        {
            melSlowdown = 0;
            ++mel.i;
        }
    }

    return saf;
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
        if (ctx.currFront) speed = max(abs(ctx.currFront->sample.speedL_meas), abs(ctx.currFront->sample.speedR_meas));
        if (ctx.currRear) speed = max(speed, static_cast<int16_t>(max(abs(ctx.currRear->sample.speedL_meas), abs(ctx.currRear->sample.speedR_meas))));
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

DriveTrain::TickDecision DriveTrain::ComputeDecision(const TickContext& ctx, VehicleState& state, const DriveTrain::Requests r)
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
    SafetyDecision saf = ControllerSafety(ctx, state, r);
    dec.cmd = saf.cmd;
    dec.beep = max(dec.beep, saf.warningBeep);

    // 5) lights intent
    ComputeLights(ctx, dec, state);

    return dec;
}

void DriveTrain::ApplyDecision(const TickDecision& dec, VehicleState& state)
{
    // Motors
    uint8_t holdFlagsF = (dec.torques.hold.hFL ? Axle::FLG_STANDSTILL_FORCE_L : 0) |
                         (dec.torques.hold.hFR ? Axle::FLG_STANDSTILL_FORCE_R : 0);
    uint8_t holdFlagsR = (dec.torques.hold.hRL ? Axle::FLG_STANDSTILL_FORCE_L : 0) |
                         (dec.torques.hold.hRR ? Axle::FLG_STANDSTILL_FORCE_R : 0);

    axleF.Send(dec.torques.fl, dec.torques.fr, dec.cmd, dec.beep, holdFlagsF);
    axleR.Send(dec.torques.rl, dec.torques.rr, dec.cmd, dec.beep, holdFlagsR);


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
    st.cmd_fl  = lastFrontFb->sample.cmdL;
    st.cmd_fr  = lastFrontFb->sample.cmdR;
    st.voltage_front = lastFrontFb->sample.batVoltage;
    st.temp_front    = lastFrontFb->sample.boardTemp;
  }
  if (lastRearFb) {
    st.haveRear = true;
    st.curr_rl = lastRearFb->sample.currL_meas;
    st.curr_rr = lastRearFb->sample.currR_meas;
    st.vel_rl  = lastRearFb->sample.speedL_meas;
    st.vel_rr  = lastRearFb->sample.speedR_meas;
    st.cmd_rl  = lastRearFb->sample.cmdL;
    st.cmd_rr  = lastRearFb->sample.cmdR;
    st.voltage_rear = lastRearFb->sample.batVoltage;
    st.temp_rear    = lastRearFb->sample.boardTemp;
  }

  xQueueOverwrite(statusQueue, &st);
}

DriveTrain::Requests DriveTrain::ProcessExtCmds(TickContext& ctx, VehicleState& state)
{
    // keep track of timing for external control
    static uint32_t lastExtSteerCmd = 0;
    Requests r = {};

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
                // only allow gear changes when speed == 0 and user present
                if ((state.vehicleSpeed == 0) && (ctx.user.detected || state.externalControl)) {
                    state.currGear = static_cast<Gear>(extCmd.p1.u8);
                    r.beep = SafeBeep(r.beep, BeepRequest::BeepSuccess);
                } else {
                    r.beep = SafeBeep(r.beep, BeepRequest::BeepFailure);
                }
                break;
            case DriveCommand::SetIndicators:
                state.indicatorsL = extCmd.p1.onOff;
                state.indicatorsR = extCmd.p2.onOff;
                break;
            case DriveCommand::SetPowerLimit:
                {
                    uint16_t IDmaxPower, IDspdFwd, IDspdRev;
                    bool success = true;
                    Tuning::IdByKey("maxDrivePower", IDmaxPower);
                    Tuning::IdByKey("maxSpeedFwd", IDspdFwd);
                    Tuning::IdByKey("maxSpeedRev", IDspdRev);
                    success &= Tuning::SetByID(&ctx.params->TV, IDmaxPower, extCmd.p1.u16);
                    success &= Tuning::SetByID(&ctx.params->TV, IDspdFwd, extCmd.p2.u16);
                    success &= Tuning::SetByID(&ctx.params->TV, IDspdRev, extCmd.p3.u16);
                    r.beep = SafeBeep(r.beep, success ? BeepRequest::BeepSuccess : BeepRequest::BeepFailure);
                }
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
                r.reqPowerOff = true;
                break;
            case DriveCommand::TuneTVParam:
                {
                    bool success = Tuning::SetByID(&ctx.params->TV, extCmd.p1.u16, extCmd.p2.f16); // will fail if outside min/max
                    r.beep = SafeBeep(r.beep, success ? BeepRequest::BeepSuccess : BeepRequest::BeepFailure);
                }
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

    return r;
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
        Requests r = ProcessExtCmds(ctx, state);
        TickDecision dec = ComputeDecision(ctx, state, r);
        ApplyDecision(dec, state);

        PublishStatus(ctx, dec, state);

        // roll “last” forward for next tick
        lastFrontFb = ctx.currFront; // also overwrite last if currFront is nullopt 
        lastRearFb  = ctx.currRear; // so it's never too old in terms of time

        vTaskDelayUntil(&lastWakeTime, period);
    }
}