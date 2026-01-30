#pragma once
#include <optional>
#include "axle.h"
#include "lights.h"
#include "rcinput.h"
#include "driveparams.h"
#include "torquevectoring.h"
#include "melodyplayer.h"

class DriveTrain
{
public:
    static constexpr uint16_t ControlPeriodMs = 20; // 50 Hz

    using Gear = TorqueVectoring::Gear;

    enum BeepRequest : uint8_t {
        NoBeep = 0,
        BeepSuccess = 1,
        BeepFailure = 2
    };

    static constexpr BeepRequest SafeBeep(const BeepRequest old, const BeepRequest should) {
        return max(old, should);
    }

    struct VehicleState
    {
        bool hazards = false;
        bool indicatorsR = false;
        bool indicatorsL = false;
        bool brakeLight = false;
        bool reverseLight = false;
        bool DRL = false;
        bool loBeam = false;
        bool hiBeam = false;
        bool tailLight = false;
        Gear currGear = Gear::N;
        uint16_t vehicleSpeed = 0;
        uint32_t lastUserInput = 0;
        bool externalControl = false;
        uint32_t lastExtThrottle = 0;
        uint32_t lastExtSteering = 0;
    };

    struct DriveTrainStatus
    {
        uint32_t ts_ms = 0;

        int16_t throttle = 0;
        int16_t steering = 0;
        int16_t aux = 0;
        bool userDetected = false;

        // last applied torques
        int16_t tq_fl = 0, tq_fr = 0, tq_rl = 0, tq_rr = 0;

        // motor feedback (last known good)
        int16_t curr_fl = 0, curr_fr = 0, curr_rl = 0, curr_rr = 0;
        int16_t vel_fl = 0, vel_fr = 0, vel_rl = 0, vel_rr = 0;
        int16_t cmd_fl = 0, cmd_fr = 0, cmd_rl = 0, cmd_rr = 0;
        uint16_t voltage_front = 0, voltage_rear = 0; // 35.00V => 3500 etc
        uint16_t temp_front = 0, temp_rear = 0;       // 50.0C => 500 etc
        bool haveFront = false;
        bool haveRear = false;

        VehicleState state;
    };

    enum DriveCommand
    {
        SetGear,
        SetIndicators,
        SetPowerLimit,
        EnableExternalControl,
        SetHeadlight,
        Steer,
        PowerOff,
        TuneTVParam
    };

    union CommandParameter
    {
        int16_t i16;
        uint16_t u16;
        uint8_t u8;
        float f16;
        bool onOff;
    };

    struct CommandItem
    {
        DriveCommand cmd;

        CommandParameter p1;
        CommandParameter p2;
        CommandParameter p3;
    };

    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);
    QueueHandle_t GetStatusQueue() const;
    void Shutdown();
    void SendCommand(const CommandItem* cmd);
    void SendGear(TorqueVectoring::Gear newGear);
    void SendIndicators(bool left, bool right);
    void SendPowerLimit(uint16_t maxThrottle, uint16_t maxSpeedFwd, uint16_t maxSpeedRev);
    void SendExternalControl(bool enable);
    void SendHeadlight(uint8_t mode, bool on);
    void SendPowerOff();
    void SendSteer(int16_t throttle, int16_t steer);
    void SendTuneTV(uint16_t id, float value);

protected :
    using MotorStates = std::optional<Axle::HistoryFrame>;
    using TickContext = TorqueVectoring::TickContext;
    using UserCmd = TorqueVectoring::UserCmd;
    using UserInput = RCinput::UserInput;

    enum class Severity : uint8_t
    {
        Ok = 0,
        Warn = 1,
        Off = 2
    };

    struct SafetyDecision
    {
        Axle::RemoteCommand cmd = Axle::RemoteCommand::CmdNOP;
        uint8_t payload = 0;
    };

    struct TickDecision {
        TorqueVectoring::Torques torques{};
        SafetyDecision safeCmd;
        bool failSafe = false;
    };

 
    struct Requests {
        BeepRequest beep = BeepRequest::NoBeep;
        bool reqPowerOff = false;
    };

    // ----- Members -----
    QueueHandle_t statusQueue = NULL;
    QueueHandle_t extCmdQueue = NULL;
    TaskHandle_t processTask = NULL;
    RCinput ch1;
    RCinput ch2;
    RCinput ch3;
    Axle&    axleF;
    Axle&    axleR;
    TorqueVectoring tv;
    Lights&  lights;

    MotorStates lastFrontFb;
    MotorStates lastRearFb;

    MelodyPlayer melody;

    // ----- Background control task -----
    void ControlTask();

    // ----- Pipeline -----
    TickContext  BuildContext(uint32_t nowMs);
    TickDecision ComputeDecision(const TickContext &ctx, VehicleState &state, const DriveTrain::Requests r);
    Requests ProcessExtCmds(TickContext& ctx, VehicleState& state);
    void         ApplyDecision(const TickDecision& dec, VehicleState& state);
    void         CheckGear(const TickContext& ctx, VehicleState& state);

    // ----- Helpers (small, single-purpose) -----
    void PublishStatus(const TickContext& ctx, const TickDecision& dec, const VehicleState& state);

    static UserCmd        ReadUserCmd(UserInput ch1, UserInput ch2, UserInput ch3, uint32_t nowMs);
    static Severity EvalControllerHealth(const TickContext &ctx);
    SafetyDecision ControllerSafety(const TickContext &ctx, const VehicleState &state, const DriveTrain::Requests r);
    static void           ComputeLights(const TickContext& ctx, TickDecision& dec,  VehicleState& state);
};