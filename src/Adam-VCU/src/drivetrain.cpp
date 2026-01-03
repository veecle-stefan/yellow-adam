#include "drivetrain.h"
#include "hwconfig.h"
#include "swconfig.h"

// ----- Constructor -----

DriveTrain::DriveTrain(Axle& axleF, Axle& axleR, Lights& lights)
    : ch1(HWConfig::Pins::PPM::Channel1, DriveConfig::DeadBand)
    , ch2(HWConfig::Pins::PPM::Channel2, DriveConfig::DeadBand)
    , ch3(HWConfig::Pins::PPM::Channel3, DriveConfig::DeadBand)
    , axleF(axleF)
    , axleR(axleR)
    , lights(lights)
    , currGear(Neutral)
{

    ch1.begin();
    ch2.begin();
    ch3.begin();

    // Start background control task (100 ms period)
    xTaskCreatePinnedToCore(
        [](void* pvParameters) {
            static_cast<DriveTrain*>(pvParameters)->ControlTask();
            vTaskDelete(nullptr);
        },
        "DriveTrain",
        SWConfig::Tasks::MinStakSize,
        this,
        SWConfig::Tasks::PrioHigh,
        nullptr,
        SWConfig::CoreAffinity::CoreApp
    );
}

bool DriveTrain::UpdateLights(Axle::MotorStates fr, Axle::MotorStates rr, int16_t accellerate, bool userDetected)
{
    static bool failSafeVisible = false;
    if (!userDetected) {
        if (!failSafeVisible) {
            lights.SetHeadlight(Lights::HeadLightState::Off);
            lights.SetReverseLight(false);
            lights.SetBrakeLight(false);
            lights.SetTailLight(false);
            lights.SetIndicators(true, true, true, true); // start blinking to indicate failsafe
            failSafeVisible = true;
        }
    } else {
        // restore sane state in case fail-safe was on before
        if (failSafeVisible) {
            lights.SetHeadlight(Lights::HeadLightState::DRL);
            lights.SetTailLight(true);
            lights.SetIndicators(false, false, false, false);
            failSafeVisible = false;
        }

        // from now on, only normal light situations
        lights.SetReverseLight(currGear == DriveTrain::Gear::Reverse);
        lights.SetBrakeLight(accellerate <= DriveConfig::Brakes::DetectThreshold);
    }
    return true;
}

Axle::RemoteCommand DriveTrain::ControllerSafety(Axle::MotorStates front, Axle::MotorStates rear)
{
    Axle::RemoteCommand cmd = Axle::RemoteCommand::CmdNOP;
    if (front.has_value() && rear.has_value()) {
        auto& sFront = front->sample;
        auto& sRear = rear->sample;
        // check if voltage is < absolute lowest allowed -> turn off
        if  ((sFront.batVoltage < DriveConfig::Controller::VoltageOff) || (sRear.batVoltage < DriveConfig::Controller::VoltageOff)) {
            cmd = Axle::RemoteCommand::CmdPowerOff;
        // check if temp >= absolute highest allowed
        } else if ((sFront.boardTemp >= DriveConfig::Controller::TempOff) || (sRear.boardTemp >= DriveConfig::Controller::TempOff)) {
            cmd = Axle::RemoteCommand::CmdPowerOff;
        // check if voltage is at critical level -> beep
        } else if ((sFront.batVoltage < DriveConfig::Controller::VoltageWarn) || (sRear.batVoltage < DriveConfig::Controller::VoltageWarn)) {
            cmd = Axle::RemoteCommand::CmdBeep;
        }
    }
    return cmd;
}

bool DriveTrain::ParseUserInput(
    RCinput::UserInput thr, 
    RCinput::UserInput aux, 
    RCinput::UserInput str,
    uint32_t currTime,
    int16_t& out_accell, 
    int16_t& out_steer, 
    int16_t& out_aux, 
    bool& doubleTap)
{
    static uint32_t firstTap = 0;
    static uint32_t lastTap = 0;
    static bool isTapping = false;
    static uint8_t tapCount = 0;
    bool userDetected = false;
    // only if all three RC values are valid and up-to-date
    if (thr.has_value() && str.has_value() && aux.has_value()) {
        out_accell = *thr;
        out_steer = *str;
        out_aux = *aux;

        // seems valid, now check for taps
        if (out_accell < DriveConfig::Brakes::DetectThreshold) {
            // brake pressed -> detect tap
            if (!isTapping) {
                // was released -> now pressed
                isTapping = true;
                lastTap = currTime;
                if (tapCount == 0) {
                    firstTap = lastTap;
                }
            }
        } else if (isTapping) {
            if (currTime - lastTap < DriveConfig::Brakes::TapTime) {
                tapCount++;
                if (currTime - firstTap < DriveConfig::Brakes::DoubleTapTime) {
                    // valid consecutive tap
                    if (tapCount == 2) {
                        doubleTap = true;
                        tapCount = 0; // reset
                    }
                }
            } else {
                tapCount = 0; // reset
            }
            isTapping = false;
        }

        userDetected = true;
    } else {
        userDetected = false;
    }
    return userDetected;
}

// ----- Periodic control task -----
void DriveTrain::ControlTask()
{
    const TickType_t period       = pdMS_TO_TICKS(ControlPeriodMs);
    TickType_t       lastWakeTime = xTaskGetTickCount();

    for (;;) {
        uint32_t currTime = millis();

        // 2. Get motor status (from last update)
        Axle::MotorStates currFront = axleF.GetLatestFeedback(currTime);   // optional<HistoryFrame>
        Axle::MotorStates currRear  = axleR.GetLatestFeedback(currTime);

        int16_t accellerate = 0; // safe defaults
        int16_t steering = 0;
        int16_t aux = 0;
        bool doubleTap = false;
        bool userDetected = ParseUserInput(*ch1, *ch2, *ch3, currTime, accellerate, steering, aux, doubleTap);

        // 3. Run torque vectoring
        TorqueVectoring(accellerate, steering, sentTorqueFL, sentTorqueFR, sentTorqueRL, sentTorqueRR, currFront, currRear, this->lastFrontFb, this->lastRearFb, userDetected, currTime);
        
        // 4. Update all 4 motors via the two axles
        Axle::RemoteCommand command = ControllerSafety(currFront, currRear);
        axleF.Send(sentTorqueFL, sentTorqueFR, command); // front left/right
        axleR.Send(sentTorqueRL, sentTorqueRR, command); // rear left/right

        //5. Update Lights based on throttle, speed etc.
        UpdateLights(currFront, currRear, accellerate, userDetected);

        //6. Wait until next cycle
        this->lastFrontFb = currFront;
        this->lastRearFb = currRear;
        vTaskDelayUntil(&lastWakeTime, period);
    }
}


// ----- Very naive torque vectoring -----

void DriveTrain::TorqueVectoring(int16_t accellerate,
                                 int16_t steering,
                                 int16_t& frontLeft,
                                 int16_t& frontRight,
                                 int16_t& rearLeft,
                                 int16_t& rearRight,
                                Axle::MotorStates currF,
                            Axle::MotorStates currR,
                            Axle::MotorStates lastF,
                            Axle::MotorStates lastR,
                            bool userDetected,
                            uint32_t currTime
                    )
{
    // Normalize steering to [-1.0 .. +1.0]
    float s = static_cast<float>(steering) / 1000.0f;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    // Base torque from throttle, also in [-1000..+1000]
    float T = static_cast<float>(accellerate);

    // Front axle: simple split based on steering
    // Turn right (s > 0): more torque on FL, less on FR.
    // Turn left  (s < 0): more torque on FR, less on FL.
    constexpr float frontVectorGain = 0.5f; // 0.0 = no effect, 1.0 = strong

    float fl = T * (1.0f + frontVectorGain * s);
    float fr = T * (1.0f - frontVectorGain * s);

    // Rear axle: brake the inner wheel in a corner
    // Inner wheel:
    //  - s > 0 (right turn): inner rear = RR
    //  - s < 0 (left turn):  inner rear = RL
    constexpr float innerBrakeGain = 0.6f; // fraction of torque removed at full steer

    float rl = T;
    float rr = T;

    float absS = (s >= 0.0f) ? s : -s;

    if (s > 0.0f) {
        // Right turn: brake right rear
        rr = T * (1.0f - innerBrakeGain * absS);
    } else if (s < 0.0f) {
        // Left turn: brake left rear
        rl = T * (1.0f - innerBrakeGain * absS);
    }

    // Clamp all outputs back into [-1000, +1000]
    auto clamp = [](float v) -> int16_t {
        if (v >  1000.0f) v =  1000.0f;
        if (v < -1000.0f) v = -1000.0f;
        return static_cast<int16_t>(v);
    };

    frontLeft  = clamp(fl);
    frontRight = clamp(fr);
    rearLeft   = clamp(rl);
    rearRight  = clamp(rr);
}