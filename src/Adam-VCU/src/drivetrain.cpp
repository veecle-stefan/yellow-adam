#include "drivetrain.h"
#include "hwconfig.h"
#include "swconfig.h"

// ----- Constructor -----

DriveTrain::DriveTrain(Axle& axleF, Axle& axleR, Lights& lights)
    : ch1(HWConfig::Pins::PPM::Channel1)
    , ch2(HWConfig::Pins::PPM::Channel2)
    , ch3(HWConfig::Pins::PPM::Channel3)
    , axleF(axleF)
    , axleR(axleR)
    , lights(lights)
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

bool DriveTrain::UpdateLights(Axle::MotorStates fr, Axle::MotorStates rr, int16_t accellerate)
{
    // if going reverse, turn on reverse light
    if (fr.has_value() && rr.has_value() && !fr->isStale && !rr->isStale &&
        fr->sample.speedL_meas < 0 && rr->sample.speedR_meas < 0) {
        lights.SetReverseLight(true);
        return true;
    } else {
        lights.SetReverseLight(false);
        return false;
    }

    // if decelating, turn on brake light
    lights.SetBrakeLight(accellerate <= DriveTrain::BrakeDecelThreshold);
    
}

// ----- Periodic control task -----

void DriveTrain::ControlTask()
{
    const TickType_t period       = pdMS_TO_TICKS(ControlPeriodMs);
    TickType_t       lastWakeTime = xTaskGetTickCount();
    char rcinputdebug[30];
    uint32_t lastUpdate = 0;
    bool failSafe = false;

    for (;;) {
        uint32_t currTime = millis();

        // 1. Read input channels
        val_accell = *ch1;
        val_steer  = *ch3;
        val_aux    = *ch2;

        // 2. Get motor status (from last update)
        Axle::MotorStates currFront = axleF.GetLatestFeedback(currTime);   // optional<HistoryFrame>
        Axle::MotorStates currRear  = axleR.GetLatestFeedback(currTime);

        int16_t accellerate = 0; // safe defaults
        int16_t steering = 0;
        int16_t aux = 0;
        // only if all three RC values are valid and up-to-date
        if (val_accell.has_value() && val_steer.has_value() && val_aux.has_value()) {
            accellerate = *val_accell;
            steering = *val_steer;
            aux = *val_aux;
            if (failSafe) {
                lights.SetHeadlight(Lights::HeadLightState::DRL);
                lights.SetIndicators(false, false, false, false);
            }
            failSafe = false;
            if (currTime - lastUpdate > 100) {
                snprintf(rcinputdebug, 30, "%i;%i;%i", accellerate, steering, aux);
                Serial.println(rcinputdebug);
                lastUpdate = currTime;
            }
        } else {
            if (!failSafe) {
                lights.SetHeadlight(Lights::HeadLightState::Off);
                lights.SetIndicators(true, true, true, true); // hazards to show no data
            }
            failSafe = true;
        }

        // 3. Run torque vectoring
        TorqueVectoring(accellerate, steering, sentTorqueFL, sentTorqueFR, sentTorqueRL, sentTorqueRR, currFront, currRear, this->lastFrontFb, this->lastRearFb, failSafe, currTime);
        this->lastFrontFb = currFront;
        this->lastRearFb = currRear;

        //FIXME: temporary to test only rear motors
        if(aux > 500) {
            sentTorqueFL = 0;
            sentTorqueRL = 0;
        } else {
            sentTorqueFR = 0;
            sentTorqueRR = 0;
        }
        

        // 3. Update all 4 motors via the two axles
        axleF.Send(sentTorqueFL, sentTorqueFR); // front left/right
        axleR.Send(sentTorqueRL, sentTorqueRR); // rear left/right

        //4. Update Lights based on throttle, speed etc.
        UpdateLights(currFront, currRear, accellerate);

        // 4. Wait until next cycle
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
                            bool failSafe,
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