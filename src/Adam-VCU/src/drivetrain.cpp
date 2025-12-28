#include "drivetrain.h"
#include "pindefs.h"

// ----- Constructor -----

DriveTrain::DriveTrain(RCinput* input, Axle& axleF, Axle& axleR, Lights& lights)
    : input(input)
    , axleF(axleF)
    , axleR(axleR)
    , lights(lights)
{
    // Start background control task (100 ms period)
    xTaskCreate(
        [](void* pvParameters) {
            static_cast<DriveTrain*>(pvParameters)->ControlTask();
            vTaskDelete(nullptr);
        },
        "DriveTrain",
        4096,
        this,
        1,
        nullptr
    );
}


// ----- Periodic control task -----

void DriveTrain::ControlTask()
{
    const TickType_t period       = pdMS_TO_TICKS(ControlPeriodMs);
    TickType_t       lastWakeTime = xTaskGetTickCount();

    for (;;) {
        // 1. Read input channels
        int16_t throttle = 0;
        int16_t steering = 0;
        int16_t aux      = 0;
        this->input->ReadChannels(throttle, steering, aux);

        // 2. Run torque vectoring
        int16_t fl = 0, fr = 0, rl = 0, rr = 0;
        TorqueVectoring(throttle, steering, fl, fr, rl, rr);

        // 3. Update all 4 motors via the two axles
        axleF.Send(fl, fr); // front left/right
        axleR.Send(rl, rr); // rear left/right

        //TODO: (Lights could be updated based on throttle, speed etc. here)

        // 4. Wait until next 100ms cycle
        vTaskDelayUntil(&lastWakeTime, period);
    }
}


// ----- Very naive torque vectoring -----

void DriveTrain::TorqueVectoring(int16_t throttle,
                                 int16_t steering,
                                 int16_t& frontLeft,
                                 int16_t& frontRight,
                                 int16_t& rearLeft,
                                 int16_t& rearRight)
{
    // Normalize steering to [-1.0 .. +1.0]
    float s = static_cast<float>(steering) / 1000.0f;
    if (s >  1.0f) s =  1.0f;
    if (s < -1.0f) s = -1.0f;

    // Base torque from throttle, also in [-1000..+1000]
    float T = static_cast<float>(throttle);

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