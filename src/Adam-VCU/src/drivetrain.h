#pragma once

#include <Arduino.h>
#include "axle.h"
#include "lights.h"
#include "rcinput.h"

class DriveTrain
{
public:
    DriveTrain(RCinput* input, Axle& axleF, Axle& axleR, Lights& lights);

    // Control task period in milliseconds
    static constexpr uint16_t ControlPeriodMs = 20; // 20 ms / 50 Hz

protected:
    RCinput* input;
    Axle&  axleF;   // front axle: left/right motors
    Axle&  axleR;   // rear axle: left/right motors
    Lights& lights; // currently unused here, but kept


    // ----- Background control task -----
    void ControlTask();

    // Very naive torque vectoring:
    //  - throttle: -1000..+1000 (brake/accel)
    //  - steering: -1000..+1000 (left/right)
    // Outputs per-wheel torques in same range.
    void TorqueVectoring(int16_t throttle,
                         int16_t steering,
                         int16_t& frontLeft,
                         int16_t& frontRight,
                         int16_t& rearLeft,
                         int16_t& rearRight);

};