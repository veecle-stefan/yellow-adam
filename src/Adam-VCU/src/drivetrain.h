#pragma once

#include <Arduino.h>
#include "axle.h"
#include "lights.h"
#include "rcinput.h"

class DriveTrain
{
public:
    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);

    // Control task period in milliseconds
    static constexpr uint16_t ControlPeriodMs = 20; // 20 ms / 50 Hz
    static constexpr uint16_t BrakeDecelThreshold = 200; 

    std::optional<int16_t> val_accell;
    std::optional<int16_t> val_steer;
    std::optional<int16_t> val_aux;
    Axle::MotorStates lastFrontFb;
    Axle::MotorStates lastRearFb;
    int16_t sentTorqueFL = 0, sentTorqueFR = 0, sentTorqueRL = 0, sentTorqueRR = 0;

protected:
    RCPWMinput ch1;
    RCPWMinput ch2;
    RCPWMinput ch3;
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
                         int16_t& rearRight,
                        Axle::MotorStates currF,
                        Axle::MotorStates currR,
                        Axle::MotorStates lastF,
                        Axle::MotorStates lastR,
                        bool failSafe,
                        uint32_t currTime
                        );
    bool UpdateLights(Axle::MotorStates fr, Axle::MotorStates rr, int16_t accellerate);

};