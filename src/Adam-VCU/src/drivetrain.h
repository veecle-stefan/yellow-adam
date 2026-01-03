#pragma once

#include <Arduino.h>
#include "axle.h"
#include "lights.h"
#include "rcinput.h"

namespace DriveConfig {
    static constexpr int16_t DeadBand = 50;
    
    namespace Brakes {
        static constexpr uint16_t DetectThreshold = 50;
        static constexpr uint32_t TapTime = 200;
        static constexpr uint32_t DoubleTapTime = 1000;
        static constexpr uint16_t AntiReversingSpeed = 100;
    }
    
    namespace Controller {
        static constexpr uint16_t VoltageWarn = 3500; // 35V
        static constexpr uint16_t VoltageOff  = 3300; // 33V
        static constexpr uint16_t TempOff     = 500;  // 50 C
    }

    namespace TV { // Torque Vectoring

    }
}

class DriveTrain
{
public:
    enum Gear {
        Forward = 1,
        Reverse = 2,
        Neutral = 3
    };

    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);

    // Control task period in milliseconds
    static constexpr uint16_t ControlPeriodMs = 20; // 20 ms / 50 Hz

    std::optional<int16_t> val_accell;
    std::optional<int16_t> val_steer;
    std::optional<int16_t> val_aux;
    Axle::MotorStates lastFrontFb;
    Axle::MotorStates lastRearFb;
    int16_t sentTorqueFL = 0, sentTorqueFR = 0, sentTorqueRL = 0, sentTorqueRR = 0;

protected:
    RCinput ch1;
    RCinput ch2;
    RCinput ch3;
    Axle&  axleF;   // front axle: left/right motors
    Axle&  axleR;   // rear axle: left/right motors
    Lights& lights; // currently unused here, but kept
    Gear currGear;

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
                        bool userDetected,
                        uint32_t currTime
                        );
    bool UpdateLights(Axle::MotorStates fr, Axle::MotorStates rr, int16_t accellerate, bool userDetected);

    /// @brief Checks motor controller statuses and detects undervoltage or overtemp and issues emergency commands
    /// @param front 
    /// @param rear 
    /// @return 
    Axle::RemoteCommand ControllerSafety(Axle::MotorStates front, Axle::MotorStates rear);

    /// @brief Tests if user commands are present, not stale and valid. Also detects taps
    /// @param thr Raw optional PWM input
    /// @param aux Raw optional PWM input
    /// @param str Raw optional PWM input
    /// @param currTime
    /// @param out_accell 
    /// @param out_steer 
    /// @param out_aux 
    /// @return Whether inputs were valid and user present (not stale)
    bool ParseUserInput(RCinput::UserInput thr, RCinput::UserInput aux, RCinput::UserInput str, uint32_t currTime, int16_t& out_accell, int16_t& out_steer, int16_t& out_aux, bool& doubleTap);

};