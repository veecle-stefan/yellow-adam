#pragma once

#include <Arduino.h>
#include "driver/rmt.h"
#include "axle.h"
#include "lights.h"

class DriveTrain
{
public:
    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);

    // Control task period in milliseconds
    static constexpr uint16_t ControlPeriodMs = 100; // 100 ms

protected:
    Axle&  axleF;   // front axle: left/right motors
    Axle&  axleR;   // rear axle: left/right motors
    Lights& lights; // currently unused here, but kept

    // ----- PPM configuration -----
    // Typical RC pulse range in microseconds
    static constexpr uint16_t PpmMinUs    = 1000;
    static constexpr uint16_t PpmMaxUs    = 2000;
    static constexpr uint16_t PpmCenterUs = 1500;
    static constexpr rmt_channel_t kRmtChannels[3] = {
        RMT_CHANNEL_0,
        RMT_CHANNEL_1,
        RMT_CHANNEL_2
    };

    // Ring buffers for each RMT channel
    static RingbufHandle_t s_rmtRingBuf[3];

    // ----- Background control task -----
    void ControlTask();

    // Reads PPM/PWM channels into -1000..+1000 range
    // ch1 = throttle (accel/brake), ch2 = steering, ch3 = aux (ignored for now)
    void ReadChannels(int16_t& throttle, int16_t& steering, int16_t& aux);

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

    
    // Map a pulse width (µs) to -1000..+1000
    static int16_t mapPulseToCommand(uint16_t pulseWidthUs);
    static void initRmtRxChannel(uint8_t chIndex, gpio_num_t gpio);
    static void updatePulseFromRmt(uint8_t chIndex);


    // Give ISRs access to static state
    static DriveTrain* instance;

    // Last measured pulse widths for each channel (in microseconds)
    // ch 0 -> PIN_PPM_CH1, ch 1 -> PIN_PPM_CH2, ch 2 -> PIN_PPM_CH3
    static volatile uint16_t s_pulseWidthUs[3];
};