#pragma once

#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/rmt_rx.h"   // new RMT RX driver

class RCinput
{
public:
    RCinput();
    
    // Reads PPM/PWM channels into -1000..+1000 range
    // ch1 = throttle (accel/brake), ch2 = steering, ch3 = aux (ignored for now)
    void ReadChannels(int16_t& throttle, int16_t& steering, int16_t& aux);

protected:
   
    // ----- PPM configuration -----
    // Typical RC pulse range in microseconds
    static constexpr uint16_t PpmMinUs    = 1000;
    static constexpr uint16_t PpmMaxUs    = 2000;
    static constexpr uint16_t PpmCenterUs = 1500;

    // track last update timestamps for failsafe (µs)
    static volatile uint64_t s_lastUpdateUs[3];

    // Very small spinlock to protect multi-field updates
    static portMUX_TYPE s_rcMux;

  
    
    // Map a pulse width (µs) to -1000..+1000
    static int16_t mapPulseToCommand(uint16_t pulseWidthUs);
    static void initRmtRxChannel(uint8_t chIndex, gpio_num_t gpio);
     // RMT RX callback (new driver)
    static bool IRAM_ATTR rmtRxDoneCallback(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_data);

    // Give ISRs access to static state
    static RCinput* instance;

    // Last measured pulse widths for each channel (in microseconds)
    // ch 0 -> PIN_PPM_CH1, ch 1 -> PIN_PPM_CH2, ch 2 -> PIN_PPM_CH3
    static volatile uint16_t s_pulseWidthUs[3];
};