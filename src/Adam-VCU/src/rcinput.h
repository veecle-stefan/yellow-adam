#pragma once
#include <Arduino.h>
#include <driver/rmt_rx.h>
#include <optional>

class RCPWMinput {
public:
  // Map 1000–2000 µs to -1000 to 1000
    static constexpr int16_t MIN_US = 1000;
    static constexpr int16_t MAX_US = 2000;
    static constexpr int16_t MID_US = (MIN_US + MAX_US) / 2;
    static constexpr int16_t OUTPUT_RANGE = 1000;
    static constexpr int16_t DEADBAND = 50;

    // pin = GPIO_NUM_x (not int!), expects classic RC PWM 1000–2000 µs @ ~50 Hz
    explicit RCPWMinput(gpio_num_t pin);

    // returns ESP_OK on success
    esp_err_t begin();

    int16_t mapRange(uint16_t timeUs);
    std::optional<int16_t> value();
    std::optional<int16_t> operator*();

private:
    static bool IRAM_ATTR on_rx_done_static(rmt_channel_handle_t channel,
                                            const rmt_rx_done_event_data_t *edata,
                                            void *user_data);
    bool on_rx_done(const rmt_rx_done_event_data_t *edata);

    gpio_num_t _pin;
    rmt_channel_handle_t _channel = nullptr;

    // buffer for received symbols, stays in RAM
    static constexpr size_t SYMBOL_BUF_LEN = 64;
    rmt_symbol_word_t _symbols[SYMBOL_BUF_LEN];

    rmt_receive_config_t _rx_cfg{};
    volatile uint16_t _pulseUs = 1500;  // sane default
    volatile uint32_t       _updateCount = 0;
    uint32_t _lastRead = 0;

};