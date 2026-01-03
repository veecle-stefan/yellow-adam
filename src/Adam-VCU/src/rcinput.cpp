#include "rcinput.h"

RCinput::RCinput(gpio_num_t pin, uint16_t deadBand)
    : DeadBand(deadBand),
    _pin(pin) 
    {

    }

esp_err_t RCinput::begin() {
    // 1 MHz resolution: 1 tick = 1 µs
    rmt_rx_channel_config_t cfg = {
        .gpio_num = _pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1'000'000,
        .mem_block_symbols = SYMBOL_BUF_LEN,
        .flags = {
            .invert_in = false,
            .with_dma  = false,
        },
    };

    esp_err_t err = rmt_new_rx_channel(&cfg, &_channel);
    if (err != ESP_OK) return err;

    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = &RCinput::on_rx_done_static,
    };
    err = rmt_rx_register_event_callbacks(_channel, &cbs, this);
    if (err != ESP_OK) return err;

    err = rmt_enable(_channel);
    if (err != ESP_OK) return err;

    // Use a very small min range to satisfy the driver.
    // We do our own validation in the callback.
    _rx_cfg.signal_range_min_ns = 1'000;         // 1 µs
    _rx_cfg.signal_range_max_ns = 3'000'000;    // 3 ms -> anything longer (the low gap) stops RX

    // Start the first receive; callback re-arms it
    return rmt_receive(_channel, _symbols,
                       sizeof(_symbols), &_rx_cfg);
}

bool IRAM_ATTR RCinput::on_rx_done_static(rmt_channel_handle_t channel,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *user_data) {
    auto *self = static_cast<RCinput *>(user_data);
    self->on_rx_done(edata);
    return false; // no need to yield from ISR
}

bool IRAM_ATTR RCinput::on_rx_done(const rmt_rx_done_event_data_t *edata) {
    const rmt_symbol_word_t *syms = edata->received_symbols;

    // Each symbol has two halves: (duration0, level0) then (duration1, level1)
    // We only care about the HIGH portion (the pulse width).
    for (size_t i = 0; i < edata->num_symbols; ++i) {
        uint32_t highTicks = 0;

        // HIGH then LOW
        if (syms[i].level0 && !syms[i].level1) {
            highTicks = syms[i].duration0;
        }
        // LOW then HIGH (just in case)
        else if (syms[i].level1 && !syms[i].level0) {
            highTicks = syms[i].duration1;
        } else {
            continue;
        }

        // Classic RC PWM is roughly 1000–2000 µs. Allow some margin.
        if (highTicks >= 800 && highTicks <= 2200) {
            _pulseUs = static_cast<uint16_t>(highTicks);
            _updateCount = _updateCount + 1;
        }
    }

    // Re-arm receiver for next frame
    rmt_receive(_channel, _symbols,
                sizeof(_symbols), &_rx_cfg);

    return false;
}

int16_t RCinput::mapRange(uint16_t timeUs)
{
    int16_t value = static_cast<int16_t>(timeUs) - MID_US;
    value = (value * OUTPUT_RANGE) / (MAX_US - MID_US);

    // Clamp to [-1000, 1000]
    if (value > OUTPUT_RANGE) value = OUTPUT_RANGE;
    if (value < -OUTPUT_RANGE) value = -OUTPUT_RANGE;
    // apply deadband
    if ((value >= 0) && (value < DeadBand)) value = 0;
    if ((value <= 0) && (value > -DeadBand)) value = 0;

    return value;
}

RCinput::UserInput RCinput::value()
{
    if (_updateCount > 0 && _updateCount > _lastRead) {
        _lastRead = _updateCount;
        return mapRange(_pulseUs);
    } else {
        return std::nullopt;
    }
}

RCinput::UserInput RCinput::operator*()
{
    return this->value();
}