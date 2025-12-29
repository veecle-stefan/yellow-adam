#include "rcinput.h"
#include "hwconfig.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
// ---- Static storage for RMT & pulse data ----

RCinput* RCinput::instance = nullptr;
volatile uint16_t RCinput::s_pulseWidthUs[3] = {0, 0, 0};

volatile uint64_t RCinput::s_lastUpdateUs[3] = {0, 0, 0};
portMUX_TYPE RCinput::s_rcMux = portMUX_INITIALIZER_UNLOCKED;

// New-driver RMT objects (file-local, one per RC channel)
static rmt_channel_handle_t s_rmtRxChannel[3] = { nullptr, nullptr, nullptr };

// 1 MHz resolution → 1 tick = 1 µs (like your old clk_div = 80)
static constexpr uint32_t kRmtResolutionHz  = 1'000'000;
static constexpr size_t   kRmtMaxSymbols    = 64;          // plenty for one PWM frame

// Storage for received RMT symbols for each channel
static rmt_symbol_word_t s_rxSymbols[3][kRmtMaxSymbols];

// Simple channel index mapping for the callback
static uint8_t s_rmtChannelIndex[3] = { 0, 1, 2 };

// Common RX config: ignore pulses < 100 µs, stop after > 2500 µs per segment
static const rmt_receive_config_t s_rmtReceiveConfig = {
    .signal_range_min_ns = 100 * 1000,   // 100 µs
    .signal_range_max_ns = 2500 * 1000, // 2.5 ms (longer segment ends the frame)
};

// Helper: configure one RMT RX channel
// Helper: configure one RMT RX channel (new RX driver)
void RCinput::initRmtRxChannel(uint8_t chIndex, gpio_num_t gpio)
{
    if (chIndex >= 3) {
        return;
    }

    // Configure RX channel
    rmt_rx_channel_config_t cfg = {};
    cfg.clk_src          = RMT_CLK_SRC_DEFAULT;
    cfg.gpio_num         = gpio;
    cfg.resolution_hz    = kRmtResolutionHz;   // 1 tick = 1 µs
    cfg.mem_block_symbols = kRmtMaxSymbols;    // internal RMT buffer depth
    cfg.intr_priority    = 0;
    cfg.flags.invert_in  = 0;
    cfg.flags.with_dma   = 0;
    cfg.flags.io_loop_back = 0;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&cfg, &s_rmtRxChannel[chIndex]));

    // Enable channel
    ESP_ERROR_CHECK(rmt_enable(s_rmtRxChannel[chIndex]));

    // Register receive-done callback
    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = &RCinput::rmtRxDoneCallback;
    ESP_ERROR_CHECK(
        rmt_rx_register_event_callbacks(s_rmtRxChannel[chIndex],
                                        &cbs,
                                        &s_rmtChannelIndex[chIndex]) // user_data
    );

    // Start first receive transaction
    ESP_ERROR_CHECK(
        rmt_receive(s_rmtRxChannel[chIndex],
                    s_rxSymbols[chIndex],
                    sizeof(s_rxSymbols[chIndex]),
                    &s_rmtReceiveConfig)
    );
}

// Called by the new RMT driver when a receive transaction completes.
// Runs in ISR context, so keep it short and non-blocking.
bool IRAM_ATTR RCinput::rmtRxDoneCallback(rmt_channel_handle_t channel,
                                           const rmt_rx_done_event_data_t *edata,
                                           void *user_data)
{
    uint8_t chIndex = user_data ? *static_cast<uint8_t*>(user_data) : 0;
    if (chIndex >= 3 || !edata || !edata->received_symbols || edata->num_symbols == 0) {
        return false;
    }

    const rmt_symbol_word_t *items = edata->received_symbols;
    size_t num = edata->num_symbols;

    uint32_t pulseTicks = 0;

    for (size_t i = 0; i < num; ++i) {
        const rmt_symbol_word_t &sym = items[i];

        uint32_t candidate = 0;
        if (sym.level0) {
            candidate = sym.duration0;
        } else if (sym.level1) {
            candidate = sym.duration1;
        } else {
            continue;
        }

        if (candidate >= PpmMinUs && candidate <= PpmMaxUs) {
            pulseTicks = candidate;
            break;
        }
    }

    if (pulseTicks != 0) {
        portENTER_CRITICAL_ISR(&s_rcMux);
        s_pulseWidthUs[chIndex] = static_cast<uint16_t>(pulseTicks);
        s_lastUpdateUs[chIndex] = esp_timer_get_time(); // optional
        portEXIT_CRITICAL_ISR(&s_rcMux);
    }

    // Re-arm RX
    (void) rmt_receive(channel,
                       s_rxSymbols[chIndex],
                       sizeof(s_rxSymbols[chIndex]),
                       &s_rmtReceiveConfig);

    return false;
}

// Map pulse width [PpmMinUs..PpmMaxUs] to [-1000..+1000]
int16_t RCinput::mapPulseToCommand(uint16_t pulseWidthUs)
{
    if (pulseWidthUs == 0) {
        // Not yet seen anything: treat as neutral
        return 0;
    }

    // Clamp to valid range
    if (pulseWidthUs < PpmMinUs) pulseWidthUs = PpmMinUs;
    if (pulseWidthUs > PpmMaxUs) pulseWidthUs = PpmMaxUs;

    const int32_t center    = PpmCenterUs;
    const int32_t halfRange = (PpmMaxUs - PpmMinUs) / 2; // ~500

    int32_t delta = static_cast<int32_t>(pulseWidthUs) - center;

    // Scale: center±halfRange -> 0±1000
    int32_t cmd = (delta * 1000) / halfRange;

    // Saturate to [-1000, 1000]
    if (cmd >  1000) cmd =  1000;
    if (cmd < -1000) cmd = -1000;

    return static_cast<int16_t>(cmd);
}


// ----- Constructor -----

RCinput::RCinput()
{
    instance = this;

    // Configure PPM input pins as inputs
    pinMode(HWConfig::Pins::PPM::Channel1, INPUT_PULLUP);
    pinMode(HWConfig::Pins::PPM::Channel2, INPUT_PULLUP);
    pinMode(HWConfig::Pins::PPM::Channel3, INPUT_PULLUP);

    // Initialize three RMT RX channels
    initRmtRxChannel(0, static_cast<gpio_num_t>(HWConfig::Pins::PPM::Channel1));
    initRmtRxChannel(1, static_cast<gpio_num_t>(HWConfig::Pins::PPM::Channel2));
    initRmtRxChannel(2, static_cast<gpio_num_t>(HWConfig::Pins::PPM::Channel3));
}



// ----- ReadChannels: non-blocking RMT-based read -----
void RCinput::ReadChannels(int16_t& throttle, int16_t& steering, int16_t& aux)
{
    uint16_t ch[3];
    uint64_t ts[3];

    // Take an atomic-ish snapshot of all channels
    portENTER_CRITICAL(&s_rcMux);
    ch[0] = s_pulseWidthUs[0];
    ch[1] = s_pulseWidthUs[1];
    ch[2] = s_pulseWidthUs[2];
    ts[0] = s_lastUpdateUs[0];
    ts[1] = s_lastUpdateUs[1];
    ts[2] = s_lastUpdateUs[2];
    portEXIT_CRITICAL(&s_rcMux);

    // Optional: check for stale input (signal lost)
    const uint64_t nowUs = esp_timer_get_time();
    constexpr uint64_t kTimeoutUs = 200000; // 200 ms, tweak as needed

    bool valid = (nowUs - ts[0] < kTimeoutUs) &&
                 (nowUs - ts[1] < kTimeoutUs) &&
                 (nowUs - ts[2] < kTimeoutUs);

    if (!valid) {
        throttle = 0;
        steering = 0;
        aux      = 0;
        return;
    }

    // Map to [-1000 .. +1000]
    throttle = mapPulseToCommand(ch[0]);
    steering = mapPulseToCommand(ch[1]);
    aux      = mapPulseToCommand(ch[2]);
}