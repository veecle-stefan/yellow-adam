#include "drivetrain.h"
#include "pindefs.h"
#include "driver/rmt_rx.h"
#include "driver/gpio.h"
// ---- Static storage for RMT & pulse data ----

DriveTrain* DriveTrain::instance = nullptr;
volatile uint16_t DriveTrain::s_pulseWidthUs[3] = {0, 0, 0};

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
void DriveTrain::initRmtRxChannel(uint8_t chIndex, gpio_num_t gpio)
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
    cbs.on_recv_done = &DriveTrain::rmtRxDoneCallback;
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
bool IRAM_ATTR DriveTrain::rmtRxDoneCallback(rmt_channel_handle_t channel,
                                             const rmt_rx_done_event_data_t *edata,
                                             void *user_data)
{
    // Figure out which logical channel this belongs to
    uint8_t chIndex = 0;
    if (user_data) {
        chIndex = *static_cast<uint8_t*>(user_data);
    }
    if (chIndex >= 3 || !edata || !edata->received_symbols || edata->num_symbols == 0) {
        return false;
    }

    const rmt_symbol_word_t *items = edata->received_symbols;
    size_t num = edata->num_symbols;

    // Find the high pulse that looks like a valid RC servo pulse
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
        // resolution is 1 µs/tick → directly store in microseconds
        s_pulseWidthUs[chIndex] = static_cast<uint16_t>(pulseTicks);
    }

    // Re-arm RX for the next pulse using the same buffer and config.
    // rmt_receive() is non-blocking and used this way in Espressif’s own examples.
    (void) rmt_receive(channel,
                       s_rxSymbols[chIndex],
                       sizeof(s_rxSymbols[chIndex]),
                       &s_rmtReceiveConfig);

    // we didn’t wake a higher-priority task
    return false;
}

// Map pulse width [PpmMinUs..PpmMaxUs] to [-1000..+1000]
int16_t DriveTrain::mapPulseToCommand(uint16_t pulseWidthUs)
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

DriveTrain::DriveTrain(Axle& axleF, Axle& axleR, Lights& lights)
    : axleF(axleF)
    , axleR(axleR)
    , lights(lights)
{
    instance = this;

    // Configure PPM input pins as inputs
    pinMode(PIN_PPM_CH1, INPUT_PULLUP);
    pinMode(PIN_PPM_CH2, INPUT_PULLUP);
    pinMode(PIN_PPM_CH3, INPUT_PULLUP);

    // Initialize three RMT RX channels
    initRmtRxChannel(0, static_cast<gpio_num_t>(PIN_PPM_CH1));
    initRmtRxChannel(1, static_cast<gpio_num_t>(PIN_PPM_CH2));
    initRmtRxChannel(2, static_cast<gpio_num_t>(PIN_PPM_CH3));

    // Start background control task (100 ms period)
    xTaskCreate(
        [](void* pvParameters) {
            static_cast<DriveTrain*>(pvParameters)->ControlTask();
            vTaskDelete(nullptr);
        },
        "DriveTrainCtrl",
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
        ReadChannels(throttle, steering, aux);

        // 2. Run torque vectoring
        int16_t fl = 0, fr = 0, rl = 0, rr = 0;
        TorqueVectoring(throttle, steering, fl, fr, rl, rr);

        // 3. Update all 4 motors via the two axles
        axleF.Send(fl, fr); // front left/right
        axleR.Send(rl, rr); // rear left/right

        // (Lights could be updated based on throttle/steering here)

        // 4. Wait until next 100ms cycle
        vTaskDelayUntil(&lastWakeTime, period);
    }
}


// ----- ReadChannels: non-blocking RMT-based read -----

void DriveTrain::ReadChannels(int16_t& throttle, int16_t& steering, int16_t& aux)
{
    // Copy last measured pulse widths (volatile -> local)
    uint16_t ch1 = s_pulseWidthUs[0];
    uint16_t ch2 = s_pulseWidthUs[1];
    uint16_t ch3 = s_pulseWidthUs[2];

    // Map to [-1000 .. +1000]
    throttle = mapPulseToCommand(ch1);
    steering = mapPulseToCommand(ch2);
    aux      = mapPulseToCommand(ch3); // currently unused, but ready for future
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