#include "drivetrain.h"
#include "pindefs.h"

#include "driver/rmt.h"
#include "driver/gpio.h"

// ---- Static storage for RMT & pulse data ----

DriveTrain* DriveTrain::instance = nullptr;
volatile uint16_t DriveTrain::s_pulseWidthUs[3] = {0, 0, 0};
RingbufHandle_t DriveTrain::s_rmtRingBuf[3] = { nullptr, nullptr, nullptr };


// Helper: configure one RMT RX channel
void DriveTrain::initRmtRxChannel(uint8_t chIndex, gpio_num_t gpio)
{
    rmt_channel_t channel = kRmtChannels[chIndex];

    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_RX;
    config.channel  = channel;
    config.gpio_num = gpio;
    config.clk_div  = 80; // 80 MHz APB / 80 = 1 MHz -> 1 tick = 1 µs
    config.mem_block_num = 1;

    config.rx_config.filter_en           = true;
    config.rx_config.filter_ticks_thresh = 100;   // ignore pulses < 100 µs
    config.rx_config.idle_threshold      = 2500;  // idle if low for > 2500 µs

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(channel, 1000, 0));  // small RX buffer

    // Get ring buffer handle
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(channel, &s_rmtRingBuf[chIndex]));
    // Start receiving
    ESP_ERROR_CHECK(rmt_rx_start(channel, true));
}

// Helper: drain RMT ring buffer for one channel and update latest pulse width
void DriveTrain::updatePulseFromRmt(uint8_t chIndex)
{
    RingbufHandle_t rb = s_rmtRingBuf[chIndex];
    if (!rb) return;

    size_t length = 0;
    // Non-blocking receive: timeout = 0
    rmt_item32_t* items = (rmt_item32_t*) xRingbufferReceive(rb, &length, 0);

    while (items) {
        int numItems = length / sizeof(rmt_item32_t);

        for (int i = 0; i < numItems; ++i) {
            const rmt_item32_t& it = items[i];

            // RMT item has two levels: level0 for duration0, then level1 for duration1
            // We care about whichever segment is HIGH and plausible as RC pulse.
            uint32_t pulseTicks = 0;

            if (it.level0 == 1) {
                pulseTicks = it.duration0; // high time in ticks (1 tick = 1 µs)
            } else if (it.level1 == 1) {
                pulseTicks = it.duration1;
            }

            if (pulseTicks > 0) {
                uint32_t pw = pulseTicks; // already in µs (clk_div = 80)

                if (pw >= DriveTrain::PpmMinUs && pw <= DriveTrain::PpmMaxUs) {
                    DriveTrain::s_pulseWidthUs[chIndex] = static_cast<uint16_t>(pw);
                }
            }
        }

        // Return buffer to RMT driver
        vRingbufferReturnItem(rb, (void*)items);

        // Try to grab more items, still non-blocking
        items = (rmt_item32_t*) xRingbufferReceive(rb, &length, 0);
    }
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
    // First, drain RMT ring buffers to update s_pulseWidthUs[]
    updatePulseFromRmt(0); // CH1
    updatePulseFromRmt(1); // CH2
    updatePulseFromRmt(2); // CH3

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