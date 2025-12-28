#include "lights.h"

Lights::Lights()
{
    FastLED.addLeds<NEOPIXEL, PIN_LED_HEAD>(this->HeadLights, NUMLED_HEAD);
    FastLED.addLeds<NEOPIXEL, PIN_LED_TAIL>(this->TailLights, NUMLED_TAIL);
    FastLED.addLeds<WS2812B, PIN_LED_IND>(this->Indicators, NUMLED_IND);

    xTaskCreate(
        [](void* pvParameters) {
            static_cast<Lights*>(pvParameters)->CyclicUpdateTask();
            vTaskDelete(NULL);
        },
        "Lights",
        2048,
        this,
        1,
        NULL
    );
}

void Lights::SetHeadlight(HeadLightState newState)
{
    this->stHeadlights = newState;
}
    
void Lights::SetTailLight(bool onOff)
{
    this->stTaillight = onOff;
}
    
void Lights::SetBrakeLight(bool onOff)
{
    this->stBrakeLight = onOff;
}
    
void Lights::SetReverseLight(bool onOff)
{
    this->stReverseLight = onOff;
}
    
void Lights::SetIndicators(bool FR, bool FL, bool RR, bool RL)
{
    SetIndicator(IndicatorPosition::FR, FR);
    SetIndicator(IndicatorPosition::FL, FL);
    SetIndicator(IndicatorPosition::RR, RR);
    SetIndicator(IndicatorPosition::RL, RL);
}
    
void Lights::SetIndicator(IndicatorPosition pos, bool newState)
{
    this->stIndicators[pos] = newState;
    this->internalBlinkPhase = true;
}
    

void Lights::UpdateLights()
{
    // take all the current states and generate LED patterns

    // 1. indicators
    for(uint8_t i = 0; i < 4; i++) {
        Indicators[i] = this->stIndicators[i] && this->internalBlinkPhase ? ColIndicator : ColOff;
    }

    // 2. tail lights
    for(uint8_t t = 0; t < NUMLED_TAIL; t++) {
        TailLights[t] = this->stTaillight ? ColTail : ColOff;
    }
    if (this->stBrakeLight) {
        TailLights[0] = TailLights[NUMLED_TAIL-1] = ColBrake;
    }

    // 3. Head lights
    // 7 LEDs:
    // L L L L L L L
    for(uint8_t h = 0; h < NUMLED_HEAD; h++) {
        HeadLights[h] = ColOff; // reset default off state
    }
    switch (this->stHeadlights) {
        case DRL:
            HeadLights[0] = HeadLights[13] = ColDRL;
            HeadLights[1] = HeadLights[12] = ColDRL;
            HeadLights[2] = HeadLights[11] = ColDRL;
            HeadLights[4] = HeadLights[9] = ColDRL;
            HeadLights[5] = HeadLights[8] = ColDRL;
            HeadLights[6] = HeadLights[7] = ColDRL;
           break;
        case Dipped:
            HeadLights[0] = HeadLights[13] = ColLoBeam;
            HeadLights[1] = HeadLights[12] = ColLoBeam;
            HeadLights[2] = HeadLights[11] = ColLoBeam;
            HeadLights[3] = HeadLights[10] = ColLoBeam;
            HeadLights[4] = HeadLights[9] = ColDRL;
            HeadLights[5] = HeadLights[8] = ColDRL;
            HeadLights[6] = HeadLights[7] = ColDRL;
            break;
        case High:
            HeadLights[0] = HeadLights[13] = ColHiBeam;
            HeadLights[1] = HeadLights[12] = ColHiBeam;
            HeadLights[2] = HeadLights[11] = ColHiBeam;
            HeadLights[3] = HeadLights[10] = ColHiBeam;
            HeadLights[4] = HeadLights[9] = ColDRL;
            HeadLights[5] = HeadLights[8] = ColDRL;
            HeadLights[6] = HeadLights[7] = ColDRL;
            break;
    }

    
}

void Lights::CyclicUpdateTask()
{
    const TickType_t period = pdMS_TO_TICKS(LightsUpdateFreq);
    TickType_t lastWakeTime = xTaskGetTickCount();

    for(;;) {
        // update the lights
        UpdateLights();
        FastLED.show();

        // sleep until next cycle
        vTaskDelayUntil(&lastWakeTime, period);
    }
}