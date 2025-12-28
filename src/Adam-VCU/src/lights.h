#pragma once
#include <FastLED.h>
#include "pindefs.h"


class Lights {
public:
    static constexpr uint16_t LightsUpdateFreq = 100; // ms
    static constexpr CRGB::HTMLColorCode ColOff = CRGB::Black;
    static constexpr CRGB::HTMLColorCode ColIndicator = CRGB::Red;
    static constexpr CRGB::HTMLColorCode ColDRL = CRGB::Gray10;
    static constexpr CRGB::HTMLColorCode ColLoBeam = CRGB::Gray50;
    static constexpr CRGB::HTMLColorCode ColHiBeam = CRGB::Gray75;
    static constexpr CRGB::HTMLColorCode ColTail = CRGB::Red4;
    static constexpr CRGB::HTMLColorCode ColBrake = CRGB::Red2;

    enum HeadLightState {
        Off = 0,
        DRL = 1,
        Dipped = 2,
        High = 3
    };

    enum IndicatorPosition : uint8_t {
        FR = 0,
        FL = 1,
        RR = 2,
        RL = 3
    };

    Lights();
    void SetHeadlight(HeadLightState newState);
    void SetTailLight(bool onOff);
    void SetBrakeLight(bool onOff);
    void SetReverseLight(bool onOff);
    void SetIndicators(bool FR, bool FL, bool RR, bool RL);
    void SetIndicator(IndicatorPosition pos, bool newState);

protected:
    CRGB HeadLights[NUMLED_HEAD];
    CRGB TailLights[NUMLED_TAIL];
    CRGB Indicators[NUMLED_IND];

    HeadLightState stHeadlights;
    bool stIndicators[4] = {false, false, false, false};
    bool stTaillight = false;
    bool stBrakeLight = false;
    bool stReverseLight = false;

    bool internalBlinkPhase = false;

    void CyclicUpdateTask();
    void UpdateLights();
};