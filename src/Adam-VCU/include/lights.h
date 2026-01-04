#pragma once
#include <FastLED.h>
#include "hwconfig.h"


class Lights {
public:
    static constexpr uint16_t LightsUpdateFreq = 100; // ms
    static constexpr uint32_t BlinkOn = 600; // ms
    static constexpr uint32_t BlinkOff = 400; // ms
    static constexpr CRGB ColOff = CRGB::Black;
    static constexpr CRGB ColIndicator = 0x773300;
    static constexpr CRGB ColDRL = 0x101010;
    static constexpr CRGB ColLoBeam = 0x303030;
    static constexpr CRGB ColHiBeam = 0x888888;
    static constexpr CRGB ColTail = 0x110000;
    static constexpr CRGB ColBrake = 0xaa0000;
    static constexpr CRGB ColReverse = 0x119999;
    static constexpr CRGB ColOTA = 0x550055;

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
    void SetOTA(bool onOff);
    void SetOTAprogress(uint8_t percent);

protected:
    CRGB HeadLights[HWConfig::Sizes::LEDs::NumHeadlights];
    CRGB TailLights[HWConfig::Sizes::LEDs::NumTaillights];
    CRGB Indicators[HWConfig::Sizes::LEDs::NumIndicators];

    HeadLightState stHeadlights;
    bool stIndicators[4] = {false, false, false, false};
    bool stTaillight = false;
    bool stBrakeLight = false;
    bool stReverseLight = false;
    bool stOTA = false;
    uint8_t OTAprogress = 0;

    bool internalBlinkPhase = false;

    void CyclicUpdateTask();
    void UpdateLights();
};