#pragma once
#include <Arduino.h>

namespace SWConfig {

    namespace Tasks {
        static constexpr uint32_t MinStakSize = 2048;
        static constexpr UBaseType_t PrioLow = 1;
        static constexpr UBaseType_t PrioMed = 10;
        static constexpr UBaseType_t PrioHigh = 15;
    }

    namespace CoreAffinity {
        static constexpr BaseType_t CoreApp = 1;
        static constexpr BaseType_t CoreComms = 0;
    }

    namespace InputFiltering {

        // Deadband for user input (e.g., joystick) to avoid noise around center.
        // Default: 50 (input units, typically [-1000..1000])
        static constexpr int16_t DeadBand = 50;
    }

    namespace UserBehaviour {
        // Max time between press/release to count as a "tap".
        // Default: 300 ms
        static constexpr uint32_t TapTimeMs = 300;

        // Max time between two taps to count as "double tap".
        // Default: 1000 ms
        static constexpr uint32_t DoubleTapTimeMs = 1000;

        // Timeout for external (WiFi) control before falling back / disabling it.
        // Default: 1000 ms (1s)
        static constexpr uint32_t ExternalTimeoutMs = 1000;

        // Hard shutdown if user provides no input for this long.
        // Default: 600000 ms (10 minutes)
        static constexpr uint32_t MaxUserIdleBeforeShutdownMs = 600000;

        // Warning starts this long before shutdown (e.g., beep pattern).
        // Default: 30s before poweroff
        static constexpr uint32_t MaxUserIdleWarnMs = 570000;
    }
}