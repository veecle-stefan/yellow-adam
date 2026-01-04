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
}