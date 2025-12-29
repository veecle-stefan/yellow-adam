#pragma once
#include <Arduino.h>

namespace HWConfig {

    namespace Pins
    {
        typedef uint8_t pin_t;
        namespace LEDs {
            static constexpr pin_t Headlights = 19;
            static constexpr pin_t Taillights = 13;
            static constexpr pin_t Indicators = 23;
        }

        namespace PPM {
            static constexpr pin_t Channel1 = 34;
            static constexpr pin_t Channel2 = 35;
            static constexpr pin_t Channel3 = 39;
        }

        namespace UART {
            namespace Front {
                static constexpr pin_t RX = 16;
                static constexpr pin_t TX = 17;
            }
            namespace Rear {
                static constexpr pin_t RX = 33;
                static constexpr pin_t TX = 32;
            }
        }
    }

    namespace Sizes {
        namespace LEDs {
            static constexpr uint8_t NumHeadlights = 14;
            static constexpr uint8_t NumTaillights = 10;
            static constexpr uint8_t NumIndicators = 4;
        }
    }
}