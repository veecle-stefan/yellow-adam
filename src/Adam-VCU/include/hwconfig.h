#pragma once
#include <Arduino.h>

namespace HWConfig {

    namespace Pins
    {
        namespace LEDs {
            static constexpr gpio_num_t Headlights = GPIO_NUM_19;
            static constexpr gpio_num_t Taillights = GPIO_NUM_13;
            static constexpr gpio_num_t Indicators = GPIO_NUM_23;
        }

        namespace PPM {
            static constexpr gpio_num_t Channel1 = GPIO_NUM_34;
            static constexpr gpio_num_t Channel2 = GPIO_NUM_35;
            static constexpr gpio_num_t Channel3 = GPIO_NUM_39;
        }

        namespace UART {
            namespace Front {
                static constexpr gpio_num_t RX = GPIO_NUM_16;
                static constexpr gpio_num_t TX = GPIO_NUM_17;
            }
            namespace Rear {
                static constexpr gpio_num_t RX = GPIO_NUM_33;
                static constexpr gpio_num_t TX = GPIO_NUM_32;
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