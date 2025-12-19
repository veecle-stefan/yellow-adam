#include <Arduino.h>
#include "motors.h"

void setup() {
  setupMotors();
}

void loop() {
  // do nothing, everything is done in FreeRTOS tasks that are set up in setupMotors()
  vTaskDelay(100 / portTICK_PERIOD_MS);
}