#include <Arduino.h>
#include "drivetrain.h"

Axle axleF(UART_NUM_1, PIN_UART1_RX, PIN_UART1_TX);
Axle axleR(UART_NUM_2, PIN_UART2_RX, PIN_UART2_TX);
Lights lights;
DriveTrain* drive = NULL;

void setup() {
  drive = new DriveTrain(axleF, axleR, lights);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000); // nothing to be done, all in background threads
}

