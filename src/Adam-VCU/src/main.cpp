#include <Arduino.h>
#include "drivetrain.h"

Axle axleF(UART_NUM_1, HWConfig::Pins::UART::Front::RX, HWConfig::Pins::UART::Front::TX);
Axle axleR(UART_NUM_2, HWConfig::Pins::UART::Rear::RX, HWConfig::Pins::UART::Rear::TX);
Lights lights;
RCinput* input = NULL;
DriveTrain* drive = NULL;

void setup() {
  Serial.begin(115200);
  Serial.println("Adam VCU starting");
  input = new RCinput();
  drive = new DriveTrain(input, axleF, axleR, lights);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000); // nothing to be done, all in background threads
}

