#pragma once
#include <SimpleFOC.h>

// Expose motor & commander if you need them elsewhere
extern HallSensor sensorL;
extern HallSensor sensorR;

extern BLDCMotor motorL;
extern BLDCMotor motorR;

extern Commander command;

// Call from Arduino setup()
void setupMotors();
