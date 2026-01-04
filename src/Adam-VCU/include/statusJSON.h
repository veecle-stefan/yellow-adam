#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "drivetrain.h"

// Encodes status JSON into outBuf. Returns number of bytes written (excluding '\0').
// Returns 0 if buffer too small.
size_t EncodeStatusJson(const DriveTrainStatus& st, char* outBuf, size_t outSize);