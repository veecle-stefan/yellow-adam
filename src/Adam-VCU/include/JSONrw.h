#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "drivetrain.h"

class JSONInteraction
{
    

    public:
        char* buffer;
        explicit JSONInteraction(size_t buffer);

        // Encodes status JSON into outBuf. Returns number of bytes written (excluding '\0').
        // Returns 0 if buffer too small.
        size_t EncodeStatusJson(const DriveTrainStatus& st);
        bool DispatchCommand(const String& msg, DriveTrain* drive);

    protected:
        size_t bufferSize;

        
};