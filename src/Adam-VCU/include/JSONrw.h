#pragma once
#include <stdint.h>
#include "drivetrain.h"

class JSONInteraction
{
    using DriveTrainStatus = DriveTrain::DriveTrainStatus;
    

    public:
        // Encodes status JSON into outBuf. Returns number of bytes written (excluding '\0').
        // Returns 0 if buffer too small.
        static size_t EncodeStatusJson(const DriveTrainStatus& st, char* buffer, size_t bufferSize);
        static bool DispatchCommand(const String& msg, DriveTrain* drive);
        

        
};