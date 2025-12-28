#pragma once
#include "axle.h"
#include "lights.h"

class DriveTrain
{
public:

    DriveTrain(Axle& axleF, Axle& axleR, Lights& lights);

protected:
    Axle& axleF;
    Axle& axleR;
    Lights& lights;
};