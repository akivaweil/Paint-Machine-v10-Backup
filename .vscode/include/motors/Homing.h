#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "utils/settings.h"
#include "system/machine_state.h"
#include "motors/Rotation_Motor.h" // Include Rotation_Motor for rotationStepper access

class Homing {
public:
    Homing(FastAccelStepperEngine& engine,
           FastAccelStepper* stepperX,
           FastAccelStepper* stepperY_Left,
           FastAccelStepper* stepperY_Right,
           FastAccelStepper* stepperZ);
    
    bool homeAllAxes();

private:
    FastAccelStepperEngine& _engine; // Reference to the engine
    FastAccelStepper* _stepperX;
    FastAccelStepper* _stepperY_Left;
    FastAccelStepper* _stepperY_Right;
    FastAccelStepper* _stepperZ;
    // Note: rotationStepper is accessed via the extern declaration from Rotation_Motor.h

    Bounce _xHomeSwitch;
    Bounce _yLeftHomeSwitch;
    Bounce _yRightHomeSwitch;
    Bounce _zHomeSwitch;

    const unsigned long HOMING_SWITCH_DEBOUNCE_MS = 3; // milliseconds
    bool _isHoming = false; // Internal homing state flag

    long inchesToStepsXYZ(float inches); // Keep utility function private or move elsewhere if shared
};

#endif // HOMING_H 