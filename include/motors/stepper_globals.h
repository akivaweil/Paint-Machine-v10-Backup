#ifndef STEPPER_GLOBALS_H
#define STEPPER_GLOBALS_H

// Forward declare the class to avoid including the full header here
class FastAccelStepperEngine;
class FastAccelStepper;

// Declare global stepper objects defined in Setup.cpp
// These can be included by any file needing direct access to the steppers.

extern FastAccelStepperEngine engine;
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;

#endif // STEPPER_GLOBALS_H 