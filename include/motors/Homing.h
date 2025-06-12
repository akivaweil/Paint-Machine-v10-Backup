#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "utils/settings.h"
#include "system/machine_state.h"
#include "motors/Rotation_Motor.h"
#include "settings/debounce_settings.h"

//* ************************************************************************
//* **************************** HOMING ***********************************
//* ************************************************************************

// Initialize homing system
void initializeHoming(FastAccelStepperEngine& engine,
                     FastAccelStepper* stepperX,
                     FastAccelStepper* stepperY_Left,
                     FastAccelStepper* stepperY_Right,
                     FastAccelStepper* stepperZ);

// Main homing function
bool homeAllAxes();

// Utility functions
long inchesToStepsXYZ(float inches);

#endif // HOMING_H 