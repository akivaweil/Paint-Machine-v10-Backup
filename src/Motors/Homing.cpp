#include "motors/homing.h"
#include "motors/XYZ_Movements.h"
#include <Arduino.h>
#include <Bounce2.h>
#include <FastAccelStepper.h>
#include "utils/settings.h"
#include "system/machine_state.h"
#include "settings/debounce_settings.h"

//* ************************************************************************
//* **************************** HOMING ***********************************
//* ************************************************************************

// Static variables to replace class members
static FastAccelStepperEngine* engine = nullptr;
static FastAccelStepper* stepperX = nullptr;
static FastAccelStepper* stepperY_Left = nullptr;
static FastAccelStepper* stepperY_Right = nullptr;
static FastAccelStepper* stepperZ = nullptr;

static Bounce xHomeSwitch;
static Bounce yLeftHomeSwitch;
static Bounce yRightHomeSwitch;
static Bounce zHomeSwitch;

static bool isHoming = false;

void initializeHoming(FastAccelStepperEngine& eng,
                     FastAccelStepper* sX,
                     FastAccelStepper* sY_Left,
                     FastAccelStepper* sY_Right,
                     FastAccelStepper* sZ) {
    engine = &eng;
    stepperX = sX;
    stepperY_Left = sY_Left;
    stepperY_Right = sY_Right;
    stepperZ = sZ;
    
    // Initialize bounce objects
    xHomeSwitch.attach(X_HOME_SWITCH);
    xHomeSwitch.interval(HOMING_SWITCH_DEBOUNCE_MS);
    
    yLeftHomeSwitch.attach(Y_LEFT_HOME_SWITCH);
    yLeftHomeSwitch.interval(HOMING_SWITCH_DEBOUNCE_MS);
    
    yRightHomeSwitch.attach(Y_RIGHT_HOME_SWITCH);
    yRightHomeSwitch.interval(HOMING_SWITCH_DEBOUNCE_MS);
    
    zHomeSwitch.attach(Z_HOME_SWITCH);
    zHomeSwitch.interval(HOMING_SWITCH_DEBOUNCE_MS);
    
    Serial.println("Homing system initialized");
}

long inchesToStepsXYZ(float inches) {
    return (long)(inches * STEPS_PER_INCH_XYZ);
}

bool homeAllAxes() {
    if (!stepperX || !stepperY_Left || !stepperY_Right || !stepperZ) {
        Serial.println("ERROR: Homing not initialized - steppers not set");
        return false;
    }
    
    Serial.println("Starting Home All Axes sequence...");
    
    Serial.println("Homing: Allowing a brief moment for system to settle...");
    delay(250);
    
    Serial.println("Homing: Starting homing sequence proper...");

    // Log initial switch states
    Serial.println("Homing: Initial switch states (before movement, after Bounce2 update):");
    xHomeSwitch.update();
    Serial.printf("  X Home Switch (Pin %d): Raw State = %d, Bounce2 State = %d\n", X_HOME_SWITCH, digitalRead(X_HOME_SWITCH), xHomeSwitch.read());
    yLeftHomeSwitch.update();
    Serial.printf("  Y Left Home Switch (Pin %d): Raw State = %d, Bounce2 State = %d\n", Y_LEFT_HOME_SWITCH, digitalRead(Y_LEFT_HOME_SWITCH), yLeftHomeSwitch.read());
    yRightHomeSwitch.update();
    Serial.printf("  Y Right Home Switch (Pin %d): Raw State = %d, Bounce2 State = %d\n", Y_RIGHT_HOME_SWITCH, digitalRead(Y_RIGHT_HOME_SWITCH), yRightHomeSwitch.read());
    zHomeSwitch.update();
    Serial.printf("  Z Home Switch (Pin %d): Raw State = %d, Bounce2 State = %d\n", Z_HOME_SWITCH, digitalRead(Z_HOME_SWITCH), zHomeSwitch.read());

    // Set speeds and accelerations for homing movement
    stepperX->setSpeedInHz(HOMING_SPEED_X);
    stepperX->setAcceleration(HOMING_ACCEL_X);
    stepperY_Left->setSpeedInHz(HOMING_SPEED_Y);
    stepperY_Left->setAcceleration(HOMING_ACCEL_Y);
    stepperY_Right->setSpeedInHz(HOMING_SPEED_Y);
    stepperY_Right->setAcceleration(HOMING_ACCEL_Y);
    stepperZ->setSpeedInHz(HOMING_SPEED_Z);
    stepperZ->setAcceleration(HOMING_ACCEL_Z);
    
    // Set rotation motor speeds (if it exists)
    if (rotationStepper) {
        rotationStepper->setSpeedInHz(DEFAULT_ROT_SPEED / 2);
        rotationStepper->setAcceleration(DEFAULT_ROT_ACCEL / 2);
    }
    
    // Track homing status for each motor
    bool xHomed = false;
    bool yLeftHomed = false;
    bool yRightHomed = false;
    bool zHomed = false;
    bool rotationActuallyHomed = false;

    // Perform rotation homing FIRST if stepper exists
    if (rotationStepper) {
        Serial.println("Starting rotation homing to 0 degrees (shortest path)...");
        rotateToAngle(0);
        rotationStepper->setCurrentPosition(0);
        Serial.println("Rotation axis homed and set to 0 degrees.");
        rotationActuallyHomed = true;
    }
    
    // Start X, Y, Z motors moving toward home switches
    Serial.println("Moving X, Y, Z axes toward home switches...");

    xHomeSwitch.update();
    if (xHomeSwitch.read() != HIGH) {
        Serial.println("  X not at switch, starting X homing movement.");
        pinMode(X_DIR_PIN, OUTPUT);
        Serial.printf("  DEBUG: X_DIR_PIN (%d) state before runBackward: %d\n", X_DIR_PIN, digitalRead(X_DIR_PIN));
        stepperX->runBackward();
        Serial.printf("  DEBUG: X_DIR_PIN (%d) state AFTER runBackward: %d\n", X_DIR_PIN, digitalRead(X_DIR_PIN));
    } else {
        Serial.println("  X already at switch, marking as homed.");
        if (stepperX->isRunning()) stepperX->forceStop();
        stepperX->setCurrentPosition(0);
        xHomed = true;
    }

    yLeftHomeSwitch.update();
    if (yLeftHomeSwitch.read() != HIGH) {
        Serial.println("  Y-Left not at switch, starting Y-Left homing movement.");
        stepperY_Left->runBackward();
    } else {
        Serial.println("  Y-Left already at switch, marking as homed.");
        if (stepperY_Left->isRunning()) stepperY_Left->forceStop();
        stepperY_Left->setCurrentPosition(0);
        yLeftHomed = true;
    }

    yRightHomeSwitch.update();
    if (yRightHomeSwitch.read() != HIGH) {
        Serial.println("  Y-Right not at switch, starting Y-Right homing movement.");
        stepperY_Right->runBackward();
    } else {
        Serial.println("  Y-Right already at switch, marking as homed.");
        if (stepperY_Right->isRunning()) stepperY_Right->forceStop();
        stepperY_Right->setCurrentPosition(0);
        yRightHomed = true;
    }

    zHomeSwitch.update();
    if (zHomeSwitch.read() != HIGH) {
        Serial.println("  Z not at switch, starting Z homing movement.");
        stepperZ->runForward();
    } else {
        Serial.println("  Z already at switch, marking as homed.");
        if (stepperZ->isRunning()) stepperZ->forceStop();
        stepperZ->setCurrentPosition(0);
        zHomed = true;
    }

    bool rotationHomed = (rotationStepper == NULL) || rotationActuallyHomed;
    
    unsigned long startTime = millis();
    
    // Monitor all switches simultaneously using Bounce2
    while (!xHomed || !yLeftHomed || !yRightHomed || !zHomed) {
        // Check timeout
        if (millis() - startTime > HOMING_TIMEOUT_MS) {
            Serial.println("ERROR: Homing timeout!");
            if (!xHomed && stepperX->isRunning()) stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
            if (!yLeftHomed && stepperY_Left->isRunning()) stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
            if (!yRightHomed && stepperY_Right->isRunning()) stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
            if (!zHomed && stepperZ->isRunning()) stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
            if (rotationStepper && rotationStepper->isRunning()) {
                rotationStepper->forceStopAndNewPosition(rotationStepper->getCurrentPosition());
            }
            return false;
        }
        
        // Process X switch with immediate response
        if (!xHomed) {
            xHomeSwitch.update();
            if (xHomeSwitch.read() == HIGH) {
                if (stepperX->isRunning()) {
                    stepperX->forceStopAndNewPosition(0);
                    Serial.println("X Home switch triggered - MOTOR STOPPED IMMEDIATELY");
                } else {
                    stepperX->setCurrentPosition(0);
                    Serial.println("X Home switch triggered - position set to 0");
                }
                xHomed = true;
            }
        }
        
        // Process Y Left switch with immediate response
        if (!yLeftHomed) {
            yLeftHomeSwitch.update();
            if (yLeftHomeSwitch.read() == HIGH) {
                if (stepperY_Left->isRunning()) {
                    stepperY_Left->forceStopAndNewPosition(0);
                    Serial.println("Y Left Home switch triggered - MOTOR STOPPED IMMEDIATELY");
                } else {
                    stepperY_Left->setCurrentPosition(0);
                    Serial.println("Y Left Home switch triggered - position set to 0");
                }
                yLeftHomed = true;
            }
        }
        
        // Process Y Right switch with immediate response
        if (!yRightHomed) {
            yRightHomeSwitch.update();
            if (yRightHomeSwitch.read() == HIGH) {
                if (stepperY_Right->isRunning()) {
                    stepperY_Right->forceStopAndNewPosition(0);
                    Serial.println("Y Right Home switch triggered - MOTOR STOPPED IMMEDIATELY");
                } else {
                    stepperY_Right->setCurrentPosition(0);
                    Serial.println("Y Right Home switch triggered - position set to 0");
                }
                yRightHomed = true;
            }
        }
        
        // Process Z switch with immediate response
        if (!zHomed) {
            zHomeSwitch.update();
            if (zHomeSwitch.read() == HIGH) {
                if (stepperZ->isRunning()) {
                    stepperZ->forceStopAndNewPosition(0);
                    Serial.println("Z Home switch triggered - MOTOR STOPPED IMMEDIATELY");
                } else {
                    stepperZ->setCurrentPosition(0);
                    Serial.println("Z Home switch triggered - position set to 0");
                }
                zHomed = true;
            }
        }
        
        delay(1); // Small delay to prevent overwhelming the system
    }
    
    Serial.println("All axes homed successfully!");
    
    // Move to safe position after homing
    Serial.println("Moving to safe position after homing...");
    long safeSteps = inchesToStepsXYZ(1.0);
    moveToXYZ(safeSteps, DEFAULT_X_SPEED, safeSteps, DEFAULT_Y_SPEED, -safeSteps, DEFAULT_Z_SPEED); // Move 1 inch away from home switches
    
    Serial.println("Homing sequence completed successfully!");
    return true;
} 