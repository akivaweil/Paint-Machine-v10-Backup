#include "functionality/ManualControl.h"
#include <Arduino.h>
#include "motors/XYZ_Movements.h"
#include "motors/ServoMotor.h"
#include "system/StateMachine.h"
#include "states/State.h" // Required for state->getName()
#include <FastAccelStepper.h> // Required for stepper->getCurrentPosition()
#include "motors/Rotation_Motor.h" // ADDED for tray rotation
#include "settings/motion.h" // ADDED for STEPS_PER_DEGREE
#include <limits.h> // For LONG_MIN, INT_MIN

// External instances from the main project
extern ServoMotor myServo;
extern FastAccelStepper* stepperX;
extern FastAccelStepper* stepperY_Left; // Assuming Y_Left is representative for Y position
extern FastAccelStepper* stepperZ;
extern FastAccelStepper* rotationStepper; // ADDED: extern declaration for rotation stepper
extern StateMachine* stateMachine;
// extern const float STEPS_PER_DEGREE; // This is defined in motion.h, included above

// Speed settings for manual movements (can be adjusted or made configurable later)
const unsigned int MANUAL_CONTROL_MOVE_X_SPEED = 10000;
const unsigned int MANUAL_CONTROL_MOVE_Y_SPEED = 10000;
const unsigned int MANUAL_CONTROL_MOVE_Z_SPEED = 4000;

// Define sentinel values directly if not using <limits.h> or for clarity
const long Z_NOT_PROVIDED = LONG_MIN; 
const int ANGLE_NOT_PROVIDED = INT_MIN;

//* ************************************************************************
//* *********************** MANUAL CONTROL FUNCTIONS **********************
//* ************************************************************************
// This file implements functions for direct manual control of the machine,
// such as moving to specific coordinates or rotating the tray, callable when
// the machine is in an appropriate state (e.g., Idle).

bool canPerformManualMove() {
    if (!stateMachine || !stateMachine->getCurrentState()) {
        Serial.println("Error: StateMachine or current state is null in canPerformManualMove.");
        return false;
    }
    const char* current_state_name = stateMachine->getCurrentState()->getName();
    if (strcmp(current_state_name, "IDLE") == 0) {
        return true;
    }
    // Serial.print("Manual move not allowed in state: "); // Optional: for debugging
    // Serial.println(current_state_name);
    return false;
}

void handleManualMoveToPosition(long targetX_steps, long targetY_steps, long targetZ_steps_param, int targetAngle_deg_param) {
    if (!canPerformManualMove()) {
        Serial.println("Error: Cannot execute manual move. Machine not in a permissible state (e.g., IDLE).");
        return;
    }

    long actual_targetZ_steps = targetZ_steps_param;
    int actual_targetAngle_deg = targetAngle_deg_param;
    bool z_was_provided = (targetZ_steps_param != Z_NOT_PROVIDED);
    bool angle_was_provided = (targetAngle_deg_param != ANGLE_NOT_PROVIDED);

    if (!z_was_provided) {
        if (stepperZ) {
            actual_targetZ_steps = stepperZ->getCurrentPosition();
            Serial.println("Z position not provided, using current Z: " + String(actual_targetZ_steps));
        } else {
            Serial.println("Error: Z position not provided and stepperZ is null. Cannot proceed.");
            return; // Or move to a default safe Z
        }
    }

    Serial.print("Manual Move Command: X_steps=");
    Serial.print(targetX_steps);
    Serial.print(", Y_steps=");
    Serial.print(targetY_steps);
    Serial.print(", Z_steps=");
    Serial.print(actual_targetZ_steps);
    if (z_was_provided) Serial.print(" (provided)"); else Serial.print(" (current)");
    
    if (angle_was_provided) {
        Serial.print(", Angle_deg=");
        Serial.print(actual_targetAngle_deg);
        Serial.println(" (provided)");

        //! Set Servo Angle if provided
        myServo.setAngle(actual_targetAngle_deg);
        Serial.print("Servo set to ");
        Serial.print(actual_targetAngle_deg);
        Serial.println(" degrees");
        delay(500); // Allow servo time to move, consistent with original ManualMoveState
    } else {
        Serial.println(", Angle_deg: not provided (maintaining current)");
        // Servo angle is not changed
    }

    //! Move XYZ Axis
    Serial.println("Moving XYZ axes (manual control)...");
    moveToXYZ(targetX_steps, MANUAL_CONTROL_MOVE_X_SPEED, 
              targetY_steps, MANUAL_CONTROL_MOVE_Y_SPEED, 
              actual_targetZ_steps, MANUAL_CONTROL_MOVE_Z_SPEED);
    Serial.println("Manual XYZ move complete.");

    // No need to store m_currentX_steps etc., as these are direct commands
    // The UI or calling function can fetch current positions if needed after the move.
}

void handleManualRotateClockwise90() {
    if (!canPerformManualMove()) {
        Serial.println("Error: Cannot execute manual rotate. Machine not in a permissible state (e.g., IDLE).");
        return;
    }
    if (!rotationStepper) {
        Serial.println("Error: Rotation stepper not initialized for manual rotation.");
        return;
    }

    Serial.println("Manual Tray Rotate Clockwise 90 Command Received");
    
    float currentAngle_deg = (float)rotationStepper->getCurrentPosition() / STEPS_PER_DEGREE;
    // Normalize current angle to be positive within 0-360 before adding
    currentAngle_deg = fmod(currentAngle_deg, 360.0f);
    if (currentAngle_deg < 0) {
        currentAngle_deg += 360.0f;
    }

    float newTargetAngle_deg = currentAngle_deg + 90.0f;
    // The rotateToAngle function handles shortest path and normalization to 0-360 target.
    // Example: if current is 270, newTarget will be 360. rotateToAngle(360) or rotateToAngle(0) should work.
    // If current is 350, newTarget will be 440. rotateToAngle(440) should be interpreted as rotateToAngle(80).

    Serial.printf("Current Tray Angle: %.2f deg, New Target: %.2f deg\n", currentAngle_deg, newTargetAngle_deg);
    rotateToAngle(newTargetAngle_deg);
    Serial.println("Manual tray rotation CW 90 complete.");
}

void handleManualRotateCounterClockwise90() {
    if (!canPerformManualMove()) {
        Serial.println("Error: Cannot execute manual rotate. Machine not in a permissible state (e.g., IDLE).");
        return;
    }
    if (!rotationStepper) {
        Serial.println("Error: Rotation stepper not initialized for manual rotation.");
        return;
    }

    Serial.println("Manual Tray Rotate Counter-Clockwise 90 Command Received");

    float currentAngle_deg = (float)rotationStepper->getCurrentPosition() / STEPS_PER_DEGREE;
    // Normalize current angle to be positive within 0-360 before subtracting
    currentAngle_deg = fmod(currentAngle_deg, 360.0f);
    if (currentAngle_deg < 0) {
        currentAngle_deg += 360.0f;
    }

    float newTargetAngle_deg = currentAngle_deg - 90.0f;
    // The rotateToAngle function handles shortest path and normalization to 0-360 target.
    // Example: if current is 0, newTarget will be -90. rotateToAngle(-90) should be interpreted as rotateToAngle(270).

    Serial.printf("Current Tray Angle: %.2f deg, New Target: %.2f deg\n", currentAngle_deg, newTargetAngle_deg);
    rotateToAngle(newTargetAngle_deg);
    Serial.println("Manual tray rotation CCW 90 complete.");
} 