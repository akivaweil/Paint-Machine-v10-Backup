#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"       // For moveToXYZ
#include "../../include/utils/settings.h"         // For STEPS_PER_INCH, default speeds
#include "../../include/motors/Rotation_Motor.h"     // For rotateToAngle
#include "../../include/hardware/paintGun_Functions.h" // For paintGun_ON/OFF
#include "../../include/hardware/pressurePot_Functions.h" // For PressurePot_ON
#include <FastAccelStepper.h>
#include "../../include/persistence/PaintingSettings.h" // Include for accessing saved settings
#include "../../include/settings/painting.h"         // For painting-specific constants (SIDE1_Z_HEIGHT etc.)
#include "../../include/motors/ServoMotor.h"         // For ServoMotor class
#include "states/PaintingState.h" // Correct filename
#include "settings/pins.h"        // Keep this one
#include "../../include/web/Web_Dashboard_Commands.h" // For checkForHomeCommand
#include "../../include/system/StateMachine.h" // Include StateMachine header

// External references to stepper motors
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperZ;
extern ServoMotor myServo; // Declare external servo instance
extern PaintingSettings paintingSettings; // Make sure global instance is accessible
extern StateMachine* stateMachine; // Declare external state machine instance

//* ************************************************************************
//* *************************** SIDE 1 *************************************
//* ************************************************************************

// Returns true if painting completed, false if aborted due to home command
bool paintSide1Pattern() {
    Serial.println("Starting Side 1 Pattern Painting");

    //! Load Servo Angle
    int servoAngle = paintingSettings.getServoAngleSide1(); // NEW WAY: Use getter

    // Check for home command before starting
    if (checkForHomeCommand()) {
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (before starting)");
        return false;
    }

    //! Set Servo Angle FIRST
    myServo.setAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 1 side");

    //! STEP 0: Turn on pressure pot
    PressurePot_ON();
    
    // Check for home command after servo and pressure
    if (checkForHomeCommand()) {
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after prep)");
        return false;
    }

    //! STEP 1: Move to side 1 painting Z height
    // Use constants directly from settings/painting.h
    long zPos = (long)(paintingSettings.getSide1ZHeight() * STEPS_PER_INCH_XYZ); // Use getter
    long sideZPos = (long)(paintingSettings.getSide1SideZHeight() * STEPS_PER_INCH_XYZ); // Use getter

    // Use constants from utils/settings.h for default speeds
    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);
              
    // Check for home command after Z move
    if (checkForHomeCommand()) {
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after initial Z)");
        return false;
    }

    //! STEP 2: Rotate to the side 1 position
    rotateToAngle(SIDE1_ROTATION_ANGLE); // Speed likely handled within rotateToAngle
    Serial.println("Rotated to side 1 position");
    
    // Check for home command after rotation
    if (checkForHomeCommand()) {
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after rotation)");
        return false;
    }

    //! STEP 3: Move to start position (P2)
    long startX = (long)(paintingSettings.getSide1StartX() * STEPS_PER_INCH_XYZ); // Use getter
    long startY = (long)(paintingSettings.getSide1StartY() * STEPS_PER_INCH_XYZ); // Use getter
    moveToXYZ(startX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to side 1 pattern start position (P2)");
    
    // Check for home command after move to start
    if (checkForHomeCommand()) {
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after move to start)");
        return false;
    }

    //! STEP 4: Lower to painting Z height
    moveToXYZ(startX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
    
    // Check for home command after Z lower
    if (checkForHomeCommand()) {
        // Raise to safe Z height before aborting
        moveToXYZ(startX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after Z lower)");
        return false;
    }

    //! STEP 5: Execute simplified side 1 painting pattern (Single X Shift)
    long currentX = startX;
    long currentY = startY;
    long shiftXDistance = (long)(paintingSettings.getSide1ShiftX() * STEPS_PER_INCH_XYZ); // Use getter for shift distance
    long xSpeed = paintingSettings.getSide1PaintingXSpeed(); // Use getter for X speed
    long ySpeed = paintingSettings.getSide1PaintingYSpeed(); // Use getter for Y speed (though Y isn't moving)
    long shutOffOffsetSteps = (long)(2.0f * STEPS_PER_INCH_XYZ); // 1.0 inches in steps

    Serial.println("Side 1 Pattern: Performing single X shift");
    paintGun_ON();

    // Calculate the final X position and the X position to trigger paint gun OFF
    long finalX = startX + shiftXDistance;
    long triggerX_paintOff = finalX - shutOffOffsetSteps;
    
    // Perform a single continuous move, triggering paintGun_OFF at triggerX_paintOff
    moveToXYZ_with_X_trigger(finalX, xSpeed, currentY, ySpeed, zPos, DEFAULT_Z_SPEED, 
                             triggerX_paintOff, paintGun_OFF);

    // Since paintGun_OFF is now called by moveToXYZ_with_X_trigger, 
    // we don't call it separately here.
    // The movement to finalX is also handled by the single call above.

    Serial.println("Paint gun OFF signal sent, travel to final X completed.");

    // Check for home command after the single move
    if (checkForHomeCommand()) {
        // Raise to safe Z height before aborting
        moveToXYZ(finalX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 1 Pattern Painting ABORTED due to home command (after painting)");
        return false;
    }

    //! STEP 6: Raise to safe Z height (Was STEP 8)
    moveToXYZ(finalX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Stage 5: Transition back to Homing State after completion
    Serial.println("Side 1 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState()); // Corrected state change call

    return true;
}
