#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"       // For moveToXYZ
#include "../../include/utils/settings.h"         // For STEPS_PER_INCH, default speeds
#include "../../include/motors/Rotation_Motor.h"     // For rotateToAngle
#include "../../include/hardware/paintGun_Functions.h" // For paintGun_ON/OFF
#include "../../include/hardware/pressurePot_Functions.h" // For PressurePot_ON
#include <FastAccelStepper.h>
#include "storage/painting_settings.h" // Function-based painting settings
#include "../../include/settings/painting.h"         // For painting-specific constants (SIDE1_Z_HEIGHT etc.)
#include "motors/servo_motor.h"         // For ServoMotor class
#include "states/PaintingState.h" // Correct filename
#include "settings/pins.h"        // Keep this one
#include "../../include/web/Web_Dashboard_Commands.h" // For checkForHomeCommand
#include "../../include/system/StateMachine.h" // Include StateMachine header
#include <WebSocketsServer.h>     // For webSocket.loop()

// External references to stepper motors
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
// Removed extern ServoMotor - using function-based approach
// Removed extern PaintingSettings - using function-based approach
extern WebSocketsServer webSocket;    // For immediate command processing

// External references to immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

// Function to check for immediate commands during painting operations
bool checkForImmediateCommandsSide1() {
    // Process any pending WebSocket events to catch immediate commands
    webSocket.loop();
    
    // Check for immediate commands
    if (immediateCommandPending) {
        Serial.println("*** IMMEDIATE COMMAND DETECTED during Side1 painting - ABORTING NOW! ***");
        Serial.printf("*** Command: %s, Client: %d ***\n", pendingCommand.c_str(), pendingCommandClientNum);
        
        // Stop all motors immediately
        if (stepperX && stepperX->isRunning()) {
            Serial.println("*** FORCE STOPPING X MOTOR ***");
            stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
        }
        if (stepperY_Left && stepperY_Left->isRunning()) {
            Serial.println("*** FORCE STOPPING Y_LEFT MOTOR ***");
            stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
        }
        if (stepperY_Right && stepperY_Right->isRunning()) {
            Serial.println("*** FORCE STOPPING Y_RIGHT MOTOR ***");
            stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
        }
        if (stepperZ && stepperZ->isRunning()) {
            Serial.println("*** FORCE STOPPING Z MOTOR ***");
            stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
        }
        
        // Turn off paint gun for safety
        Serial.println("*** FORCE TURNING OFF PAINT GUN ***");
        paintGun_OFF();
        
        return true; // Immediate command pending
    }
    
    return false; // No immediate commands
}

//* ************************************************************************
//* *************************** SIDE 1 *************************************
//* ************************************************************************

// Simplified hardcoded painting function - no pauses or checks
bool paintSide1Pattern() {
    Serial.println("Starting Side 1 Pattern Painting - With Immediate Command Support");

    //! Set Servo Angle
    int servoAngle = getServoAngleSide1();
    setServoAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 1");

    //! Turn on pressure pot
    PressurePot_ON();

    //! Move to side 1 safe Z height
    long zPos = (long)(getSide1ZHeight() * STEPS_PER_INCH_XYZ);
    long sideZPos = (long)(getSide1SideZHeight() * STEPS_PER_INCH_XYZ);
    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);

    //! Rotate to side 1 position
    rotateToAngle(SIDE1_ROTATION_ANGLE);
    Serial.println("Rotated to side 1 position");

    //! Move to start position
    long startX = (long)(getSide1StartX() * STEPS_PER_INCH_XYZ);
    long startY = (long)(getSide1StartY() * STEPS_PER_INCH_XYZ);
    moveToXYZ(startX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to side 1 pattern start position");

    //! Lower to painting Z height
    moveToXYZ(startX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    //! Execute painting pattern - continuous motion with paint gun control
    long shiftXDistance = (long)(getSide1ShiftX() * STEPS_PER_INCH_XYZ);
    long xSpeed = getSide1PaintingXSpeed();
    long finalX = startX + shiftXDistance;
    long paintStartX = startX + (long)(0.25f * STEPS_PER_INCH_XYZ);
    long paintStopX = finalX - (long)(1.0f * STEPS_PER_INCH_XYZ);

    Serial.println("Side 1 Pattern: Executing continuous X movement with paint gun control");

    // Start continuous movement from startX to finalX
    stepperX->moveTo(finalX);
    stepperX->setSpeedInHz(xSpeed);
    
    bool paintGunOn = false;
    
    // Monitor movement and control paint gun based on position
    // **CRITICAL FIX**: Add immediate command checking during motion
    while (stepperX->isRunning()) {
        long currentX = stepperX->getCurrentPosition();
        
        // **KEY ENHANCEMENT**: Check for immediate commands during motion
        if (checkForImmediateCommandsSide1()) {
            Serial.println("*** Side 1 painting ABORTED due to immediate command ***");
            return false; // Exit immediately
        }
        
        // Turn paint gun ON when reaching paint start position
        if (!paintGunOn && currentX >= paintStartX) {
            paintGun_ON();
            paintGunOn = true;
            Serial.println("Paint gun ON during movement");
        }
        
        // Turn paint gun OFF when reaching paint stop position
        if (paintGunOn && currentX >= paintStopX) {
            paintGun_OFF();
            paintGunOn = false;
            Serial.println("Paint gun OFF during movement");
        }
        
        // Small delay to prevent excessive CPU usage
        delay(1);
    }
    
    // Ensure paint gun is off at the end
    if (paintGunOn) {
        paintGun_OFF();
        Serial.println("Paint gun OFF - movement complete");
    }

    //! STEP 6: Raise to safe Z height (Was STEP 8)
    moveToXYZ(finalX, DEFAULT_X_SPEED, startY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Move to position (1,1,0) before homing
    moveToPositionOneOneBeforeHoming();

    //! Stage 5: Transition back to Homing State after completion
    Serial.println("Side 1 painting complete. Transitioning to Homing State...");
    changeState(MachineState::HOMING);

    return true;
}
