#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"       // For moveToXYZ
#include "../../include/utils/settings.h"         // For STEPS_PER_INCH, default speeds
#include "../../include/motors/Rotation_Motor.h"     // For rotateToAngle
#include "../../include/hardware/paintGun_Functions.h" // For paintGun_ON/OFF
#include "../../include/hardware/pressurePot_Functions.h" // For PressurePot_ON
#include "../../include/settings/painting.h"         // For painting-specific constants (SIDE3_Z_HEIGHT etc.)
#include "storage/painting_settings.h" // Function-based painting settings
#include <FastAccelStepper.h>
#include "../../include/motors/ServoMotor.h"         // For ServoMotor class
#include "../../include/web/Web_Dashboard_Commands.h" // For checkForHomeCommand
#include "../../include/system/StateMachine.h" // Include StateMachine header
#include <WebSocketsServer.h>     // For webSocket.loop()

// External references to stepper motors
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
extern ServoMotor myServo;
// Removed extern PaintingSettings - using function-based approach
// Function-based StateMachine - no extern needed
extern WebSocketsServer webSocket;    // For immediate command processing

// External references to immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

// Function to check for immediate commands during painting operations
bool checkForImmediateCommandsSide3() {
    // Process any pending WebSocket events to catch immediate commands
    webSocket.loop();
    
    // DEBUG: Print status every 100 calls
    static int debugCounter = 0;
    debugCounter++;
    if (debugCounter % 100 == 0) {
        Serial.printf("DEBUG: CheckImmediate call #%d, immediateCommandPending=%s\n", 
                     debugCounter, immediateCommandPending ? "TRUE" : "FALSE");
    }
    
    // Check for immediate commands
    if (immediateCommandPending) {
        Serial.println("*** IMMEDIATE COMMAND DETECTED during Side3 painting - ABORTING NOW! ***");
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
//* **************************** SIDE 3 PAINTING ****************************
//* ************************************************************************
//* SIDE 3 SIDE PAINTING PATTERN (Horizontal Sweeps)
//*
//* P1 (startX,startY)  ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←  (startX-sweepX,startY)
//*       |                                                      |
//*       | Shift 1 (Y-)                                         |
//*       ↓                                                      |
//* (startX,startY-shiftY)  → → → → → → → → → → → → → → → →  (startX-sweepX,startY-shiftY)
//*       |                                                      |
//*       | Shift 2 (Y-)                                         |
//*       ↓                                                      |
//* (startX,startY-2*shiftY)← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←  (startX-sweepX,startY-2*shiftY)
//*       |                                                      |
//*       | Shift 3 (Y-)                                         |
//*       ↓                                                      |
//* (startX,startY-3*shiftY)→ → → → → → → → → → → → → → → →  (startX-sweepX,startY-3*shiftY)
//*
//* Sequence: Start → Sweep X- → Shift Y- → Sweep X+ → Shift Y- → Sweep X- → Shift Y- → Sweep X+
//* Paint ON during horizontal (X) sweeps. Start position assumed to be top-right corner.
//*

// Function to paint the side 3 pattern
void paintSide3Pattern() {
    Serial.println("Starting Side 3 Pattern Painting (Horizontal Sweeps) - WITH IMMEDIATE COMMAND SUPPORT");

    int servoAngle = getServoAngleSide3();

    //! Set Servo Angle FIRST
    myServo.setAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 3 side");

    //! STEP 0: Turn on pressure pot
    PressurePot_ON();

    //! STEP 1: Move to side 3 painting Z height
    long zPos = (long)(getSide3ZHeight() * STEPS_PER_INCH_XYZ);
    long sideZPos = (long)(getSide3SideZHeight() * STEPS_PER_INCH_XYZ);

    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);

    //! STEP 2: Rotate to the side 3 position
    rotateToAngle(SIDE3_ROTATION_ANGLE);
    Serial.println("Rotated to side 3 position");

    //! STEP 3: Move to start position (Top Right - P1 assumed)
    long startX_steps = (long)(getSide3StartX() * STEPS_PER_INCH_XYZ);
    long startY_steps = (long)(getSide3StartY() * STEPS_PER_INCH_XYZ);
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to side 3 pattern start position (Top Right)");

    //! STEP 4: Lower to painting Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    //! STEP 5: Execute side 3 horizontal painting pattern
    long currentX = startX_steps;
    long currentY = startY_steps;
    long sweepX_steps = (long)(getSide3ShiftX() * STEPS_PER_INCH_XYZ); 
    long shiftY_steps = (long)(getSide3SweepY() * STEPS_PER_INCH_XYZ); 

    long paint_x_speed = getSide3PaintingXSpeed();
    long paint_y_speed = getSide3PaintingYSpeed();
    long final_sweep_paint_x_speed_side3 = (long)(paint_x_speed * 0.75f);
    long paintOffsetSteps = (long)(0.25f * STEPS_PER_INCH_XYZ); // 0.25 inches in steps

    // First sweep: X- direction with smooth motion
    Serial.println("Side 3 Pattern: First sweep X- with smooth paint gun control + IMMEDIATE COMMAND SUPPORT");
    
    long finalX1 = currentX - sweepX_steps;
    float totalDistance1 = (float)sweepX_steps / STEPS_PER_INCH_XYZ;
    float timeToStart1 = 0.25f * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    float timeToStop1 = (totalDistance1 - 0.5f) * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    
    unsigned long moveStartTime1 = millis();
    stepperX->moveTo(finalX1);
    stepperX->setSpeedInHz(paint_x_speed);
    
    bool paintGunActivated1 = false;
    bool paintGunDeactivated1 = false;
    
    while(stepperX->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime1) / 1000.0f;
        
        if (!paintGunActivated1 && elapsedSeconds >= timeToStart1) {
            paintGun_ON();
            paintGunActivated1 = true;
        }
        
        if (paintGunActivated1 && !paintGunDeactivated1 && elapsedSeconds >= timeToStop1) {
            paintGun_OFF();
            paintGunDeactivated1 = true;
        }
        
        // **REVOLUTIONARY CHANGE**: Call the full dashboard server function
        // This processes WebSocket events 15+ times per iteration!
        runDashboardServer();
        
        // NEW: Check for immediate commands during motor movement
        if (checkForImmediateCommandsSide3()) {
            Serial.println("*** Side 3 Sweep 1 ABORTED due to immediate command ***");
            return; // Exit immediately
        }
        
        // Shorter delay for more responsive checking
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX1;

    // Check for immediate commands after sweep
    if (checkForImmediateCommandsSide3()) {
        Serial.println("Side 3 Pattern Painting ABORTED due to immediate command after sweep 1");
        return;
    }

    // First shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Second sweep: X+ direction with smooth motion
    Serial.println("Side 3 Pattern: Second sweep X+ with smooth paint gun control + IMMEDIATE COMMAND SUPPORT");
    
    long finalX2 = currentX + sweepX_steps;
    float totalDistance2 = (float)sweepX_steps / STEPS_PER_INCH_XYZ;
    float timeToStart2 = 0.25f * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    float timeToStop2 = (totalDistance2 - 0.5f) * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    
    unsigned long moveStartTime2 = millis();
    stepperX->moveTo(finalX2);
    stepperX->setSpeedInHz(paint_x_speed);
    
    bool paintGunActivated2 = false;
    bool paintGunDeactivated2 = false;
    
    while(stepperX->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime2) / 1000.0f;
        
        if (!paintGunActivated2 && elapsedSeconds >= timeToStart2) {
            paintGun_ON();
            paintGunActivated2 = true;
        }
        
        if (paintGunActivated2 && !paintGunDeactivated2 && elapsedSeconds >= timeToStop2) {
            paintGun_OFF();
            paintGunDeactivated2 = true;
        }
        
        // **REVOLUTIONARY CHANGE**: Call the full dashboard server function
        runDashboardServer();
        
        // NEW: Check for immediate commands during motor movement
        if (checkForImmediateCommandsSide3()) {
            Serial.println("*** Side 3 Sweep 2 ABORTED due to immediate command ***");
            return; // Exit immediately
        }
        
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX2;

    // Check for immediate commands after sweep
    if (checkForImmediateCommandsSide3()) {
        Serial.println("Side 3 Pattern Painting ABORTED due to immediate command after sweep 2");
        return;
    }

    // Second shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Third sweep: X- direction with smooth motion
    Serial.println("Side 3 Pattern: Third sweep X- with smooth paint gun control + IMMEDIATE COMMAND SUPPORT");
    
    long finalX3 = currentX - sweepX_steps;
    float totalDistance3 = (float)sweepX_steps / STEPS_PER_INCH_XYZ;
    float timeToStart3 = 0.25f * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    float timeToStop3 = (totalDistance3 - 0.5f) * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    
    unsigned long moveStartTime3 = millis();
    stepperX->moveTo(finalX3);
    stepperX->setSpeedInHz(paint_x_speed);
    
    bool paintGunActivated3 = false;
    bool paintGunDeactivated3 = false;
    
    while(stepperX->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime3) / 1000.0f;
        
        if (!paintGunActivated3 && elapsedSeconds >= timeToStart3) {
            paintGun_ON();
            paintGunActivated3 = true;
        }
        
        if (paintGunActivated3 && !paintGunDeactivated3 && elapsedSeconds >= timeToStop3) {
            paintGun_OFF();
            paintGunDeactivated3 = true;
        }
        
        // **REVOLUTIONARY CHANGE**: Call the full dashboard server function
        runDashboardServer();
        
        // NEW: Check for immediate commands during motor movement
        if (checkForImmediateCommandsSide3()) {
            Serial.println("*** Side 3 Sweep 3 ABORTED due to immediate command ***");
            return; // Exit immediately
        }
        
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX3;

    // Check for immediate commands after sweep
    if (checkForImmediateCommandsSide3()) {
        Serial.println("Side 3 Pattern Painting ABORTED due to immediate command after sweep 3");
        return;
    }

    // Third shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fourth sweep: X+ direction with smooth motion
    Serial.println("Side 3 Pattern: Fourth sweep X+ with smooth paint gun control + IMMEDIATE COMMAND SUPPORT");
    
    long finalX4 = currentX + sweepX_steps;
    float totalDistance4 = (float)sweepX_steps / STEPS_PER_INCH_XYZ;
    float timeToStart4 = 0.25f * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    float timeToStop4 = (totalDistance4 - 0.5f) * STEPS_PER_INCH_XYZ / (float)paint_x_speed;
    
    unsigned long moveStartTime4 = millis();
    stepperX->moveTo(finalX4);
    stepperX->setSpeedInHz(paint_x_speed);
    
    bool paintGunActivated4 = false;
    bool paintGunDeactivated4 = false;
    
    while(stepperX->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime4) / 1000.0f;
        
        if (!paintGunActivated4 && elapsedSeconds >= timeToStart4) {
            paintGun_ON();
            paintGunActivated4 = true;
        }
        
        if (paintGunActivated4 && !paintGunDeactivated4 && elapsedSeconds >= timeToStop4) {
            paintGun_OFF();
            paintGunDeactivated4 = true;
        }
        
        // **REVOLUTIONARY CHANGE**: Call the full dashboard server function
        runDashboardServer();
        
        // NEW: Check for immediate commands during motor movement
        if (checkForImmediateCommandsSide3()) {
            Serial.println("*** Side 3 Sweep 4 ABORTED due to immediate command ***");
            return; // Exit immediately
        }
        
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX4;

    // Check for immediate commands after sweep
    if (checkForImmediateCommandsSide3()) {
        Serial.println("Side 3 Pattern Painting ABORTED due to immediate command after sweep 4");
        return;
    }

    // Fourth shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fifth sweep: X- direction (Final X painting movement) with smooth motion
    Serial.println("Side 3 Pattern: Fifth sweep X- with smooth paint gun control + IMMEDIATE COMMAND SUPPORT");
    Serial.printf("Side 3 Pattern: Applying 75%% speed for final X sweep: %ld\n", final_sweep_paint_x_speed_side3);
    
    long finalX5 = currentX - sweepX_steps;
    float totalDistance5 = (float)sweepX_steps / STEPS_PER_INCH_XYZ;
    float timeToStart5 = 0.25f * STEPS_PER_INCH_XYZ / (float)final_sweep_paint_x_speed_side3;
    float timeToStop5 = (totalDistance5 - 0.5f) * STEPS_PER_INCH_XYZ / (float)final_sweep_paint_x_speed_side3;
    
    unsigned long moveStartTime5 = millis();
    stepperX->moveTo(finalX5);
    stepperX->setSpeedInHz(final_sweep_paint_x_speed_side3);
    
    bool paintGunActivated5 = false;
    bool paintGunDeactivated5 = false;
    
    while(stepperX->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime5) / 1000.0f;
        
        if (!paintGunActivated5 && elapsedSeconds >= timeToStart5) {
            paintGun_ON();
            paintGunActivated5 = true;
        }
        
        if (paintGunActivated5 && !paintGunDeactivated5 && elapsedSeconds >= timeToStop5) {
            paintGun_OFF();
            paintGunDeactivated5 = true;
        }
        
        // **REVOLUTIONARY CHANGE**: Call the full dashboard server function
        runDashboardServer();
        
        // NEW: Check for immediate commands during motor movement
        if (checkForImmediateCommandsSide3()) {
            Serial.println("*** Side 3 Sweep 5 ABORTED due to immediate command ***");
            return; // Exit immediately
        }
        
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX5;

    // Final check for immediate commands
    if (checkForImmediateCommandsSide3()) {
        Serial.println("Side 3 Pattern Painting ABORTED due to immediate command after final sweep");
        return;
    }

    //! STEP 8: Raise to safe Z height
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Move to position (1,1,0) before homing
    moveToPositionOneOneBeforeHoming();

    //! Transition to Homing State
    Serial.println("Side 3 painting complete. Transitioning to Homing State...");
    changeState(MachineState::HOMING);
    // No return needed as function is void
} 