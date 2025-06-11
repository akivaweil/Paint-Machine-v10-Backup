#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"       // For moveToXYZ
#include "../../include/utils/settings.h"         // For STEPS_PER_INCH, default speeds
#include "../../include/motors/Rotation_Motor.h"     // For rotateToAngle
#include "../../include/hardware/paintGun_Functions.h" // For paintGun_ON/OFF
#include "../../include/hardware/pressurePot_Functions.h" // For PressurePot_ON
#include "../../include/settings/painting.h"         // For painting-specific constants (SIDE3_Z_HEIGHT etc.)
#include "../../include/persistence/PaintingSettings.h" // Include for accessing saved settings
#include <FastAccelStepper.h>
#include "../../include/motors/ServoMotor.h"         // For ServoMotor class
#include "../../include/web/Web_Dashboard_Commands.h" // For checkForHomeCommand
#include "../../include/system/StateMachine.h" // Include StateMachine header

// External references to stepper motors
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
extern ServoMotor myServo;
extern PaintingSettings paintingSettings; // Make sure global instance is accessible
extern StateMachine* stateMachine; // Declare external state machine instance

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
    Serial.println("Starting Side 3 Pattern Painting (Horizontal Sweeps)");

    int servoAngle = paintingSettings.getServoAngleSide3();

    //! Set Servo Angle FIRST
    myServo.setAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 3 side");

    //! STEP 0: Turn on pressure pot
    PressurePot_ON();

    //! STEP 1: Move to side 3 painting Z height
    long zPos = (long)(paintingSettings.getSide3ZHeight() * STEPS_PER_INCH_XYZ);
    long sideZPos = (long)(paintingSettings.getSide3SideZHeight() * STEPS_PER_INCH_XYZ);

    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);

    //! STEP 2: Rotate to the side 3 position
    rotateToAngle(SIDE3_ROTATION_ANGLE);
    Serial.println("Rotated to side 3 position");

    //! STEP 3: Move to start position (Top Right - P1 assumed)
    long startX_steps = (long)(paintingSettings.getSide3StartX() * STEPS_PER_INCH_XYZ);
    long startY_steps = (long)(paintingSettings.getSide3StartY() * STEPS_PER_INCH_XYZ);
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to side 3 pattern start position (Top Right)");

    //! STEP 4: Lower to painting Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    //! STEP 5: Execute side 3 horizontal painting pattern
    long currentX = startX_steps;
    long currentY = startY_steps;
    long sweepX_steps = (long)(paintingSettings.getSide3ShiftX() * STEPS_PER_INCH_XYZ); 
    long shiftY_steps = (long)(paintingSettings.getSide3SweepY() * STEPS_PER_INCH_XYZ); 

    long paint_x_speed = paintingSettings.getSide3PaintingXSpeed();
    long paint_y_speed = paintingSettings.getSide3PaintingYSpeed();
    long final_sweep_paint_x_speed_side3 = (long)(paint_x_speed * 0.75f);
    long paintOffsetSteps = (long)(0.25f * STEPS_PER_INCH_XYZ); // 0.25 inches in steps

    // First sweep: X- direction with smooth motion
    Serial.println("Side 3 Pattern: First sweep X- with smooth paint gun control");
    
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
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 3 Pattern Painting ABORTED due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX1;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    // First shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Second sweep: X+ direction with smooth motion
    Serial.println("Side 3 Pattern: Second sweep X+ with smooth paint gun control");
    
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
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 3 Pattern Painting ABORTED due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX2;

    // Second shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Third sweep: X- direction with smooth motion
    Serial.println("Side 3 Pattern: Third sweep X- with smooth paint gun control");
    
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
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 3 Pattern Painting ABORTED due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX3;

    // Third shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fourth sweep: X+ direction with smooth motion
    Serial.println("Side 3 Pattern: Fourth sweep X+ with smooth paint gun control");
    
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
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 3 Pattern Painting ABORTED due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX4;

    // Fourth shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fifth sweep: X- direction (Final X painting movement) with smooth motion
    Serial.println("Side 3 Pattern: Fifth sweep X- with smooth paint gun control");
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
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 3 Pattern Painting ABORTED due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = finalX5;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    //! STEP 8: Raise to safe Z height
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Move to position (1,1,0) before homing
    moveToPositionOneOneBeforeHoming();

    //! Transition to Homing State
    Serial.println("Side 3 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState()); // Corrected state change call
    // No return needed as function is void
} 