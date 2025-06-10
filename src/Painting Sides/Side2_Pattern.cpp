#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"
#include "../../include/utils/settings.h"
#include "../../include/motors/Rotation_Motor.h"
#include "../../include/hardware/paintGun_Functions.h"
#include "../../include/hardware/pressurePot_Functions.h"
#include "../../include/settings/painting.h"
#include "../../include/persistence/PaintingSettings.h"
#include <FastAccelStepper.h>
#include "../../include/motors/ServoMotor.h"
#include "../../include/web/Web_Dashboard_Commands.h"
#include "../../include/system/StateMachine.h"

// External references to stepper motors
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
extern ServoMotor myServo;
extern PaintingSettings paintingSettings;
extern StateMachine* stateMachine;

//* ************************************************************************
//* ************************** SIDE 2 PAINTING ***************************
//* ************************************************************************
//* SIDE 2 PAINTING PATTERN (Vertical Sweeps, specific sequence) - NEW LOGIC
//* Based on original Side 4 pattern (5-sweep, starts -Y, shifts -X)
//*
//*    P9                      P7                      P5                      P3                      P1 (SIDE2_START_X,Y)
//*    | (-Y)                | (+Y)                  | (-Y)                  | (+Y)                  | (-Y) Start Sweep
//*    ↓                     ↑                       ↓                       ↑                       ↓
//*    P10 (END)               P8                      P6                      P4                      P2
//*         ← (-X shift)          ← (-X shift)          ← (-X shift)          ← (-X shift)
//* Pattern: Start at (Side2_StartX, Side2_StartY). Sweep -Y. Shift -X. Sweep +Y. Shift -X. Sweep -Y ... for 5 Y sweeps.

void paintSide2Pattern() {
    Serial.println("Starting Side 2 Pattern Painting (New Logic - Swapped from Side 4)");

    int servoAngle = paintingSettings.getServoAngleSide2(); // Use Side 2 settings
    long zPos = (long)(paintingSettings.getSide2ZHeight() * STEPS_PER_INCH_XYZ); // Use Side 2 settings
    long sideZPos = (long)(paintingSettings.getSide2SideZHeight() * STEPS_PER_INCH_XYZ); // Use Side 2 settings
    long startX_steps = (long)(paintingSettings.getSide2StartX() * STEPS_PER_INCH_XYZ); // Use Side 2 settings
    long startY_steps = (long)(paintingSettings.getSide2StartY() * STEPS_PER_INCH_XYZ); // Use Side 2 settings
    long sweepYDistance = (long)(paintingSettings.getSide2SweepY() * STEPS_PER_INCH_XYZ); // Use Side 2 settings
    long shiftXDistance = (long)(paintingSettings.getSide2ShiftX() * STEPS_PER_INCH_XYZ); // Use Side 2 settings - ensure this is positive
    long paint_x_speed = paintingSettings.getSide2PaintingXSpeed(); // Use Side 2 settings
    long paint_y_speed = paintingSettings.getSide2PaintingYSpeed(); // Use Side 2 settings
    long first_sweep_paint_y_speed_side2 = (long)(paint_y_speed * 0.75f);
    long paintOffsetSteps = (long)(0.25f * STEPS_PER_INCH_XYZ); // 0.25 inches in steps

    //! Set Servo Angle FIRST
    myServo.setAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 2");

    //! STEP 0: Turn on pressure pot
    PressurePot_ON();

    //! STEP 1: Move to Side 2 safe Z height at current X, Y
    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);

    //! STEP 2: Rotate to the Side 2 position
    rotateToAngle(SIDE2_ROTATION_ANGLE); // Use Side 2 angle
    Serial.println("Rotated to Side 2 position");

    //! STEP 3: Move to user-defined start X, Y for Side 2 at safe Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to Side 2 Start X, Y at safe Z.");

    //! STEP 4: Lower to painting Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
    Serial.println("Lowered to painting Z for Side 2.");

    long currentX = startX_steps;
    long currentY = startY_steps;
    const int num_y_sweeps = 5; // 5 Y-sweeps results in 4 X-shifts

    for (int i = 0; i < num_y_sweeps; ++i) {
        bool isNegativeYSweep = (i % 2 == 0); // 0th, 2nd, 4th sweeps are -Y
        long current_paint_y_speed = paint_y_speed;

        if (isNegativeYSweep) {
            Serial.printf("Side 2 Pattern: Sweep %d (-Y) with smooth paint gun control\\\n", i + 1);
            if (i == 0) { // First sweep
                current_paint_y_speed = first_sweep_paint_y_speed_side2;
                Serial.printf("Side 2 Pattern: Applying 75%% speed for first sweep: %ld\\n", current_paint_y_speed);
            }
            
            // Calculate smooth movement parameters
            long finalY = currentY - sweepYDistance;
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            
            // Start smooth movement for BOTH Y motors
            unsigned long moveStartTime = millis();
            stepperY_Left->moveTo(finalY);
            stepperY_Left->setSpeedInHz(current_paint_y_speed);
            stepperY_Right->moveTo(finalY);
            stepperY_Right->setSpeedInHz(current_paint_y_speed);
            
            bool paintGunActivated = false;
            bool paintGunDeactivated = false;
            
            while(stepperY_Left->isRunning() || stepperY_Right->isRunning()) {
                unsigned long currentTime = millis();
                float elapsedSeconds = (currentTime - moveStartTime) / 1000.0f;
                
                if (!paintGunActivated && elapsedSeconds >= (timeToStart / 1000.0f)) {
                    paintGun_ON();
                    paintGunActivated = true;
                }
                
                if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= (timeToStop / 1000.0f)) {
                    paintGun_OFF();
                    paintGunDeactivated = true;
                }
                
                if (checkForHomeCommand()) {
                    stepperY_Left->forceStop();
                    stepperY_Right->forceStop();
                    paintGun_OFF();
                    return;
                }
                delay(1);
            }
            
            paintGun_OFF();
            currentY = finalY;
        } else {
            Serial.printf("Side 2 Pattern: Sweep %d (+Y) with smooth paint gun control\\\n", i + 1);
            
            // Calculate smooth movement parameters
            long finalY = currentY + sweepYDistance;
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            
            // Start smooth movement for BOTH Y motors
            unsigned long moveStartTime = millis();
            stepperY_Left->moveTo(finalY);
            stepperY_Left->setSpeedInHz(current_paint_y_speed);
            stepperY_Right->moveTo(finalY);
            stepperY_Right->setSpeedInHz(current_paint_y_speed);
            
            bool paintGunActivated = false;
            bool paintGunDeactivated = false;
            
            while(stepperY_Left->isRunning() || stepperY_Right->isRunning()) {
                unsigned long currentTime = millis();
                float elapsedSeconds = (currentTime - moveStartTime) / 1000.0f;
                
                if (!paintGunActivated && elapsedSeconds >= (timeToStart / 1000.0f)) {
                    paintGun_ON();
                    paintGunActivated = true;
                }
                
                if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= (timeToStop / 1000.0f)) {
                    paintGun_OFF();
                    paintGunDeactivated = true;
                }
                
                if (checkForHomeCommand()) {
                    stepperY_Left->forceStop();
                    stepperY_Right->forceStop();
                    paintGun_OFF();
                    return;
                }
                delay(1);
            }
            
            paintGun_OFF();
            currentY = finalY;
        }

        if (checkForHomeCommand()) {
            moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
            Serial.println("Side 2 Pattern Painting ABORTED due to home command");
            return;
        }

        // Perform X shift if it's not the last Y sweep
        if (i < num_y_sweeps - 1) {
            Serial.printf("Side 2 Pattern: Shift -X after sweep %d\\\\n", i + 1);
            currentX -= shiftXDistance; // Shift in -X direction (ensure shiftXDistance is positive in settings)
            moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
            
            if (checkForHomeCommand()) {
                 moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
                 Serial.println("Side 2 Pattern Painting ABORTED due to home command during X shift");
                 return;
            }
        }
    }

    //! NEW: Perform additional movements at the end of the pattern
    Serial.println("Side 2 Pattern: Starting end sequence movements.");

    //! Move -2 inches in X (gun OFF)
    paintGun_OFF(); // Ensure gun is off
    long endSeq_targetX1 = currentX - (long)(2.0 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 2 Pattern: Moving to X=%ld, Y=%ld (relative -2in)\\n", endSeq_targetX1, currentY);
    moveToXYZ(endSeq_targetX1, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED); // Keep at painting Z
    currentX = endSeq_targetX1;

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 2 Pattern Painting ABORTED during end sequence (move 1) due to home command");
        return;
    }

    //! Set Servo Angle and Z height for final X pass
    myServo.setAngle(85);
    Serial.println("Servo set to: 85 degrees for final X pass on Side 2");
    long finalXPassZPos_Side2 = (long)(-1.75 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 2 Pattern: Setting Z to %.2f inches for final X pass\n", -1.75);

    //! Move +23 inches in X with smooth paint gun control
    Serial.println("Side 2 Pattern: Starting +23in X sweep with smooth paint gun control.");
    long finalXSweepDistance = (long)(23.0 * STEPS_PER_INCH_XYZ);
    long endSeq_targetX2 = currentX + finalXSweepDistance;
    
    // Calculate smooth movement parameters
    float totalDistance = 23.0f; // Distance in inches
    float timeToStart = 0.25f * 60.0f / ((float)paint_x_speed / STEPS_PER_INCH_XYZ);
    float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)paint_x_speed / STEPS_PER_INCH_XYZ);
    
    // Start smooth movement with Z change
    moveToXYZ(endSeq_targetX2, paint_x_speed, currentY, paint_y_speed, finalXPassZPos_Side2, DEFAULT_Z_SPEED);
    
    // Monitor movement for paint gun control
    unsigned long moveStartTime = millis();
    bool paintGunActivated = false;
    bool paintGunDeactivated = false;
    
    while(stepperX->isRunning() || stepperZ->isRunning()) {
        unsigned long currentTime = millis();
        float elapsedSeconds = (currentTime - moveStartTime) / 1000.0f;
        
        if (!paintGunActivated && elapsedSeconds >= (timeToStart / 1000.0f)) {
            paintGun_ON();
            paintGunActivated = true;
            Serial.println("Side 2 Pattern: Gun ON after 0.25in offset for final X sweep.");
        }
        
        if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= (timeToStop / 1000.0f)) {
            paintGun_OFF();
            paintGunDeactivated = true;
            Serial.println("Side 2 Pattern: Gun OFF, 0.25in before end of X sweep.");
        }
        
        if (checkForHomeCommand()) {
            stepperX->forceStop();
            stepperZ->forceStop();
            paintGun_OFF();
            Serial.println("Side 2 Pattern Painting ABORTED during end sequence due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = endSeq_targetX2;

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 2 Pattern Painting ABORTED during end sequence (move 2) due to home command");
        return;
    }
    Serial.println("Side 2 Pattern: End sequence movements completed.");

    // Move Z to safe height
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Move to position (3,3) before homing
    Serial.println("Moving to position (3,3,0) before homing...");
    long xHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long yHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long zHoming = 0;
    moveToXYZ(xHoming, DEFAULT_X_SPEED, yHoming, DEFAULT_Y_SPEED, zHoming, DEFAULT_Z_SPEED);
    Serial.println("Reached position (3,3,0).");

    //! Transition to Homing State
    Serial.println("Side 2 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState()); // Corrected state change call
}