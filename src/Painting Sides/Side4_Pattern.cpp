#include <Arduino.h>
#include "../../include/motors/XYZ_Movements.h"
#include "../../include/utils/settings.h"
#include "../../include/motors/Rotation_Motor.h"
#include "../../include/hardware/paintGun_Functions.h"
#include "../../include/hardware/pressurePot_Functions.h"
#include <FastAccelStepper.h>
#include "../../include/settings/painting.h"
#include "../../include/motors/ServoMotor.h"
#include "../../include/persistence/PaintingSettings.h"
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
//* ************************** SIDE 4 PAINTING ***************************
//* ************************************************************************
//* SIDE 4 PAINTING PATTERN (Vertical Sweeps) - NEW LOGIC
//* Based on original Side 2 pattern (5-sweep, starts +Y, shifts +X)
//*
//*    P1 (SIDE4_START_X,Y)         P3                      P5                      P7                      P9
//*          ↓ (+Y Start Sweep)      ↑ (-Y)                  ↓ (+Y)                  ↑ (-Y)                  ↓ (+Y)
//*    P2                      P4                      P6                      P8                      P10 (END)
//*          → (+X shift)          → (+X shift)          → (+X shift)          → (+X shift)
//*
//* Pattern: Start at (Side4_StartX, Side4_StartY). Sweep +Y. Shift +X. Sweep -Y. Shift +X. Sweep +Y ... for 5 Y sweeps.    

void paintSide4Pattern() {
    Serial.println("Starting Side 4 Pattern Painting (New Logic - Swapped from Side 2)");

    int servoAngle = paintingSettings.getServoAngleSide4(); // Use Side 4 settings
    long zPos = (long)(paintingSettings.getSide4ZHeight() * STEPS_PER_INCH_XYZ); // Use Side 4 settings
    long sideZPos = (long)(paintingSettings.getSide4SideZHeight() * STEPS_PER_INCH_XYZ); // Use Side 4 settings
    long startX_steps = (long)(paintingSettings.getSide4StartX() * STEPS_PER_INCH_XYZ); // Use Side 4 settings
    long startY_steps = (long)(paintingSettings.getSide4StartY() * STEPS_PER_INCH_XYZ); // Use Side 4 settings
    long sweepYDistance = (long)(paintingSettings.getSide4SweepY() * STEPS_PER_INCH_XYZ); // Use Side 4 settings
    long shiftXDistance = (long)(paintingSettings.getSide4ShiftX() * STEPS_PER_INCH_XYZ); // Use Side 4 settings - ensure this is positive for +X shift
    long paint_x_speed = paintingSettings.getSide4PaintingXSpeed(); // Use Side 4 settings
    long paint_y_speed = paintingSettings.getSide4PaintingYSpeed(); // Use Side 4 settings
    long initial_sweep_paint_y_speed_side4 = (long)(paint_y_speed * 0.75f); // Renamed from final_sweep_paint_y_speed_side4
    long paintOffsetSteps = (long)(0.25f * STEPS_PER_INCH_XYZ); // 0.25 inches in steps

    //! Set Servo Angle FIRST
    myServo.setAngle(servoAngle);
    Serial.println("Servo set to: " + String(servoAngle) + " degrees for Side 4");

    //! Rotate the tray to 90 degrees for Side 4
    Serial.println("Rotating tray to 90 degrees for Side 4 painting");
    rotateToAngle(SIDE4_ROTATION_ANGLE); // Changed from 90.0f to use constant
    Serial.println("Tray rotation to 90 degrees complete");

    //! STEP 0: Turn on pressure pot
    PressurePot_ON();

    //! STEP 1: Move to Side 4 safe Z height at current X,Y
    moveToXYZ(stepperX->getCurrentPosition(), DEFAULT_X_SPEED,
              stepperY_Left->getCurrentPosition(), DEFAULT_Y_SPEED,
              sideZPos, DEFAULT_Z_SPEED);

    //! STEP 3: Move to user-defined start X, Y for Side 4 at safe Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    Serial.println("Moved to Side 4 Start X, Y at safe Z.");

    //! STEP 4: Lower to painting Z height
    moveToXYZ(startX_steps, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
    Serial.println("Lowered to painting Z for Side 4.");

    long currentX = startX_steps;
    long currentY = startY_steps;
    const int num_y_sweeps = 5; // Corresponds to P1-P10 in the diagram if using 4 shifts

    for (int i = 0; i < num_y_sweeps; ++i) {
        bool isPositiveYSweep = (i % 2 == 0); // 0th, 2nd, 4th sweeps are +Y
        long current_paint_y_speed = paint_y_speed;

        if (i == 0) { // First sweep (changed from i == num_y_sweeps - 1)
            current_paint_y_speed = initial_sweep_paint_y_speed_side4;
            Serial.printf("Side 4 Pattern: Applying 75%% speed for first sweep: %ld\\n", current_paint_y_speed);
        }

        if (isPositiveYSweep) {
            Serial.printf("Side 4 Pattern: Sweep %d (+Y) - Moving to end position first, then painting backwards\\\n", i + 1);
            
            // First: Move to final position without painting
            long finalY = currentY + sweepYDistance;
            moveToXYZ(currentX, DEFAULT_X_SPEED, finalY, current_paint_y_speed, zPos, DEFAULT_Z_SPEED);
            
            if (checkForPauseCommand()) {
                stepperY_Left->forceStop();
                stepperY_Right->forceStop();
                paintGun_OFF();
                Serial.println("Side 4 Pattern Painting ABORTED due to home command");
                return;
            }
            
            // Second: Paint while moving back to original position (-Y direction)
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            
            // Start smooth movement back to original Y
            unsigned long moveStartTime = millis();
            stepperY_Left->moveTo(currentY);
            stepperY_Left->setSpeedInHz(current_paint_y_speed);
            stepperY_Right->moveTo(currentY);
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
                
                if (checkForPauseCommand()) {
                    stepperY_Left->forceStop();
                    stepperY_Right->forceStop();
                    paintGun_OFF();
                    Serial.println("Side 4 Pattern Painting ABORTED due to home command");
                    return;
                }
                delay(1);
            }
            
            paintGun_OFF();
            // currentY remains the same as we moved back to original position
        } else {
            Serial.printf("Side 4 Pattern: Sweep %d (-Y) with smooth paint gun control\\\n", i + 1);
            
            long finalY = currentY - sweepYDistance;
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)current_paint_y_speed / STEPS_PER_INCH_XYZ);
            
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
                
                if (checkForPauseCommand()) {
                    stepperY_Left->forceStop();
                    stepperY_Right->forceStop();
                    paintGun_OFF();
                    Serial.println("Side 4 Pattern Painting ABORTED due to home command");
                    return;
                }
                delay(1);
            }
            
            paintGun_OFF();
            currentY = finalY;
        }

        if (checkForPauseCommand()) {
            moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
            Serial.println("Side 4 Pattern Painting ABORTED due to home command");
            return;
        }

        // Perform X shift if it's not the last Y sweep
        if (i < num_y_sweeps - 1) {
            Serial.printf("Side 4 Pattern: Shift +X after sweep %d\\\\n", i + 1);
            currentX += shiftXDistance; // Shift in +X direction (ensure shiftXDistance is positive in settings for +X)
            moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED); // Use original paint_y_speed for X shift
            
            if (checkForPauseCommand()) {
                 moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
                 Serial.println("Side 4 Pattern Painting ABORTED due to home command during X shift");
                 return;
            }
        }
    }

    //! NEW: Perform additional movements at the end of the pattern for Side 4
    Serial.println("Side 4 Pattern: Starting end sequence movements.");

    //! 1. Paint gun OFF
    paintGun_OFF();
    Serial.println("Side 4 Pattern: Gun OFF for initial end sequence movements.");

    //! 2. Move -Y by sweepYDistance
    long endSeq_targetY1 = currentY - sweepYDistance; // Use the existing sweepYDistance for this side
    Serial.printf("Side 4 Pattern: Moving -Y by sweep distance to Y=%ld\n", endSeq_targetY1);
    moveToXYZ(currentX, paint_x_speed, endSeq_targetY1, paint_y_speed, zPos, DEFAULT_Z_SPEED); // Maintain painting Z
    currentY = endSeq_targetY1;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 4 Pattern Painting ABORTED during end sequence (move 1) due to home command");
        return;
    }

    //! 3. Move +2 inches in X
    long endSeq_targetX1 = currentX + (long)(1.0 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 4 Pattern: Moving +2in X to X=%ld\n", endSeq_targetX1);
    moveToXYZ(endSeq_targetX1, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED); // Maintain painting Z
    currentX = endSeq_targetX1;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 4 Pattern Painting ABORTED during end sequence (move 2) due to home command");
        return;
    }

    //! Set Servo Angle and Z height for final X pass
    myServo.setAngle(85);
    Serial.println("Servo set to: 85 degrees for final X pass on Side 4");
    long finalXPassZPos_Side4 = (long)(-1.75 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 4 Pattern: Setting Z to %.2f inches for final X pass\n", -1.75);

    //! 4-6. Move -23 inches in X with smooth paint gun control
    Serial.println("Side 4 Pattern: Starting -23in X sweep with smooth paint gun control.");
    long finalXSweepDistance = (long)(23.0 * STEPS_PER_INCH_XYZ);
    long endSeq_targetX2 = currentX - finalXSweepDistance;
    
    // Calculate smooth movement parameters
    float totalDistance = 23.0f; // Distance in inches
    float timeToStart = 0.25f * 60.0f / ((float)paint_x_speed / STEPS_PER_INCH_XYZ);
    float timeToStop = (totalDistance - 0.5f) * 60.0f / ((float)paint_x_speed / STEPS_PER_INCH_XYZ);
    
    // Start smooth movement with Z change
    moveToXYZ(endSeq_targetX2, paint_x_speed, currentY, paint_y_speed, finalXPassZPos_Side4, DEFAULT_Z_SPEED);
    
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
            Serial.println("Side 4 Pattern: Gun ON after 0.25in offset for final X sweep.");
        }
        
        if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= (timeToStop / 1000.0f)) {
            paintGun_OFF();
            paintGunDeactivated = true;
            Serial.println("Side 4 Pattern: Gun OFF, 0.25in before end of X sweep.");
        }
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            stepperZ->forceStop();
            paintGun_OFF();
            Serial.println("Side 4 Pattern Painting ABORTED during end sequence due to home command");
            return;
        }
        delay(1);
    }
    
    paintGun_OFF();
    currentX = endSeq_targetX2;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 4 Pattern Painting ABORTED during end sequence (move 3) due to home command");
        return;
    }
    Serial.println("Side 4 Pattern: End sequence movements completed.");

    //! Move Z to safe height before transitioning
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
    
    //! Move to position (3,3) before homing
    Serial.println("Moving to position (3,3,0) before homing...");
    long xHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long yHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long zHoming = 0;
    moveToXYZ(xHoming, DEFAULT_X_SPEED, yHoming, DEFAULT_Y_SPEED, zHoming, DEFAULT_Z_SPEED);
    Serial.println("Reached position (3,3,0).");

    //! Transition to Homing State
    Serial.println("Side 4 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState()); // Corrected state change call
    // No return needed as function is void
}