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
            Serial.printf("Side 4 Pattern: Sweep %d (start at top, paint -Y) with smooth paint gun control\\\n", i + 1);
            
            if (i == 0) { // First sweep only
                // First sweep - move to top position only
                Serial.printf("Side 4 Pattern: Moving to top position Y=%ld without painting at fast speed\n", startY_steps);
                moveToXYZ(currentX, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
                
                if (checkForPauseCommand()) {
                    Serial.println("Side 4 Pattern Painting ABORTED due to home command");
                    return;
                }
            }
            // For subsequent sweeps, X shift and Y top move are combined before this loop
            
            // Now paint down to bottom position - same as -Y sweeps
            long finalY = startY_steps - sweepYDistance;
            Serial.printf("Side 4 Pattern: Painting while moving -Y down to Y=%ld\n", finalY);
            
            // Calculate smooth movement parameters for -Y movement
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * STEPS_PER_INCH_XYZ / (float)current_paint_y_speed;
            float timeToStop = (totalDistance - 0.5f) * STEPS_PER_INCH_XYZ / (float)current_paint_y_speed;
            
            // Start smooth movement for BOTH Y motors down to bottom
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
                
                if (!paintGunActivated && elapsedSeconds >= timeToStart) {
                    paintGun_ON();
                    paintGunActivated = true;
                }
                
                if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= timeToStop) {
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
        } else {
            Serial.printf("Side 4 Pattern: Sweep %d (start at top, paint -Y) with smooth paint gun control\\\n", i + 1);
            
            if (i == 1) { // Second sweep - first -Y sweep - needs to move to top first
                // Move to top position first (no paint) - use fast speed
                Serial.printf("Side 4 Pattern: Moving to top position Y=%ld without painting at fast speed\n", startY_steps);
                moveToXYZ(currentX, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
                
                if (checkForPauseCommand()) {
                    Serial.println("Side 4 Pattern Painting ABORTED due to home command");
                    return;
                }
            }
            // For other -Y sweeps, X shift and Y top move are combined before this loop
            
            // Now paint down to bottom position - same as +Y sweeps
            long finalY = startY_steps - sweepYDistance;
            Serial.printf("Side 4 Pattern: Painting while moving -Y down to Y=%ld\n", finalY);
            
            float totalDistance = (float)sweepYDistance / STEPS_PER_INCH_XYZ;
            float timeToStart = 0.25f * STEPS_PER_INCH_XYZ / (float)current_paint_y_speed;
            float timeToStop = (totalDistance - 0.5f) * STEPS_PER_INCH_XYZ / (float)current_paint_y_speed;
            
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
                
                if (!paintGunActivated && elapsedSeconds >= timeToStart) {
                    paintGun_ON();
                    paintGunActivated = true;
                }
                
                if (paintGunActivated && !paintGunDeactivated && elapsedSeconds >= timeToStop) {
                    paintGun_OFF();
                    paintGunDeactivated = true;
                }
                
                // Process WebSocket events frequently during movement
                processWebSocketEventsFrequently();
                
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

        // Perform combined X shift and Y top move if it's not the last Y sweep
        if (i < num_y_sweeps - 1) {
            Serial.printf("Side 4 Pattern: Combined shift +X and move to top Y after sweep %d at fast speed\\\\n", i + 1);
            currentX += shiftXDistance; // Shift in +X direction (ensure shiftXDistance is positive in settings for +X)
            moveToXYZ(currentX, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED); // Move to next X AND top Y simultaneously
            
            if (checkForPauseCommand()) {
                 moveToXYZ(currentX, DEFAULT_X_SPEED, startY_steps, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
                 Serial.println("Side 4 Pattern Painting ABORTED due to home command during combined X shift and Y move");
                 return;
            }
        }
    }

    //! NEW: Perform additional movements at the end of the pattern for Side 4
    Serial.println("Side 4 Pattern: Starting end sequence movements.");

    //! 1. Paint gun OFF
    paintGun_OFF();
    Serial.println("Side 4 Pattern: Gun OFF for initial end sequence movements.");

    //! 2. Move +1 inches in X - use fast speed
    long endSeq_targetX1 = currentX + (long)(1.0 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 4 Pattern: Moving +1in X to X=%ld at fast speed\n", endSeq_targetX1);
    moveToXYZ(endSeq_targetX1, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED); // Maintain painting Z
    currentX = endSeq_targetX1;

    if (checkForPauseCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 4 Pattern Painting ABORTED during end sequence due to home command");
        return;
    }

    //! Set Servo Angle and Z height for final X pass
    myServo.setAngle(85);
    Serial.println("Servo set to: 85 degrees for final X pass on Side 4");
    long finalXPassZPos_Side4 = (long)(-1.75 * STEPS_PER_INCH_XYZ);
    Serial.printf("Side 4 Pattern: Setting Z to %.2f inches for final X pass\n", -1.75);

    //! 4-6. Move -23 inches in X with continuous motion and position-based paint gun control
    Serial.println("Side 4 Pattern: Starting -23in X sweep with continuous motion paint gun control.");
    long finalXSweepDistance = (long)(23.0 * STEPS_PER_INCH_XYZ);
    long endSeq_targetX2 = currentX - finalXSweepDistance;
    long paintStartX = currentX - (long)(0.25f * STEPS_PER_INCH_XYZ);
    long paintStopX = endSeq_targetX2 + (long)(0.5f * STEPS_PER_INCH_XYZ);
    
    Serial.printf("Side 4 Final X: Start=%ld, PaintStart=%ld, PaintStop=%ld, End=%ld\n", 
                  currentX, paintStartX, paintStopX, endSeq_targetX2);
    
    // First move Z to the final X pass Z height
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, finalXPassZPos_Side4, DEFAULT_Z_SPEED);
    
    // Start continuous movement from currentX to endSeq_targetX2
    stepperX->moveTo(endSeq_targetX2);
    stepperX->setSpeedInHz(paint_x_speed);
    
    bool paintGunOn = false;
    
    // Monitor movement and control paint gun based on position
    while (stepperX->isRunning()) {
        long currentXPos = stepperX->getCurrentPosition();
        
        // Turn paint gun ON when reaching paint start position (moving in -X direction)
        if (!paintGunOn && currentXPos <= paintStartX) {
            paintGun_ON();
            paintGunOn = true;
            Serial.println("Side 4 Pattern: Paint gun ON during final X movement");
        }
        
        // Turn paint gun OFF when reaching paint stop position (moving in -X direction)
        if (paintGunOn && currentXPos <= paintStopX) {
            paintGun_OFF();
            paintGunOn = false;
            Serial.println("Side 4 Pattern: Paint gun OFF during final X movement");
        }
        
        // Process WebSocket events frequently during movement
        processWebSocketEventsFrequently();
        
        if (checkForPauseCommand()) {
            stepperX->forceStop();
            paintGun_OFF();
            Serial.println("Side 4 Pattern Painting ABORTED during final X sweep due to home command");
            return;
        }
        
        // Small delay to prevent excessive CPU usage
        delay(1);
    }
    
    // Ensure paint gun is off at the end
    if (paintGunOn) {
        paintGun_OFF();
        Serial.println("Side 4 Pattern: Paint gun OFF - final X movement complete");
    }
    
    paintGun_OFF();

    //! Move to position (1,1,0) before homing
    moveToPositionOneOneBeforeHoming();

    //! Transition to Homing State
    Serial.println("Side 4 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState());
}