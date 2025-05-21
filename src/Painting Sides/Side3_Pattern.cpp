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
    long final_sweep_paint_x_speed_side3 = (long)(paint_x_speed * 0.5f);

    // First sweep: X- direction
    Serial.println("Side 3 Pattern: First sweep X-");
    paintGun_ON();
    currentX -= sweepX_steps;
    moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
    paintGun_OFF();

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    // First shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Second sweep: X+ direction
    Serial.println("Side 3 Pattern: Second sweep X+");
    paintGun_ON();
    currentX += sweepX_steps;
    moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
    paintGun_OFF();

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    // Second shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Third sweep: X- direction
    Serial.println("Side 3 Pattern: Third sweep X-");
    paintGun_ON();
    currentX -= sweepX_steps;
    moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
    paintGun_OFF();

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    // Third shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-");
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fourth sweep: X+ direction
    Serial.println("Side 3 Pattern: Fourth sweep X+");
    paintGun_ON();
    currentX += sweepX_steps;
    moveToXYZ(currentX, paint_x_speed, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
    paintGun_OFF();

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    // Fourth shift: Y- direction
    Serial.println("Side 3 Pattern: Shift Y-"); // Should this be after a home check? Assuming not based on pattern.
    currentY -= shiftY_steps;
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);

    // Fifth sweep: X- direction (Final X painting movement)
    Serial.println("Side 3 Pattern: Fifth sweep X-");
    Serial.printf("Side 3 Pattern: Applying 75%% speed for final X sweep: %ld\n", final_sweep_paint_x_speed_side3);
    paintGun_ON();
    currentX -= sweepX_steps;
    moveToXYZ(currentX, final_sweep_paint_x_speed_side3, currentY, paint_y_speed, zPos, DEFAULT_Z_SPEED);
    paintGun_OFF();

    if (checkForHomeCommand()) {
        moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);
        Serial.println("Side 3 Pattern Painting ABORTED due to home command");
        return;
    }

    //! STEP 8: Raise to safe Z height
    moveToXYZ(currentX, DEFAULT_X_SPEED, currentY, DEFAULT_Y_SPEED, sideZPos, DEFAULT_Z_SPEED);

    //! Move to position (3,3) before homing
    Serial.println("Moving to position (3,3,0) before homing...");
    long xHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long yHoming = (long)(3.0 * STEPS_PER_INCH_XYZ);
    long zHoming = 0;
    moveToXYZ(xHoming, DEFAULT_X_SPEED, yHoming, DEFAULT_Y_SPEED, zHoming, DEFAULT_Z_SPEED);
    Serial.println("Reached position (3,3,0).");

    //! Transition to Homing State
    Serial.println("Side 3 painting complete. Transitioning to Homing State...");
    stateMachine->changeState(stateMachine->getHomingState()); // Corrected state change call
    // No return needed as function is void
} 