#include <Arduino.h>
#include "motors/PaintingSides.h"
#include "../../include/web/Web_Dashboard_Commands.h" // For checkForHomeCommand
#include "motors/ServoMotor.h" // Added for cleaning burst
#include "hardware/paintGun_Functions.h" // Added for cleaning burst
#include "hardware/pressurePot_Functions.h" // Added for pressure pot check
#include "motors/XYZ_Movements.h" // Added for cleaning movement
#include "utils/settings.h"     // Added for STEPS_PER_INCH_XYZ and speeds
#include <FastAccelStepper.h>    // Added for stepper extern declarations
#include "motors/Homing.h"      // For Homing class and homeAllAxes()
#include "motors/Rotation_Motor.h" // For rotation motor reset

extern ServoMotor myServo; // Added for cleaning burst
extern FastAccelStepper *stepperX;      // Added for Z move
extern FastAccelStepper *stepperY_Left; // Added for Z move
extern FastAccelStepper *stepperY_Right; // Needed for Homing class constructor
extern FastAccelStepper *stepperZ;      // Added for Z move
extern bool isPressurePot_ON; // Added for pressure pot check
extern FastAccelStepperEngine engine; // Needed for Homing class constructor

// Global variable definition for requested coats
int g_requestedCoats = 1; // Default to 1 coat
int g_interCoatDelaySeconds = 10; // ADDED: Default 10 seconds delay

//* ************************************************************************
//* ********************** ALL SIDES PAINTING ************************
//* ************************************************************************
// This file handles the sequence of painting all sides of the piece in three runs
// Each run follows the same pattern (sides 4, 3, 2) with a loading bar animation
// between runs to show progress during the delay time.
// This file handles the sequence of painting all sides of the piece in a single run
// painting sides 4, 3, and 2.

// Cleaning parameters
const float CLEANING_X_INCH = 0.0;
const float CLEANING_Y_INCH = 2.0;
const float CLEANING_Z_INCH = -1.5;
const unsigned int CLEANING_X_SPEED = 15000;
const unsigned int CLEANING_Y_SPEED = 15000;
const unsigned int CLEANING_Z_SPEED = 4000;

// const unsigned long ALL_SIDES_REPEAT_DELAY_MS = 10 * 1000; // REMOVED: Replaced by g_interCoatDelaySeconds

// Define the loading bar X position as a constant to ensure consistency
const float LOADING_BAR_X_START = 24.0f;
const float LOADING_BAR_X_END = 0.0f;

// Helper function to prepare for the next painting sequence
// This only does minimal preparation to ensure consistent painting
void _prepareForPaintingSequence() {
    // Reset servo to a safe initial position (each side sets its own angle)
    myServo.setAngle(0);
    Serial.println("Reset servo angle to 0 degrees before painting sequence");
    
    // Ensure paint gun is off before starting a sequence
    paintGun_OFF();
    Serial.println("Ensured paint gun is off before starting painting sequence");
    
    // Small delay for safety
    delay(200);
}

// Helper function for a single painting sequence
// Returns true if completed, false if aborted by home command
bool _executeSinglePaintAllSidesSequence(const char* runLabel) {
    Serial.print("Starting All Sides Painting Sequence (");
    Serial.print(runLabel);
    Serial.println(")");
    
    // Minimal preparation for painting sequence
    _prepareForPaintingSequence();
    
    // The PaintingState now handles a dedicated pre-paint clean using CleaningState.

    Serial.print("Z axis at 0 (assumed or handled by pre-clean). Starting painting. (");
    Serial.print(runLabel);
    Serial.println(")");
    // Note: Servo angle should be set by individual side patterns as needed.
    
    //! STEP 1: Paint left side (Side 4)
    Serial.print("Starting Left Side (Side 4) ("); Serial.print(runLabel); Serial.println(")");
    paintSide4Pattern();
    if (checkForHomeCommand()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after left side)");
        return false;
    }

    //! STEP 2: Paint back side (Side 3)
    Serial.print("Starting Back Side (Side 3) ("); Serial.print(runLabel); Serial.println(")");
    paintSide3Pattern();
    if (checkForHomeCommand()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after back side)");
        return false;
    }

    //! STEP 3: Paint right side (Side 2)
    Serial.print("Starting Right Side (Side 2) ("); Serial.print(runLabel); Serial.println(")");
    paintSide2Pattern();
    if (checkForHomeCommand()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after right side)");
        return false;
    }
    
    //! STEP 4: Paint front side (Side 1)
    Serial.print("Starting Front Side (Side 1) ("); Serial.print(runLabel); Serial.println(")");
    paintSide1Pattern();
    if (checkForHomeCommand()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after front side)");
        return false;
    }

    //! Check if pressure pot needs to be turned on
    if (!isPressurePot_ON) {
        Serial.print("Pressure pot is off. Turning on and pressurizing for 1 second... (");
        Serial.print(runLabel);
        Serial.println(")");
        PressurePot_ON();
        
        // Replace blocking delay with non-blocking version that checks for home command
        unsigned long pressureStartTime = millis();
        while (millis() - pressureStartTime < 1000) { // 1 second wait
            if (checkForHomeCommand()) {
                Serial.print("All Sides Painting ABORTED (");
                Serial.print(runLabel);
                Serial.println(", during pressure pot pressurization)");
                return false;
            }
            delay(10); // Short delay between checks
        }
        
        Serial.print("Pressurization complete. (");
        Serial.print(runLabel);
        Serial.println(")");
    } else {
        Serial.print("Pressure pot already on. (");
        Serial.print(runLabel);
        Serial.println(")");
    }

    Serial.print("All Sides Painting Sequence (");
    Serial.print(runLabel);
    Serial.println(") Completed.");
    
    return true;
} 

// Main function to be called externally
void paintAllSides() {
    Serial.printf("Initiating All Sides Painting Process for %d coat(s).\n", g_requestedCoats);
    int totalCoats = g_requestedCoats; // Capture the requested coats
    g_requestedCoats = 1; // Reset global for next time, unless set again by command

    for (int coat = 1; coat <= totalCoats; ++coat) {
        char runLabel[10];
        snprintf(runLabel, sizeof(runLabel), "Run %d", coat);
        Serial.printf("Starting %s of %d\n", runLabel, totalCoats);

        if (!_executeSinglePaintAllSidesSequence(runLabel)) {
            Serial.printf("Painting %s aborted. Process terminated.\n", runLabel);
            return; // Abort if the run was cancelled
        }

        Serial.printf("%s finished.\n", runLabel);

        // If this was the last coat, skip the delay/loading bar
        if (coat == totalCoats) {
            break; 
        }

        // --- Inter-coat Delay and Loading Bar --- 
        Serial.println("Preparing for inter-coat delay: Moving X to loading bar start position.");
        long target_x_start_loading_bar_steps = (long)(LOADING_BAR_X_START * STEPS_PER_INCH_XYZ);
        stepperX->setSpeedInHz(DEFAULT_X_SPEED); // Use standard speed for this move
        stepperX->setAcceleration(DEFAULT_X_ACCEL);
        stepperX->moveTo(target_x_start_loading_bar_steps);
        
        while (stepperX->isRunning()) {
            if (checkForHomeCommand()) {
                Serial.printf("Home command during move to loading bar start before coat %d. Process terminated.\n", coat + 1);
                stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                return;
            }
            delay(1);
        }
        Serial.println("Reached loading bar start position.");

        // --- START: Custom servo and X-axis movement during inter-coat delay ---
        Serial.println("Inter-coat: Moving X to 5 inches from home.");
        long five_inches_from_home_steps = (long)(5.0f * STEPS_PER_INCH_XYZ);
        stepperX->moveTo(five_inches_from_home_steps);
        while (stepperX->isRunning()) {
            if (checkForHomeCommand()) {
                Serial.printf("Home command during X move to 5in before coat %d. Process terminated.\n", coat + 1);
                stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                return;
            }
            delay(1);
        }

        Serial.println("Inter-coat: Moving X to home (0 inches).");
        stepperX->moveTo(0); // Move to 0 (home)
        while (stepperX->isRunning()) {
            if (checkForHomeCommand()) {
                Serial.printf("Home command during X move to 0in before coat %d. Process terminated.\n", coat + 1);
                stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                return;
            }
            delay(1);
        }

        Serial.println("Inter-coat: Setting servo to 180 degrees.");
        myServo.setAngle(180);
        // --- END: Custom servo and X-axis movement during inter-coat delay ---

        Serial.println("Starting X-axis loading bar movement for delay.");
        long target_x_end_loading_bar_steps = (long)(LOADING_BAR_X_END * STEPS_PER_INCH_XYZ);
        float duration_seconds = (float)g_interCoatDelaySeconds; // NEW
        long current_x_actual_start_steps = stepperX->getCurrentPosition(); // Should be LOADING_BAR_X_START

        if (duration_seconds < 0.1f) { 
            Serial.printf("Loading bar (%d) fallback: Simple timed wait for %d s.\n", coat, g_interCoatDelaySeconds);
            unsigned long simpleDelayStartTime = millis();
            while (millis() - simpleDelayStartTime < (unsigned long)g_interCoatDelaySeconds * 1000) { // NEW
                if (checkForHomeCommand()) {
                    Serial.printf("Home command during fallback wait (%d). Process terminated.\n", coat);
                    return;
                }
                delay(10); 
            }
        } else {
            long distance_steps = abs(target_x_end_loading_bar_steps - current_x_actual_start_steps);
            float calculated_speed_hz = (distance_steps > 0 && duration_seconds > 0) ? ((float)distance_steps / duration_seconds) : 1.0f;
            unsigned int speed_to_set_hz = (unsigned int)max(1.0f, calculated_speed_hz);

            Serial.printf("Loading Bar (%d): Moving X from %.2f to %.2f over %.2f s (Speed: %u Hz).\n",
                          coat, (float)current_x_actual_start_steps / STEPS_PER_INCH_XYZ, LOADING_BAR_X_END, duration_seconds, speed_to_set_hz);

            stepperX->setSpeedInHz(speed_to_set_hz);
            stepperX->setAcceleration(DEFAULT_X_ACCEL); 
            stepperX->moveTo(target_x_end_loading_bar_steps);

            while (stepperX->isRunning()) {
                if (checkForHomeCommand()) {
                    Serial.printf("Home command during loading bar (%d). Process terminated.\n", coat);
                    stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                    return;
                }
                delay(1);
            }
            Serial.printf("Loading bar movement (%d) complete.\n", coat);
        }
        
        // At the end of the delay (and loading bar movement), X is at LOADING_BAR_X_END.
        // For the next coat, the painting sequence will begin.
        // The _executeSinglePaintAllSidesSequence might require X to be at a specific starting point or 0.
        // For now, we assume the individual paintSideXPattern functions handle their initial positioning from wherever X is.
        // If they need X at 0,0, then a move to 0,0 (or at least X=0) should happen before the next call to _executeSinglePaintAllSidesSequence.
        // The current _prepareForPaintingSequence in _executeSinglePaintAllSidesSequence does not move X/Y/Z.
        // This is acceptable for now, individual patterns should manage their start.

        Serial.printf("Finished inter-coat delay for coat %d. Ready for coat %d.\n", coat, coat + 1);
    }

    Serial.println("All Sides Painting Process Fully Completed.");
} 