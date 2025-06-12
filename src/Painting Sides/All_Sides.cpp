#include <Arduino.h>
#include "motors/PaintingSides.h"
#include "../../include/web/Web_Dashboard_Commands.h"
#include "motors/ServoMotor.h"
#include "hardware/paintGun_Functions.h"
#include "hardware/pressurePot_Functions.h"
#include "motors/XYZ_Movements.h"
#include "utils/settings.h"
#include <FastAccelStepper.h>
#include "motors/Homing.h"
#include "motors/Rotation_Motor.h"
#include "system/StateMachine.h"
#include "system/GlobalState.h"    // For isPaused
#include <WebSocketsServer.h>     // For webSocket.loop()

extern ServoMotor myServo;
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
extern bool isPressurePot_ON;
extern FastAccelStepperEngine engine;
extern FastAccelStepper *rotationStepper;
// Function-based StateMachine - no extern needed
extern WebSocketsServer webSocket;    // For pause loop

// External references to immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

// Global variable definition for requested coats
int g_requestedCoats = 3; // Default to 3 coats
int g_interCoatDelaySeconds = 10; // Default 10 seconds delay

//* ************************************************************************
//* ********************** ALL SIDES PAINTING ************************
//* ************************************************************************
// This file handles the sequence of painting all sides of the piece in multiple runs
// Each run follows the same pattern (sides 4, 3, 2, 1) with a loading bar animation
// between runs to show progress during the delay time.

// Cleaning parameters
const float CLEANING_X_INCH = 0.0;
const float CLEANING_Y_INCH = 2.0;
const float CLEANING_Z_INCH = -1.5;
const unsigned int CLEANING_X_SPEED = 15000;
const unsigned int CLEANING_Y_SPEED = 15000;
const unsigned int CLEANING_Z_SPEED = 4000;

// Define the loading bar X position as a constant to ensure consistency
const float LOADING_BAR_X_START = 24.0f;
const float LOADING_BAR_X_END = 0.0f;

// Function to check for immediate commands during painting operations
bool checkForImmediateCommands() {
    // Process any pending WebSocket events to catch immediate commands
    webSocket.loop();
    
    // Check for immediate commands
    if (immediateCommandPending) {
        Serial.println("IMMEDIATE COMMAND detected during painting - aborting current operation");
        
        // Stop all motors immediately
        if (stepperX && stepperX->isRunning()) {
            stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
        }
        if (stepperY_Left && stepperY_Left->isRunning()) {
            stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
        }
        if (stepperY_Right && stepperY_Right->isRunning()) {
            stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
        }
        if (stepperZ && stepperZ->isRunning()) {
            stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
        }
        
        // Turn off paint gun for safety
        paintGun_OFF();
        
        return true; // Immediate command pending
    }
    
    // Also check for home command for backward compatibility
    if (checkForPauseCommand()) {
        return true;
    }
    
    return false; // No immediate commands
}

// Helper function to prepare for the next painting sequence
void _prepareForPaintingSequence() {
    myServo.setAngle(0);
    Serial.println("Reset servo angle to 0 degrees before painting sequence");
    
    paintGun_OFF();
    Serial.println("Ensured paint gun is off before starting painting sequence");
    
    delay(200);
}

// Helper function for a single painting sequence
// Returns true if completed, false if aborted by home command
bool _executeSinglePaintAllSidesSequence(const char* runLabel) {
    Serial.print("Starting All Sides Painting Sequence (");
    Serial.print(runLabel);
    Serial.println(")");
    
    _prepareForPaintingSequence();
    
    Serial.print("Z axis at 0 (assumed or handled by pre-clean). Starting painting. (");
    Serial.print(runLabel);
    Serial.println(")");
    
    //! STEP 1: Paint left side (Side 4)
    Serial.print("Starting Left Side (Side 4) ("); Serial.print(runLabel); Serial.println(")");
    while (isPaused) { webSocket.loop(); delay(100); }
    // Check for immediate commands before starting side painting
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", before left side due to immediate command)");
        return false;
    }
    paintSide4Pattern();
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after left side)");
        return false;
    }

    //! STEP 2: Paint back side (Side 3)
    Serial.print("Starting Back Side (Side 3) ("); Serial.print(runLabel); Serial.println(")");
    while (isPaused) { webSocket.loop(); delay(100); }
    // Check for immediate commands before starting side painting
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", before back side due to immediate command)");
        return false;
    }
    paintSide3Pattern();
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after back side)");
        return false;
    }

    //! STEP 3: Paint right side (Side 2)
    Serial.print("Starting Right Side (Side 2) ("); Serial.print(runLabel); Serial.println(")");
    while (isPaused) { webSocket.loop(); delay(100); }
    // Check for immediate commands before starting side painting
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", before right side due to immediate command)");
        return false;
    }
    paintSide2Pattern();
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after right side)");
        return false;
    }
    
    //! STEP 4: Paint front side (Side 1)
    Serial.print("Starting Front Side (Side 1) ("); Serial.print(runLabel); Serial.println(")");
    while (isPaused) { webSocket.loop(); delay(100); }
    // Check for immediate commands before starting side painting
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", before front side due to immediate command)");
        return false;
    }
    paintSide1Pattern();
    if (checkForImmediateCommands()) {
        Serial.print("All Sides Painting ABORTED ("); Serial.print(runLabel); Serial.println(", after front side)");
        return false;
    }

    //! Check if pressure pot needs to be turned on
    if (!isPressurePot_ON) {
        Serial.print("Pressure pot is off. Turning on and pressurizing for 1 second... (");
        Serial.print(runLabel);
        Serial.println(")");
        PressurePot_ON();
        
        unsigned long pressureStartTime = millis();
        while (millis() - pressureStartTime < 1000) {
            if (checkForImmediateCommands()) {
                Serial.print("All Sides Painting ABORTED (");
                Serial.print(runLabel);
                Serial.println(", during pressure pot pressurization due to immediate command)");
                return false;
            }
            delay(10);
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
    int totalCoats = g_requestedCoats;
    g_requestedCoats = 1; // Reset global for next time

    for (int coat = 1; coat <= totalCoats; ++coat) {
        char runLabel[10];
        snprintf(runLabel, sizeof(runLabel), "Run %d", coat);
        Serial.printf("Starting %s of %d\n", runLabel, totalCoats);

        if (!_executeSinglePaintAllSidesSequence(runLabel)) {
            Serial.printf("Painting %s aborted. Process terminated.\n", runLabel);
            return;
        }

        Serial.printf("%s finished.\n", runLabel);

        // If this was the last coat, skip the delay/loading bar
        if (coat == totalCoats) {
            break; 
        }

        //! ************************************************************************
        //! INTER-COAT DELAY AND LOADING BAR
        //! ************************************************************************
        Serial.println("Setting servo to 180 degrees for inter-coat movement.");
        myServo.setAngle(180);
        Serial.println("Servo set to 180 degrees during inter-coat delay.");
        
        Serial.println("Preparing for inter-coat delay: Moving X to loading bar start position.");
        long target_x_start_loading_bar_steps = (long)(LOADING_BAR_X_START * STEPS_PER_INCH_XYZ);
        stepperX->setSpeedInHz(DEFAULT_X_SPEED);
        stepperX->setAcceleration(DEFAULT_X_ACCEL);
        stepperX->moveTo(target_x_start_loading_bar_steps);
        
        while (stepperX->isRunning()) {
            if (checkForImmediateCommands()) {
                Serial.printf("Immediate command during move to loading bar start before coat %d. Process terminated.\n", coat + 1);
                stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                return;
            }
            delay(1);
        }
        Serial.println("Reached loading bar start position.");

        Serial.println("Starting X-axis loading bar movement for delay.");
        long target_x_end_loading_bar_steps = (long)(LOADING_BAR_X_END * STEPS_PER_INCH_XYZ);
        float duration_seconds = (float)g_interCoatDelaySeconds;
        long current_x_actual_start_steps = stepperX->getCurrentPosition();

        if (duration_seconds < 0.1f) { 
            Serial.printf("Loading bar (%d) fallback: Simple timed wait for %d s.\n", coat, g_interCoatDelaySeconds);
            unsigned long simpleDelayStartTime = millis();
            while (millis() - simpleDelayStartTime < (unsigned long)g_interCoatDelaySeconds * 1000) {
                if (checkForImmediateCommands()) {
                    Serial.printf("Immediate command during fallback wait (%d). Process terminated.\n", coat);
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
                if (checkForImmediateCommands()) {
                    Serial.printf("Immediate command during loading bar (%d). Process terminated.\n", coat);
                    stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
                    return;
                }
                delay(1);
            }
            Serial.printf("Loading bar movement (%d) complete.\n", coat);
        }

        Serial.printf("Finished inter-coat delay for coat %d. Ready for coat %d.\n", coat, coat + 1);
    }

    Serial.println("All Sides Painting Process Fully Completed.");

    //! ************************************************************************
    //! PRESSURE POT DEPRESSURIZATION
    //! ************************************************************************
    Serial.println("Depressurizing pressure pot after painting completion...");
    PressurePot_OFF();
    Serial.println("Pressure pot depressurized.");

    //! ************************************************************************
    //! HOMING SEQUENCE
    //! ************************************************************************
    Serial.println("Initiating homing sequence using proper homing state...");
    
    // Change to homing state - this will properly home all axes including rotation
    changeState(MachineState::HOMING);
    Serial.println("Changed to homing state for proper axis positioning.");
} 