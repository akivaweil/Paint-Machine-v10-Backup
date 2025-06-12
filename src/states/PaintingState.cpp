#include "states/PaintingState.h"
#include <Arduino.h>
#include "motors/PaintingSides.h" // Include header for paintAllSides
#include "system/StateMachine.h"  // Include header for StateMachine access
// #include "motors/Homing.h"        // REMOVE: Homing will be handled by HomingState
#include <FastAccelStepper.h>      // Include for stepper access
#include "storage/painting_settings.h"
#include "motors/Rotation_Motor.h"
// #include "system/machine_state.h" // No longer needed
#include "hardware/paintGun_Functions.h" // Added include for paintGun_OFF
#include "motors/XYZ_Movements.h"      // ADDED: For moveToXYZ
#include "utils/settings.h"            // ADDED: For default speeds
#include "system/GlobalState.h"        // ADDED: For isPaused global variable

// Define necessary variables or includes specific to PaintingState if known
// #include "settings.h"
// #include "XYZ_Movements.h"

// Function-based StateMachine - no extern needed

// Need access to the global stepper instances and engine
extern FastAccelStepperEngine engine;
extern FastAccelStepper* stepperX;
extern FastAccelStepper* stepperY_Left;
extern FastAccelStepper* stepperY_Right;
extern FastAccelStepper* stepperZ;

// External references to immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

//* ************************************************************************
//* ************************* PAINTING STATE ***************************
//* ************************************************************************

PaintingState::PaintingState() : currentStep(PS_IDLE) {
    // Constructor implementation
}

void PaintingState::enter() {
    Serial.print("PaintingState: enter() called. CurrentStep BEFORE logic: ");
    Serial.println(currentStep); // Log its value *before* the if. Add specific name if possible later.
    
    // Clear any lingering pause state from previous cycles
    isPaused = false;
    Serial.println("PaintingState: Cleared pause state for new painting cycle");
    
    // Check if we are in the special "Paint All Sides" transition
    // Check if transitioning to paint all sides (simplified for function-based approach)
    if (true) { // Always execute painting logic
        Serial.println("PaintingState: Detected 'Paint All Sides' transition. Starting 'All Sides' directly without cleaning.");
        currentStep = PS_PERFORM_ALL_SIDES_PAINTING; // Set step to perform all sides painting
        // Flag cleared (function-based approach handles this differently)
    } else if (currentStep == PS_IDLE) {
        Serial.println("PaintingState: enter() - Normal entry. Starting 'All Sides' directly without cleaning.");
        currentStep = PS_PERFORM_ALL_SIDES_PAINTING; // Go directly to painting
    } else {
        Serial.print("PaintingState: enter() - CurrentStep is not IDLE. Preserving currentStep: ");
        Serial.println(currentStep); // Log its value if preserved.
    }
    Serial.print("PaintingState: enter() finished. CurrentStep AFTER logic: ");
    Serial.println(currentStep);
    // Update will handle the rest based on currentStep
}

void PaintingState::update() {
    // Check for immediate commands that should interrupt painting
    if (immediateCommandPending) {
        Serial.println("PaintingState: Immediate command detected - interrupting painting process");
        
        // Stop any running motors immediately
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
        
        // Let the main loop handle the immediate command
        return;
    }
    
    // Declare variables outside of switch statement
    long xPos, yPos, zPos;
    
    switch (currentStep) {
        case PS_PERFORM_ALL_SIDES_PAINTING:
            Serial.println("PaintingState: Starting all sides painting routine.");
            paintAllSides(); // This is assumed to be a blocking call
            Serial.println("PaintingState: All Sides Painting routine finished.");
            currentStep = PS_MOVE_TO_POSITION_BEFORE_HOMING;
            break;

        case PS_MOVE_TO_POSITION_BEFORE_HOMING:
            Serial.println("PaintingState: Moving to position (1,1,0) before Homing.");
            // Convert inches to steps
            xPos = (long)(1.0 * STEPS_PER_INCH_XYZ);
            yPos = (long)(1.0 * STEPS_PER_INCH_XYZ);
            zPos = 0;
            
            moveToXYZ(xPos, DEFAULT_X_SPEED, yPos, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED); // Blocking
            Serial.println("PaintingState: Reached position (1,1,0).");
            currentStep = PS_REQUEST_HOMING;
            // Fall through intentionally to PS_REQUEST_HOMING
        
        case PS_REQUEST_HOMING:
            Serial.println("PaintingState: Sequence complete. Requesting Homing State.");
            changeState(MachineState::HOMING);
            currentStep = PS_IDLE; // Reset for next entry into PaintingState
            break;
        
        case PS_IDLE:
            // Waiting for a new painting command (which would call enter() and reset currentStep)
            break;
    }
}

void PaintingState::exit() {
    Serial.println("Exiting Painting State (All Sides)");
    // setMachineState(MachineState::UNKNOWN); // REMOVED
    paintGun_OFF(); 
    // currentStep = PS_IDLE; // REMOVED: Reset step on exit - This was causing the loop.
                            // currentStep is now reset at the end of the painting sequence within update().
}

const char* PaintingState::getName() const {
    return "PAINTING";
}

//* ************************************************************************
//* ************************** PAINTING STATE ****************************
//* ************************************************************************ 