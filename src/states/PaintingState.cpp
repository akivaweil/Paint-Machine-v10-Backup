#include "states/PaintingState.h"
#include <Arduino.h>
#include "motors/PaintingSides.h" // Include header for paintAllSides
#include "system/StateMachine.h"  // Include header for StateMachine access
// #include "motors/Homing.h"        // REMOVE: Homing will be handled by HomingState
#include <FastAccelStepper.h>      // Include for stepper access
#include "persistence/PaintingSettings.h"
#include "motors/Rotation_Motor.h"
// #include "system/machine_state.h" // No longer needed
#include "hardware/paintGun_Functions.h" // Added include for paintGun_OFF
#include "motors/XYZ_Movements.h"      // ADDED: For moveToXYZ
#include "utils/settings.h"            // ADDED: For default speeds

// Define necessary variables or includes specific to PaintingState if known
// #include "settings.h"
// #include "XYZ_Movements.h"

extern StateMachine *stateMachine; // Access the global state machine instance

// Need access to the global stepper instances and engine
extern FastAccelStepperEngine engine;
extern FastAccelStepper* stepperX;
extern FastAccelStepper* stepperY_Left;
extern FastAccelStepper* stepperY_Right;
extern FastAccelStepper* stepperZ;

//* ************************************************************************
//* ************************* PAINTING STATE ***************************
//* ************************************************************************

PaintingState::PaintingState() : currentStep(PS_IDLE) {
    // Constructor implementation
}

void PaintingState::enter() {
    Serial.print("PaintingState: enter() called. CurrentStep BEFORE logic: ");
    Serial.println(currentStep); // Log its value *before* the if. Add specific name if possible later.
    
    // Check if we are in the special "Paint All Sides" transition
    if (stateMachine && stateMachine->isTransitioningToPaintAllSides()) {
        Serial.println("PaintingState: Detected 'Paint All Sides' transition. Skipping pre-paint clean request and starting 'All Sides' directly.");
        currentStep = PS_PERFORM_ALL_SIDES_PAINTING; // Set step to perform all sides painting
        stateMachine->clearTransitioningToPaintAllSidesFlag(); // Clear the flag as it has been handled
    } else if (currentStep == PS_IDLE) {
        Serial.println("PaintingState: enter() - Normal entry or not 'Paint All Sides' specific transition. Setting to PS_REQUEST_PRE_PAINT_CLEAN.");
        currentStep = PS_REQUEST_PRE_PAINT_CLEAN; // Start the normal sequence (request clean)
    } else {
        Serial.print("PaintingState: enter() - CurrentStep is not IDLE and not a specific 'Paint All Sides' transition. Preserving currentStep: ");
        Serial.println(currentStep); // Log its value if preserved.
    }
    Serial.print("PaintingState: enter() finished. CurrentStep AFTER logic: ");
    Serial.println(currentStep);
    // Update will handle the rest based on currentStep
}

void PaintingState::update() {
    // Declare variables outside of switch statement
    long xPos, yPos, zPos;
    
    switch (currentStep) {
        case PS_REQUEST_PRE_PAINT_CLEAN:
            Serial.println("PaintingState: Requesting short pre-paint clean.");
            if (stateMachine && stateMachine->getCleaningState()) {
                static_cast<CleaningState*>(stateMachine->getCleaningState())->setShortMode(true);
                stateMachine->setNextStateOverride(this); // Return to PaintingState
                currentStep = PS_PERFORM_ALL_SIDES_PAINTING; // Set next step for when we return
                stateMachine->changeState(stateMachine->getCleaningState());
                // PaintingState is no longer active until CleaningState returns.
            } else {
                Serial.println("ERROR: PaintingState - Cannot initiate cleaning, SM or CleaningState not available.");
                currentStep = PS_REQUEST_HOMING; // Skip to homing on error
            }
            break;

        case PS_PERFORM_ALL_SIDES_PAINTING:
            Serial.println("PaintingState: Pre-paint clean complete. Starting all sides painting routine.");
            paintAllSides(); // This is assumed to be a blocking call
            Serial.println("PaintingState: All Sides Painting routine finished.");
            currentStep = PS_MOVE_TO_POSITION_BEFORE_HOMING;
            break;

        case PS_MOVE_TO_POSITION_BEFORE_HOMING:
            Serial.println("PaintingState: Moving to position (3,3,0) before Homing.");
            // Convert inches to steps
            xPos = (long)(3.0 * STEPS_PER_INCH_XYZ);
            yPos = (long)(3.0 * STEPS_PER_INCH_XYZ);
            zPos = 0;
            
            moveToXYZ(xPos, DEFAULT_X_SPEED, yPos, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED); // Blocking
            Serial.println("PaintingState: Reached position (3,3,0).");
            currentStep = PS_REQUEST_HOMING;
            // Fall through intentionally to PS_REQUEST_HOMING
        
        case PS_REQUEST_HOMING:
            Serial.println("PaintingState: Sequence complete. Requesting Homing State.");
            if (stateMachine && stateMachine->getHomingState()) {
                stateMachine->changeState(stateMachine->getHomingState());
            } else {
                Serial.println("ERROR: PaintingState - Cannot transition to HomingState.");
                if (stateMachine && stateMachine->getIdleState()) {
                   stateMachine->changeState(stateMachine->getIdleState());
                }
            }
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