#pragma once

// #include <Bounce2.h> // No longer needed here, moved to GlobalDebouncers.h
#include <FastAccelStepper.h>
#include "states/State.h" // Assuming a base State class exists
#include "utils/settings.h" // For grid dimensions, pins etc.
#include "motors/ServoMotor.h" // Added include for ServoMotor
#include "hardware/GlobalDebouncers.h" // Include for g_pnpCycleSensorDebouncer

// Assuming StateMachine is needed for transitions
class StateMachine; 
extern StateMachine* stateMachine; 

// Forward declaration for ServoMotor is no longer needed
// class ServoMotor; 

class PnPState : public State {
public:
    PnPState(); // Added parameterless constructor declaration back
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;

    // PnP specific methods
    // void moveToTarget(PnPAction action); // REMOVED - Not implemented/used and PnPAction undefined

private:
    // Grid configuration
    float gridPositionsX[GRID_ROWS * GRID_COLS];
    float gridPositionsY[GRID_ROWS * GRID_COLS];

    // Position tracking
    int currentPnPGridPosition;
    bool pnpCycleIsComplete;
    long pickLocationX_steps; // Pick location X in steps
    long pickLocationY_steps; // Pick location Y in steps

    // Cycle sensor (direct read)
    // Bounce pnpCycleSensorDebouncer; // REMOVED: Now using global g_pnpCycleSensorDebouncer
    
    // Cycle timeout
    unsigned long lastCycleTime;  // Timestamp of the last cycle completion
    unsigned long cycleTimeoutMs; // Timeout duration for cycles
    unsigned long cycleStartTimeMs; // Start time for the current cycle

    // Simplified state tracking within PnPState
    // 0: Moving to initial pick location
    // 1: Waiting at Pick Location
    // 2: Processing a PnP cycle (blocking Pick->Move->Place)
    // 3: Moving back to Pick Location (non-blocking)
    // 4: PnP Complete, ready for homing/exit transition
    int pnp_step; 

    // Flag to signal that homing is needed after PnP completion
    bool homingNeededAfterPnP; 

    // Private helper methods
    void calculateGridPositions();
    void initializeHardware();
    void moveToPickLocation(bool initialMove = false); // Moves to pick location (non-blocking)
    void process_single_pnp_cycle(); 
    void resetStateAndReturnToIdle(); // Reset state and return to idle

    // Stepper references (assuming they are globally accessible or passed somehow)
    // If they are global like in pnp_State.cpp, we might not need them as members.
    // extern FastAccelStepperEngine engine; // Example if needed
    // extern FastAccelStepper *stepperX, *stepperY_Left, *stepperY_Right; // Example
};

// Function declarations previously in pnp_State.cpp (if needed globally)
// bool isPnpHomingNeeded(); // Might be better as a method or handled internally
// void clearPnpHomingNeededFlag(); // Might be better as a method or handled internally
// bool isPnpActive(); // Can be inferred from stateMachine->currentState being PnPState 