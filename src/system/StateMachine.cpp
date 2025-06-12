#include <Arduino.h>
#include "system/StateMachine.h"
#include "motors/Homing.h"
#include <WebSocketsServer.h>
#include <FastAccelStepper.h>
#include "motors/homing.h"
#include "motors/PaintingSides.h"
#include "motors/XYZ_Movements.h"
#include "motors/servo_motor.h"
#include "hardware/GlobalDebouncers.h"
#include "system/GlobalState.h"

// External references for immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

// Extern declaration for the WebSocket server instance
extern WebSocketsServer webSocket;

// Function-based state machine variables (defined here)
MachineState currentState = MachineState::IDLE;
MachineState previousState = MachineState::IDLE;
PaintingSubState currentPaintingSubState = PaintingSubState::NONE;
PaintingProgress currentPaintingProgress = PaintingProgress::NONE;

// State timing
unsigned long stateStartTime = 0;
unsigned long subStateStartTime = 0;

// Internal state tracking
static bool stateEntered = false;

//* ************************************************************************
//* ************************ MAIN STATE MACHINE ***************************
//* ************************************************************************

void initializeStateMachine() {
    Serial.println("Initializing function-based state machine...");
    currentState = MachineState::IDLE;
    previousState = MachineState::IDLE;
    currentPaintingSubState = PaintingSubState::NONE;
    currentPaintingProgress = PaintingProgress::NONE;
    stateStartTime = millis();
    stateEntered = false;
    Serial.println("Function-based state machine initialized to IDLE");
}

void updateStateMachine() {
    // Check for immediate commands FIRST - this is the key fix!
    if (immediateCommandPending) {
        Serial.println("*** IMMEDIATE COMMAND INTERRUPTING STATE MACHINE ***");
        // Process immediate command by changing state appropriately
        if (pendingCommand.indexOf("HOME") != -1) {
            changeState(MachineState::HOMING);
        } else if (pendingCommand.indexOf("PAUSE") != -1) {
            // Handle pause without changing state
            extern bool isPaused;
            isPaused = true;
            Serial.println("System paused via immediate command");
        } else if (pendingCommand.indexOf("CLEAN") != -1) {
            changeState(MachineState::CLEANING);
        }
        
        // Clear the immediate command
        immediateCommandPending = false;
        pendingCommand = "";
        pendingCommandClientNum = 0;
    }
    
    // Enter state if not already entered
    if (!stateEntered) {
        switch (currentState) {
            case MachineState::IDLE:
                enterIdleState();
                break;
            case MachineState::HOMING:
                enterHomingState();
                break;
            case MachineState::PAINTING:
                enterPaintingState();
                break;
            case MachineState::PNP:
                enterPnPState();
                break;
            case MachineState::CLEANING:
                enterCleaningState();
                break;
            case MachineState::INSPECT_TIP:
                enterInspectTipState();
                break;
        }
        stateEntered = true;
    }
    
    // Update current state (NON-BLOCKING!)
    switch (currentState) {
        case MachineState::IDLE:
            updateIdleState();
            break;
        case MachineState::HOMING:
            updateHomingState();
            break;
        case MachineState::PAINTING:
            updatePaintingState();
            break;
        case MachineState::PNP:
            updatePnPState();
            break;
        case MachineState::CLEANING:
            updateCleaningState();
            break;
        case MachineState::INSPECT_TIP:
            updateInspectTipState();
            break;
    }
}

void changeState(MachineState newState) {
    if (newState == currentState) {
        return; // No change needed
    }
    
    Serial.print("State transition: ");
    Serial.print(getStateName(currentState));
    Serial.print(" -> ");
    Serial.println(getStateName(newState));
    
    // Exit current state
    if (stateEntered) {
        switch (currentState) {
            case MachineState::IDLE:
                exitIdleState();
                break;
            case MachineState::HOMING:
                exitHomingState();
                break;
            case MachineState::PAINTING:
                exitPaintingState();
                break;
            case MachineState::PNP:
                exitPnPState();
                break;
            case MachineState::CLEANING:
                exitCleaningState();
                break;
            case MachineState::INSPECT_TIP:
                exitInspectTipState();
                break;
        }
    }
    
    // Change state
    previousState = currentState;
    currentState = newState;
    stateStartTime = millis();
    stateEntered = false;
    
    // Reset sub-states when changing main state
    if (newState != MachineState::PAINTING) {
        currentPaintingSubState = PaintingSubState::NONE;
        currentPaintingProgress = PaintingProgress::NONE;
    }
}

void changePaintingSubState(PaintingSubState newSubState) {
    if (newSubState != currentPaintingSubState) {
        Serial.print("Painting sub-state: ");
        Serial.print(getPaintingSubStateName(currentPaintingSubState));
        Serial.print(" -> ");
        Serial.println(getPaintingSubStateName(newSubState));
        
        currentPaintingSubState = newSubState;
        subStateStartTime = millis();
        currentPaintingProgress = PaintingProgress::NONE;
    }
}

void changePaintingProgress(PaintingProgress newProgress) {
    if (newProgress != currentPaintingProgress) {
        Serial.print("Painting progress: ");
        Serial.print(getPaintingProgressName(currentPaintingProgress));
        Serial.print(" -> ");
        Serial.println(getPaintingProgressName(newProgress));
        
        currentPaintingProgress = newProgress;
    }
}

//* ************************************************************************
//* ************************** UTILITY FUNCTIONS ***************************
//* ************************************************************************

const char* getStateName(MachineState state) {
    switch (state) {
        case MachineState::IDLE: return "IDLE";
        case MachineState::HOMING: return "HOMING";
        case MachineState::PAINTING: return "PAINTING";
        case MachineState::PNP: return "PNP";
        case MachineState::CLEANING: return "CLEANING";
        case MachineState::INSPECT_TIP: return "INSPECT_TIP";
        default: return "UNKNOWN";
    }
}

const char* getPaintingSubStateName(PaintingSubState subState) {
    switch (subState) {
        case PaintingSubState::NONE: return "NONE";
        case PaintingSubState::PREPARING: return "PREPARING";
        case PaintingSubState::SIDE_1: return "SIDE_1";
        case PaintingSubState::SIDE_2: return "SIDE_2";
        case PaintingSubState::SIDE_3: return "SIDE_3";
        case PaintingSubState::SIDE_4: return "SIDE_4";
        case PaintingSubState::ALL_SIDES: return "ALL_SIDES";
        case PaintingSubState::FINISHING: return "FINISHING";
        default: return "UNKNOWN";
    }
}

const char* getPaintingProgressName(PaintingProgress progress) {
    switch (progress) {
        case PaintingProgress::NONE: return "NONE";
        case PaintingProgress::MOVING_TO_START: return "MOVING_TO_START";
        case PaintingProgress::PAINTING_SWEEP_1: return "PAINTING_SWEEP_1";
        case PaintingProgress::PAINTING_SWEEP_2: return "PAINTING_SWEEP_2";
        case PaintingProgress::PAINTING_SWEEP_3: return "PAINTING_SWEEP_3";
        case PaintingProgress::PAINTING_SWEEP_4: return "PAINTING_SWEEP_4";
        case PaintingProgress::PAINTING_SWEEP_5: return "PAINTING_SWEEP_5";
        case PaintingProgress::MOVING_TO_SAFE: return "MOVING_TO_SAFE";
        case PaintingProgress::COMPLETED: return "COMPLETED";
        default: return "UNKNOWN";
    }
}

const char* getCurrentStateName() {
    return getStateName(currentState);
}

// State checking functions
bool isIdleState() { return currentState == MachineState::IDLE; }
bool isHomingState() { return currentState == MachineState::HOMING; }
bool isPaintingState() { return currentState == MachineState::PAINTING; }
bool isPnPState() { return currentState == MachineState::PNP; }
bool isCleaningState() { return currentState == MachineState::CLEANING; }
bool isInspectTipState() { return currentState == MachineState::INSPECT_TIP; }

// Emergency state management
void emergencyStop() {
    Serial.println("*** EMERGENCY STOP ACTIVATED ***");
    
    // Stop all motors immediately
    extern FastAccelStepper *stepperX, *stepperY_Left, *stepperY_Right, *stepperZ;
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
    extern void paintGun_OFF();
    paintGun_OFF();
    
    // Force to idle state
    forceToIdleState();
}

void forceToIdleState() {
    Serial.println("Forcing state machine to IDLE");
    currentState = MachineState::IDLE;
    currentPaintingSubState = PaintingSubState::NONE;
    currentPaintingProgress = PaintingProgress::NONE;
    stateEntered = false;
    stateStartTime = millis();
}

//* ************************************************************************
//* ******************** STATE FUNCTION DECLARATIONS **********************
//* ************************************************************************
// These functions would be implemented in their respective state files
// For now, providing stub implementations to allow compilation

void enterIdleState() {
    Serial.println("Entering IDLE state");
    
    // Clear any lingering pause state when returning to idle
    isPaused = false;
    Serial.println("IdleState: Cleared pause state on entry");
    
    // TODO: Set servo to 180 degrees - setServoAngle function needs to be implemented
    // setServoAngle(180);
    Serial.println("Servo would be set to 180 degrees in Idle State (function not implemented).");
    
    Serial.println("Idle state active. Press PnP cycle sensor to enter PnP mode.");
}

void updateIdleState() {
    // Update the global PnP cycle sensor debouncer
    g_pnpCycleSensorDebouncer.update();
    
    // Debug: Print sensor value every second
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 1000) {
        int sensorValue = g_pnpCycleSensorDebouncer.read();
        lastDebugTime = millis();
    }
    
    // Check if the PnP cycle sensor is pressed (active LOW, detected by falling edge)
    if (g_pnpCycleSensorDebouncer.fell()) {
        Serial.println("PnP Cycle Sensor activated (falling edge) in IdleState. Transitioning to PnPState...");
        changeState(MachineState::PNP);
        return;
    }
}

void exitIdleState() {
    Serial.println("Exiting IDLE state");
}

void enterHomingState() {
    Serial.println("Entering HOMING state");
    
    // TODO: Implement homing logic
    // For now, just transition to IDLE
    Serial.println("Homing state entered - transitioning to IDLE");
    changeState(MachineState::IDLE);
}

void updateHomingState() {
    // Homing is handled in enterHomingState() - this is just a placeholder
    // The homing process is blocking, so we don't need continuous updates
}

void exitHomingState() {
    Serial.println("Exiting HOMING state");
}

void enterPaintingState() {
    Serial.println("Entering PAINTING state");
    
    // Clear any lingering pause state from previous cycles
    isPaused = false;
    Serial.println("PaintingState: Cleared pause state for new painting cycle");
    
    // Start painting all sides directly
    Serial.println("PaintingState: Starting 'All Sides' painting routine.");
    paintAllSides(); // This is a blocking call
    Serial.println("PaintingState: All Sides Painting routine finished.");
    
    // Move to position before homing
    Serial.println("PaintingState: Moving to position (1,1,0) before Homing.");
    long xPos = (long)(1.0 * STEPS_PER_INCH_XYZ);
    long yPos = (long)(1.0 * STEPS_PER_INCH_XYZ);
    long zPos = 0;
    
    moveToXYZ(xPos, DEFAULT_X_SPEED, yPos, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
    Serial.println("PaintingState: Reached position (1,1,0).");
    
    // Transition to homing
    Serial.println("PaintingState: Sequence complete. Requesting Homing State.");
    changeState(MachineState::HOMING);
}

void updatePaintingState() {
    // Painting logic is handled in enterPaintingState() as a blocking operation
    // This function is called during the painting process for immediate command checking
    
    // Check for immediate commands that should interrupt painting
    extern bool immediateCommandPending;
    if (immediateCommandPending) {
        Serial.println("PaintingState: Immediate command detected - interrupting painting process");
        
        // Stop any running motors immediately
        extern FastAccelStepper *stepperX, *stepperY_Left, *stepperY_Right, *stepperZ;
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
        extern void paintGun_OFF();
        paintGun_OFF();
        
        // Let the main loop handle the immediate command
        return;
    }
}

void exitPaintingState() {
    Serial.println("Exiting PAINTING state");
    
    // Turn off paint gun for safety
    extern void paintGun_OFF();
    paintGun_OFF();
}

void enterPnPState() {
    Serial.println("Entering PNP state");
}

void updatePnPState() {
    // PnP state update logic
}

void exitPnPState() {
    Serial.println("Exiting PNP state");
}

void enterCleaningState() {
    Serial.println("Entering CLEANING state");
}

void updateCleaningState() {
    // Cleaning state update logic
}

void exitCleaningState() {
    Serial.println("Exiting CLEANING state");
}

void enterInspectTipState() {
    Serial.println("Entering INSPECT_TIP state");
}

void updateInspectTipState() {
    // Inspect tip state update logic
}

void exitInspectTipState() {
    Serial.println("Exiting INSPECT_TIP state");
}

//* ************************************************************************
//* ******************** TRANSITION MANAGEMENT ***************************
//* ************************************************************************

// Global transition flag
static bool transitioningToPaintAllSides = false;

bool isTransitioningToPaintAllSides() {
    return transitioningToPaintAllSides;
}

void setTransitioningToPaintAllSides(bool flag) {
    transitioningToPaintAllSides = flag;
    Serial.print("Transition to Paint All Sides flag set to: ");
    Serial.println(flag ? "true" : "false");
}

void clearTransitioningToPaintAllSidesFlag() {
    transitioningToPaintAllSides = false;
    Serial.println("Paint All Sides transition flag cleared");
}

String createStatusJson() {
    // Basic status JSON implementation
    String status = "{";
    status += "\"state\":\"" + String(getCurrentStateName()) + "\",";
    status += "\"timestamp\":" + String(millis());
    status += "}";
    return status;
} 