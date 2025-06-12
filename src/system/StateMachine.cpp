#include "system/StateMachine.h"
#include "states/IdleState.h"
#include "states/HomingState.h"
#include "states/PaintingState.h"
#include "states/CleaningState.h"
#include "states/PausedState.h"
#include "states/PnPState.h"
#include "states/InspectTipState.h"
#include <Arduino.h>
#include "system/machine_state.h"
#include "states/State.h"
#include <WebSocketsServer.h>

// External references for immediate command system
extern bool immediateCommandPending;
extern String pendingCommand;
extern uint8_t pendingCommandClientNum;

// Extern declaration for the WebSocket server instance
extern WebSocketsServer webSocket;

// Flag to prevent circular state changes
bool inStateTransition = false;

// Global state machine pointer
StateMachine* stateMachine = nullptr;

StateMachine::StateMachine() : 
    currentState(nullptr),
    nextStateOverride(nullptr),
    _isTransitioningToPaintAllSides(false)
{
    // Initialize all state objects
    idleState = new IdleState();
    homingState = new HomingState();
    paintingState = new PaintingState();
    cleaningState = new CleaningState();
    pausedState = new PausedState();
    pnpState = new PnPState();
    inspectTipState = new InspectTipState();
    
    // Set initial state to idle
    currentState = idleState;
    currentState->enter();
    
    // Set the global pointer
    stateMachine = this;
    
    Serial.println("State Machine initialized with Idle state");
}

StateMachine::~StateMachine() {
    // Clean up all state objects
    delete idleState;
    delete homingState;
    delete paintingState;
    delete cleaningState;
    delete pausedState;
    delete pnpState;
    delete inspectTipState;
    
    // Clear the global pointer
    stateMachine = nullptr;
}

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

void StateMachine::changeState(State* newState) {
    if (newState == nullptr) {
        Serial.println("ERROR: Attempted to change to NULL state!");
        return;
    }
    
    const char* newStateName = newState->getName();
    
    // Check if already in the target state
    if (currentState == newState) {
        Serial.print("INFO: Already in state: ");
        Serial.println(newStateName);
        return;
    }

    // Set flag to prevent circular state changes
    inStateTransition = true;
    
    if (currentState != nullptr) {
        Serial.print("Changing state from ");
        Serial.print(currentState->getName());
        Serial.print(" to ");
        Serial.println(newStateName);
        currentState->exit();
    }
    
    currentState = newState;
    currentState->enter();

    // **ENHANCED**: Broadcast state change with better WebSocket handling
    String stateMessage = "STATE:";
    stateMessage += newStateName;
    webSocket.broadcastTXT(stateMessage);
    Serial.print("Broadcasted state: ");
    Serial.println(stateMessage);
    
    // Clear flag
    inStateTransition = false;
}

void StateMachine::update() {
    // **KEY ENHANCEMENT**: Check for immediate commands FIRST
    if (immediateCommandPending) {
        Serial.println("*** IMMEDIATE COMMAND DETECTED - Processing immediately ***");
        
        // Emergency stop all motors if needed
        if (pendingCommand.indexOf("HOME") != -1 || pendingCommand.indexOf("CLEAN") != -1) {
            // Stop motors immediately for safety
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
        }
        
        // Process the immediate command by calling the processor directly
        extern void processWebCommand(WebSocketsServer* webSocket, uint8_t num, String commandPayload);
        processWebCommand(&webSocket, pendingCommandClientNum, pendingCommand);
        
        // Clear the immediate command
        immediateCommandPending = false;
        pendingCommand = "";
        pendingCommandClientNum = 0;
    }
    
    // Update current state
    if (currentState != nullptr) {
        currentState->update();
    }
}

// --- nextStateOverride Methods ---
void StateMachine::setNextStateOverride(State* state) {
    nextStateOverride = state;
    if (state) {
        Serial.printf("StateMachine: Next state override set to: %s\n", state->getName());
    } else {
        Serial.println("StateMachine: Next state override cleared.");
    }
}

State* StateMachine::getNextStateOverrideAndClear() {
    State* temp = nextStateOverride;
    if (temp) {
        Serial.printf("StateMachine: Consuming next state override: %s\n", temp->getName());
    }
    nextStateOverride = nullptr;
    return temp;
}

// --- Paint All Sides Transition Flag Methods ---
void StateMachine::setTransitioningToPaintAllSides(bool value) {
    _isTransitioningToPaintAllSides = value;
    if (value) {
        Serial.println("StateMachine: Flag set - transitioning to Paint All Sides sequence.");
    } else {
        Serial.println("StateMachine: Flag cleared - no longer transitioning to Paint All Sides sequence.");
    }
}

bool StateMachine::isTransitioningToPaintAllSides() const {
    return _isTransitioningToPaintAllSides;
}

void StateMachine::clearTransitioningToPaintAllSidesFlag() {
    if (_isTransitioningToPaintAllSides) {
        Serial.println("StateMachine: Paint All Sides transition flag explicitly cleared.");
    }
    _isTransitioningToPaintAllSides = false;
}

// Helper function to get state name
const char* StateMachine::getStateName(State* state) {
    if (state != nullptr) {
        return state->getName();
    } else {
        return "Unknown";
    }
} 