#include "system/StateMachine.h" // Updated path
#include "states/IdleState.h"
#include "states/HomingState.h"
#include "states/PaintingState.h"
#include "states/CleaningState.h"
#include "states/PausedState.h"
#include "states/PnPState.h"
#include <Arduino.h>
#include "system/machine_state.h" // Updated path
#include "states/State.h"
#include <WebSocketsServer.h> // Added WebSocket header

//* ************************************************************************
//* ************************* STATE MACHINE *******************************
//* ************************************************************************

// Global state machine pointer
StateMachine* stateMachine = nullptr;

// Extern declaration for the WebSocket server instance
// Assuming it's defined in Setup.cpp or Web_Dashboard_Commands.cpp
extern WebSocketsServer webSocket;

// Flag to prevent circular state changes
bool inStateTransition = false;

StateMachine::StateMachine() : 
    currentState(nullptr),
    nextStateOverride(nullptr), // Initialize nextStateOverride
    _isTransitioningToPaintAllSides(false) // Initialize the new flag
    // Initialize state instances here if using composition
    // homingState(), // Example - PnP was likely here
    // paintingSide1State(), 
    // ... etc.
{
    // Initialize all state objects
    idleState = new IdleState();
    homingState = new HomingState();
    paintingState = new PaintingState();
    cleaningState = new CleaningState();
    pausedState = new PausedState();
    pnpState = new PnPState();
    
    // Set initial state to idle
    currentState = idleState;
    currentState->enter();
    
    // Set the global pointer
    stateMachine = this;
    
    Serial.println("State Machine initialized with Idle state");
    // Initial state broadcast might be needed here if webSocket is ready
    // but it's safer to do it on connect in webSocketEvent
}

StateMachine::~StateMachine() {
    // Clean up all state objects
    delete idleState;
    delete homingState;
    delete paintingState;
    delete cleaningState;
    delete pausedState;
    delete pnpState;
    
    // Clear the global pointer
    stateMachine = nullptr;
}

void StateMachine::changeState(State* newState) {
    if (newState == nullptr) {
        Serial.println("ERROR: Attempted to change to NULL state!");
        return;
    }
    
    // Use the new getName() method
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
        Serial.print(currentState->getName()); // Use getName here too
        Serial.print(" to ");
        Serial.println(newStateName);
        currentState->exit();
    }
    
    currentState = newState;
    currentState->enter();

    // --- Broadcast state change --- 
    String stateMessage = "STATE:";
    stateMessage += newStateName;
    webSocket.broadcastTXT(stateMessage);
    Serial.print("Broadcasted state: ");
    Serial.println(stateMessage);
    // ----------------------------
    
    // Clear flag
    inStateTransition = false;
}

void StateMachine::update() {
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
// --------------------------------

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
// ---------------------------------------------

// Helper function to get state name (now uses the virtual method)
const char* StateMachine::getStateName(State* state) {
    if (state != nullptr) {
        return state->getName();
    } else {
        return "Unknown";
    }
}

/* // REMOVING this duplicate/incorrect initialize() definition
void StateMachine::initialize() {
    Serial.println("Initializing State Machine...");
    // Initialize state instances - Ensure PnP is removed if it was here
    // pnpState = new PnPState(); // Example - REMOVED

    // Set the initial state
    currentState = getHomingState(); // Start with homing or idle
    if (currentState != nullptr) {
        currentState->enter();
    } else {
        Serial.println("ERROR: Initial state is null!");
        // Handle error appropriately, maybe enter an error state
    }
} 
*/ 