#ifndef STATEMACHINE_H
#define STATEMACHINE_H

#include "State.h"
#include "HomingState.h"
#include "PaintingState.h"
#include "CleaningState.h"
#include "PausedState.h"
#include "IdleState.h"
#include "PnPState.h"

class StateMachine {
public:
    StateMachine();
    ~StateMachine();
    
    void changeState(State* newState);
    void update();
    State* getCurrentState() { return currentState; }
    
    // Getter methods for state access from other classes
    State* getIdleState() { return idleState; }
    State* getHomingState() { return homingState; }
    State* getPaintingState() { return paintingState; }
    State* getCleaningState() { return cleaningState; }
    State* getPausedState() { return pausedState; }
    State* getPnpState() { return pnpState; }
    
    // Helper method to get state name for debugging
    const char* getStateName(State* state);

private:
    State* currentState;
    State* idleState;
    State* homingState;
    State* paintingState;
    State* cleaningState;
    State* pausedState;
    State* pnpState;
};

#endif // STATEMACHINE_H 