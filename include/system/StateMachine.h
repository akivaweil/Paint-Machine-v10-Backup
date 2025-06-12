#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "states/State.h"
#include "states/HomingState.h"
#include "states/PaintingState.h"
#include "states/CleaningState.h"
#include "states/PausedState.h"
#include "states/IdleState.h"
#include "states/PnPState.h"
#include "states/InspectTipState.h"

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
    State* getInspectTipState() { return inspectTipState; }
    
    // Mechanism to allow a state to define the next state after a sub-routine
    void setNextStateOverride(State* state);
    State* getNextStateOverrideAndClear();

    // Helper method to get state name for debugging
    const char* getStateName(State* state);

    // Flag and methods for 'paint all sides' transition
    void setTransitioningToPaintAllSides(bool value);
    bool isTransitioningToPaintAllSides() const;
    void clearTransitioningToPaintAllSidesFlag();

private:
    State* currentState;
    State* idleState;
    State* homingState;
    State* paintingState;
    State* cleaningState;
    State* pausedState;
    State* pnpState;
    State* inspectTipState;
    State* nextStateOverride;
    bool _isTransitioningToPaintAllSides;
};

#endif // STATE_MACHINE_H 