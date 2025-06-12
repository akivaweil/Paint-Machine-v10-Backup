#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// Function-based state machine enums
enum class MachineState {
    IDLE,
    HOMING,
    PAINTING,
    PNP,
    CLEANING,
    INSPECT_TIP
};

enum class PaintingSubState {
    NONE,
    PREPARING,
    SIDE_1,
    SIDE_2,
    SIDE_3,
    SIDE_4,
    ALL_SIDES,
    FINISHING
};

enum class PaintingProgress {
    NONE,
    MOVING_TO_START,
    PAINTING_SWEEP_1,
    PAINTING_SWEEP_2,
    PAINTING_SWEEP_3,
    PAINTING_SWEEP_4,
    PAINTING_SWEEP_5,
    MOVING_TO_SAFE,
    COMPLETED
};

// Global state variables
extern MachineState currentState;
extern MachineState previousState;
extern PaintingSubState currentPaintingSubState;
extern PaintingProgress currentPaintingProgress;

// Function-based state machine functions
void initializeStateMachine();
void updateStateMachine();
void changeState(MachineState newState);
void changePaintingSubState(PaintingSubState newSubState);
void changePaintingProgress(PaintingProgress newProgress);

// Utility functions
const char* getStateName(MachineState state);
const char* getPaintingSubStateName(PaintingSubState subState);
const char* getPaintingProgressName(PaintingProgress progress);
const char* getCurrentStateName();

// State checking functions
bool isIdleState();
bool isHomingState();
bool isPaintingState();
bool isPnPState();
bool isCleaningState();
bool isInspectTipState();

// Emergency state management
void emergencyStop();
void forceToIdleState();

// Individual state functions
void enterIdleState();
void updateIdleState();
void exitIdleState();

void enterHomingState();
void updateHomingState();
void exitHomingState();

void enterPaintingState();
void updatePaintingState();
void exitPaintingState();

void enterPnPState();
void updatePnPState();
void exitPnPState();

void enterCleaningState();
void updateCleaningState();
void exitCleaningState();

void enterInspectTipState();
void updateInspectTipState();
void exitInspectTipState();

#endif // STATE_MACHINE_H 