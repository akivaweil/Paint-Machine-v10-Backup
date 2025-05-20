#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#include <Arduino.h>

// Machine state constants
#define MACHINE_IDLE        0
#define MACHINE_HOMING      1
#define MACHINE_PAINTING    2
#define MACHINE_PNP         3 // Used for Pick and Place mode
#define MACHINE_CLEANING    4
#define MACHINE_MANUAL_MOVE 5 // Added for manual control
#define MACHINE_ERROR       99

// Forward declaration
class StateMachine;
extern StateMachine* stateMachine;

// Machine flags
extern volatile bool homeCommandReceived;

// Function declarations
void setMachineState(int state);
void clearMachineState();
int getMachineState();
void updateMachineState();

// Function to sync machine state with StateMachine
void syncMachineStateWithStateMachine(int state);

#endif // MACHINE_STATE_H 