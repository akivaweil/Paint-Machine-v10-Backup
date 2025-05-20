#ifndef MACHINE_STATE_H
#define MACHINE_STATE_H

#include <Arduino.h>

// Removed MachineState enum and associated functions

// Forward declaration for StateMachine might still be needed if extern is used?
// class StateMachine;
// extern StateMachine* stateMachine; // Keep if other files include this for the extern
// Let's remove these for now, includes should come from StateMachine.h directly where needed.

// Machine flags
extern volatile bool homeCommandReceived;

#endif // MACHINE_STATE_H 