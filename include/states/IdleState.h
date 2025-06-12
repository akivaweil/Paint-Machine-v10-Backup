#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "State.h"
// #include <Bounce2.h> // No longer needed here, moved to GlobalDebouncers.h
#include "hardware/GlobalDebouncers.h" // Include for g_pnpCycleSensorDebouncer

// Machine state constants
#define MACHINE_IDLE 0

class IdleState : public State {
public:
    IdleState();
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;

private:
    // Bounce pnpCycleSwitch; // REMOVED: Now using global g_pnpCycleSensorDebouncer
};

#endif // IDLE_STATE_H 