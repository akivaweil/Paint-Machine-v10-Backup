#ifndef HOMING_STATE_H
#define HOMING_STATE_H

#include "states/State.h"
#include "motors/Homing.h"

// Machine state constants
// #define MACHINE_HOMING 1

// Function declarations
// bool homeAllAxes();

class HomingState : public State {
public:
    HomingState();
    ~HomingState();
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;

private:
    Homing* _homingController;
    bool _isHoming;
    bool _homingComplete;
    bool _homingSuccess;
};

#endif // HOMING_STATE_H 