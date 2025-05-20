#ifndef CLEANING_STATE_H
#define CLEANING_STATE_H

#include "State.h"

class CleaningState : public State {
public:
    CleaningState();
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;

    void setShortMode(bool mode);

private:
    bool _isCleaning;
    bool _cleaningComplete;
    bool shortMode; // Added for short cleaning cycle
};

#endif // CLEANING_STATE_H 