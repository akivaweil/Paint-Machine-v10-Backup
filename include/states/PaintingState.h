#ifndef PAINTING_STATE_H
#define PAINTING_STATE_H

#include "State.h"

class PaintingState : public State {
public:
    PaintingState();
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;

private:
    enum PaintingSubStep {
        PS_IDLE,
        PS_REQUEST_PRE_PAINT_CLEAN,
        PS_PERFORM_ALL_SIDES_PAINTING,
        PS_MOVE_TO_ZERO_BEFORE_HOMING,
        PS_REQUEST_HOMING
    };
    PaintingSubStep currentStep;
};

#endif // PAINTING_STATE_H 