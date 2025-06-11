#ifndef INSPECT_TIP_STATE_H
#define INSPECT_TIP_STATE_H

#include "State.h"

class InspectTipState : public State {
public:
    InspectTipState();
    void enter() override;
    void update() override;
    void exit() override;
    const char* getName() const override;
    
    // Method to trigger return to idle
    void returnToIdle();

private:
    enum InspectTipSubStep {
        ITS_IDLE,
        ITS_MOVING_TO_INSPECT_POSITION,
        ITS_AT_INSPECT_POSITION,
        ITS_RETURNING_TO_ORIGINAL_POSITION,
        ITS_RETURNING_TO_IDLE
    };
    InspectTipSubStep currentStep;
    bool isInspecting;
    
    // Store original position to return to
    long originalX;
    long originalY;
    long originalZ;
};

#endif // INSPECT_TIP_STATE_H 