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
    
    // Method to trigger transition to painting
    void transitionToPainting();
    
    // Method to trigger transition to PnP
    void transitionToPnP();

private:
    enum InspectTipSubStep {
        ITS_IDLE,
        ITS_MOVING_TO_INSPECT_POSITION,
        ITS_AT_INSPECT_POSITION,
        ITS_RETURNING_TO_ORIGINAL_POSITION,
        ITS_RETURNING_TO_IDLE,
        ITS_TRANSITIONING_TO_PAINTING,
        ITS_TRANSITIONING_TO_PNP
    };
    InspectTipSubStep currentStep;
    bool isInspecting;
    
    // Store original position to return to
    long originalX;
    long originalY;
    long originalZ;
};

#endif // INSPECT_TIP_STATE_H 