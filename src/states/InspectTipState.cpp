#include "states/InspectTipState.h"
#include <Arduino.h>
#include "motors/XYZ_Movements.h"
#include "utils/settings.h"
#include "system/StateMachine.h"
#include <WebSocketsServer.h>

extern StateMachine *stateMachine;
extern FastAccelStepper* stepperX;
extern FastAccelStepper* stepperY_Left;
extern FastAccelStepper* stepperY_Right;
extern FastAccelStepper* stepperZ;
extern WebSocketsServer webSocket;

//* ************************************************************************
//* ************************ INSPECT TIP STATE ****************************
//* ************************************************************************

InspectTipState::InspectTipState() : currentStep(ITS_IDLE), isInspecting(false), originalX(0), originalY(0), originalZ(0) {
    // Constructor implementation
}

void InspectTipState::enter() {
    Serial.println("InspectTipState: Entering Inspect Tip State");
    
    // Store current position to return to later
    originalX = stepperX->getCurrentPosition();
    originalY = stepperY_Left->getCurrentPosition(); // Assuming Left/Right are synced
    originalZ = stepperZ->getCurrentPosition();
    
    Serial.printf("InspectTipState: Stored original position - X:%ld, Y:%ld, Z:%ld\n", originalX, originalY, originalZ);
    
    currentStep = ITS_MOVING_TO_INSPECT_POSITION;
    isInspecting = true;
    
    // Broadcast status to web interface
    webSocket.broadcastTXT("INSPECT_TIP_STATUS:ON");
}

void InspectTipState::update() {
    switch (currentStep) {
        case ITS_MOVING_TO_INSPECT_POSITION:
        {
            Serial.println("InspectTipState: Moving to inspect position (10, 0.5, 0)");
            
            // Convert inches to steps
            long xPos = (long)(10.0 * STEPS_PER_INCH_XYZ);
            long yPos = (long)(0.5 * STEPS_PER_INCH_XYZ);
            long zPos = 0; // Stay at current Z or go to 0
            
            // Move to inspect position
            moveToXYZ(xPos, DEFAULT_X_SPEED, yPos, DEFAULT_Y_SPEED, zPos, DEFAULT_Z_SPEED);
            
            Serial.println("InspectTipState: Reached inspect position");
            currentStep = ITS_AT_INSPECT_POSITION;
            break;
        }
            
        case ITS_AT_INSPECT_POSITION:
            // Stay at inspect position until told to exit
            // This state will be exited when INSPECT_TIP_OFF command is received
            break;
            
        case ITS_RETURNING_TO_ORIGINAL_POSITION:
            Serial.println("InspectTipState: Returning to original position");
            Serial.printf("InspectTipState: Moving back to X:%ld, Y:%ld, Z:%ld\n", originalX, originalY, originalZ);
            
            // Move back to original position
            moveToXYZ(originalX, DEFAULT_X_SPEED, originalY, DEFAULT_Y_SPEED, originalZ, DEFAULT_Z_SPEED);
            
            Serial.println("InspectTipState: Reached original position, transitioning to idle");
            currentStep = ITS_RETURNING_TO_IDLE;
            // Fall through to ITS_RETURNING_TO_IDLE
            
        case ITS_RETURNING_TO_IDLE:
            Serial.println("InspectTipState: Returning to idle state");
            if (stateMachine && stateMachine->getIdleState()) {
                stateMachine->changeState(stateMachine->getIdleState());
            }
            currentStep = ITS_IDLE;
            break;
            
        case ITS_TRANSITIONING_TO_PAINTING:
            Serial.println("InspectTipState: Transitioning to painting state");
            if (stateMachine && stateMachine->getPaintingState()) {
                stateMachine->changeState(stateMachine->getPaintingState());
            } else {
                Serial.println("ERROR: InspectTipState - Cannot transition to PaintingState, returning to idle");
                if (stateMachine && stateMachine->getIdleState()) {
                    stateMachine->changeState(stateMachine->getIdleState());
                }
            }
            currentStep = ITS_IDLE;
            break;
            
        case ITS_TRANSITIONING_TO_PNP:
            Serial.println("InspectTipState: Transitioning to PnP state");
            if (stateMachine && stateMachine->getPnpState()) {
                stateMachine->changeState(stateMachine->getPnpState());
            } else {
                Serial.println("ERROR: InspectTipState - Cannot transition to PnPState, returning to idle");
                if (stateMachine && stateMachine->getIdleState()) {
                    stateMachine->changeState(stateMachine->getIdleState());
                }
            }
            currentStep = ITS_IDLE;
            break;
            
        case ITS_IDLE:
            // Should not reach here normally
            break;
    }
}

void InspectTipState::exit() {
    Serial.println("InspectTipState: Exiting Inspect Tip State");
    isInspecting = false;
    currentStep = ITS_IDLE;
    
    // Broadcast status to web interface
    webSocket.broadcastTXT("INSPECT_TIP_STATUS:OFF");
}

const char* InspectTipState::getName() const {
    return "INSPECT_TIP";
}

// Method to trigger return to idle (called from web command)
void InspectTipState::returnToIdle() {
    if (currentStep == ITS_AT_INSPECT_POSITION) {
        currentStep = ITS_RETURNING_TO_ORIGINAL_POSITION;
    }
}

// Method to trigger transition to painting (called from web command)
void InspectTipState::transitionToPainting() {
    if (currentStep == ITS_AT_INSPECT_POSITION) {
        Serial.println("InspectTipState: Transitioning directly to painting from inspect position");
        currentStep = ITS_TRANSITIONING_TO_PAINTING;
    }
}

// Method to trigger transition to PnP (called from web command)
void InspectTipState::transitionToPnP() {
    if (currentStep == ITS_AT_INSPECT_POSITION) {
        Serial.println("InspectTipState: Transitioning directly to PnP from inspect position");
        currentStep = ITS_TRANSITIONING_TO_PNP;
    }
} 