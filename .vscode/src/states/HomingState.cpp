#include "states/HomingState.h"
#include <Arduino.h>
// #include <Bounce2.h> // No longer needed here
#include <FastAccelStepper.h>
#include "utils/settings.h"
// #include "system/machine_state.h" // No longer needed
#include "system/StateMachine.h" 
// #include "motors/XYZ_Movements.h" // XYZ_Movements likely included via Homing.h if needed
#include "motors/Homing.h" // Include the new Homing class header
#include "system/GlobalState.h"      // ADDED for isPaused
#include <WebSocketsServer.h>       // ADDED for webSocket.loop() in pause

// Add extern declaration for homeCommandReceived
extern volatile bool homeCommandReceived;

// // Declare global variables used by the homing state
// const unsigned long HOMING_SWITCH_DEBOUNCE_MS = 3; // Moved to Homing class
bool homeAfterMovement = false; // Keep this if it's used elsewhere for triggering homing

// // Bounce objects for debouncing the homing switches - Moved to Homing class
// Bounce xHomeSwitch = Bounce();
// Bounce yLeftHomeSwitch = Bounce();
// Bounce yRightHomeSwitch = Bounce();
// Bounce zHomeSwitch = Bounce();

// Need access to the global stepper pointers
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;
// extern FastAccelStepper *rotationStepper; // Declared in Rotation_Motor.h

// Machine state variables
// bool isHoming = false; // Moved to Homing class or managed internally

// Reference to the state machine
extern StateMachine* stateMachine;
extern WebSocketsServer webSocket; // ADDED: For pause loop

// // Utility function to convert inches to steps - Moved to Homing class or shared location
// long inchesToStepsXYZ(float inches) {
//     return (long)(inches * STEPS_PER_INCH_XYZ);
// }

// // Servo control function - Forward declared/included via Homing.h if needed
// void setPitchServoAngle(int angle);

// // Import the servoInitialized variable from servo_control.cpp - Handled in Homing.cpp
// extern bool servoInitialized;

// Externally defined objects (likely in Setup.cpp)
extern FastAccelStepperEngine engine; 
// // Extern Bounce objects are not needed here anymore
// extern Bounce debounceX; 
// extern Bounce debounceY_Left;
// extern Bounce debounceY_Right;
// extern Bounce debounceZ;

HomingState::HomingState() : 
    _homingController(nullptr), // Initialize pointer
    _isHoming(false),
    _homingComplete(false),
    _homingSuccess(false)
{ 
    // Constructor implementation
}

HomingState::~HomingState() {
    delete _homingController; // Clean up controller if allocated
}

void HomingState::enter() {
    Serial.println("Entering Homing State");
    
    // Reset the home command flag since we're now processing it
    homeCommandReceived = false;
    
    // Prepare for homing
    delete _homingController; // Delete previous instance if any
    _homingController = new Homing(engine, stepperX, stepperY_Left, stepperY_Right, stepperZ);
    
    _isHoming = true;
    _homingComplete = false;
    _homingSuccess = false;
    Serial.println("Homing process initiated...");
    // DO NOT call homeAllAxes() here if it's blocking
}

void HomingState::update() {
    //! Handle Pause
    while (isPaused) {
        webSocket.loop(); // Keep WebSocket responsive
        delay(100);       // Small delay to prevent busy-waiting
    }

    // If homing process hasn't completed yet
    if (_isHoming && !_homingComplete) {
        if (_homingController) {
            Serial.println("Executing Homing::homeAllAxes()...");
            _homingSuccess = _homingController->homeAllAxes(); // BLOCKING CALL
            _homingComplete = true; // Mark as complete
            _isHoming = false;      // No longer actively homing
            Serial.println("Homing::homeAllAxes() finished.");
        } else {
            Serial.println("ERROR: HomingController is null in HomingState::update()!");
            _homingComplete = true; // Mark complete to allow transition
            _homingSuccess = false;
            _isHoming = false;
        }
    }
    
    // If homing is marked as complete, transition back to Idle
    if (_homingComplete) {
        if (_homingSuccess) {
            Serial.println("Homing successful, transitioning to IDLE state.");
        } else {
            Serial.println("Homing failed, transitioning to IDLE state.");
            // Future: Transition to ErrorState?
        }
        
        if (stateMachine) {
            stateMachine->changeState(stateMachine->getIdleState()); 
            // Reset flag for next entry after transition
            _homingComplete = false; 
        } else {
            Serial.println("ERROR: StateMachine pointer null in HomingState::update()! Cannot transition.");
             // Prevent potential infinite loop if stateMachine is null
             _homingComplete = false; 
        }
    }
}

void HomingState::exit() {
     Serial.println("Exiting Homing State");
     delete _homingController; // Clean up controller
     _homingController = nullptr;
     _isHoming = false;
     _homingComplete = false;
}

const char* HomingState::getName() const {
    return "HOMING";
}

// // Implementation of the homing logic - MOVED TO Homing.cpp
// bool homeAllAxes() { 
//    // ... entire function removed ...
// } 