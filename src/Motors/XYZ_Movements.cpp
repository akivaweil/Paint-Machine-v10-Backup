#include "motors/XYZ_Movements.h"
#include <Arduino.h>
#include "utils/settings.h" // Likely needed for pin definitions, steps/mm, etc.

// Include motor control library
#include <FastAccelStepper.h>
#include <Bounce2.h>   // For debouncing limit switches
#include "web/Web_Dashboard_Commands.h" // For checking home commands
#include "system/GlobalState.h"      // ADDED for isPaused
#include <WebSocketsServer.h>       // ADDED for webSocket.loop() in pause

// Define stepper engine and steppers (example)
extern FastAccelStepperEngine engine; // Use the global one from Setup.cpp
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left; // Renamed from stepperY
extern FastAccelStepper *stepperY_Right; // Added second Y motor
extern FastAccelStepper *stepperZ;
extern WebSocketsServer webSocket; // ADDED: For pause loop

// Switch debouncing objects
extern Bounce debounceX;
extern Bounce debounceY_Left; // Renamed from debounceY
extern Bounce debounceY_Right; // Added second Y debouncer
extern Bounce debounceZ;

extern volatile bool homeCommandReceived; // For direct access to the flag

//* ************************************************************************
//* ************************* XYZ MOVEMENTS **************************
//* ************************************************************************

/* REMOVED - Logic moved to Setup.cpp
void setupMotors() {
    Serial.println("Setting up Motors...");
    engine.init();
    
    // Example Setup - Replace with actual pins and settings from settings.h
    // stepperX = engine.stepperConnectToPins(X_STEP_PIN, X_DIR_PIN);
    // if (stepperX) {
    //     stepperX->setEnablePin(X_ENABLE_PIN);
    //     stepperX->setAutoEnable(true);
    // }

    // Setup Y and Z similarly...
    
    // Setup limit switches - moved from setupFunctionality()
    pinMode(X_HOME_SWITCH, INPUT_PULLUP);
    pinMode(Y_LEFT_HOME_SWITCH, INPUT_PULLUP);
    pinMode(Y_RIGHT_HOME_SWITCH, INPUT_PULLUP);
    pinMode(Z_HOME_SWITCH, INPUT_PULLUP);
    
    // Setup Bounce2 for debouncing
    debounceX.attach(X_HOME_SWITCH);
    debounceX.interval(DEBOUNCE_INTERVAL);
    
    debounceY_Left.attach(Y_LEFT_HOME_SWITCH);
    debounceY_Left.interval(DEBOUNCE_INTERVAL);
    
    debounceY_Right.attach(Y_RIGHT_HOME_SWITCH);
    debounceY_Right.interval(DEBOUNCE_INTERVAL);

    debounceZ.attach(Z_HOME_SWITCH);
    debounceZ.interval(DEBOUNCE_INTERVAL);

    Serial.println("Motors and Switches Setup Complete.");
}
*/

void moveToXYZ(long x, unsigned int xSpeed, long y, unsigned int ySpeed, long z, unsigned int zSpeed) {
    // Set speed for each stepper individually
    stepperX->setSpeedInHz(xSpeed);
    stepperY_Left->setSpeedInHz(ySpeed); // Renamed
    stepperY_Right->setSpeedInHz(ySpeed); // Added second Y motor speed
    stepperZ->setSpeedInHz(zSpeed);
    
    // Command the move to the absolute position
    stepperX->moveTo(x);
    stepperY_Left->moveTo(y); // Renamed
    stepperY_Right->moveTo(y); // Added second Y motor move
    stepperZ->moveTo(z);
    
    // Wait until all steppers have completed their movements
    while (stepperX->isRunning() || stepperY_Left->isRunning() || stepperY_Right->isRunning() || stepperZ->isRunning()) { // Updated condition
        //! Handle Pause
        while (isPaused) {
            webSocket.loop(); // Keep WebSocket responsive
            // Steppers will hold their position when isPaused is true and this inner loop runs.
            // No need to stop/restart them unless specific behavior is desired.
            delay(100);       
        }

        // Check for limit switches while running
        checkMotors();
        
        // Also check for home command during movement
        if (checkForHomeCommand()) {
            // Home command received, stop all motors immediately
            Serial.println("HOME command received during movement - aborting movement");
            stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
            stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
            stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
            stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
            break; // Exit the wait loop
        }
        
        delay(5); // Reduced delay to check more frequently
    }
    
    if (!homeCommandReceived) {
        Serial.printf("Move complete - Position: X:%ld Y_L:%ld Y_R:%ld Z:%ld\n", stepperX->getCurrentPosition(), stepperY_Left->getCurrentPosition(), stepperY_Right->getCurrentPosition(), stepperZ->getCurrentPosition()); // Updated printf
    }
}

// New function that checks for home command during movement
// Returns true if movement completed, false if aborted due to home command
bool moveToXYZ_HomeCheck(long x, unsigned int xSpeed, long y, unsigned int ySpeed, long z, unsigned int zSpeed) {
    // Set speed for each stepper individually
    stepperX->setSpeedInHz(xSpeed);
    stepperY_Left->setSpeedInHz(ySpeed);
    stepperY_Right->setSpeedInHz(ySpeed);
    stepperZ->setSpeedInHz(zSpeed);
    
    // Command the move to the absolute position
    stepperX->moveTo(x);
    stepperY_Left->moveTo(y);
    stepperY_Right->moveTo(y);
    stepperZ->moveTo(z);
    
    // Wait until all steppers have completed their movements
    while (stepperX->isRunning() || stepperY_Left->isRunning() || stepperY_Right->isRunning() || stepperZ->isRunning()) {
        //! Handle Pause
        while (isPaused) {
            webSocket.loop();
            delay(100);
        }

        // Check for limit switches while running
        checkMotors();
        
        // Also check for home command during movement
        if (checkForHomeCommand()) {
            // Home command received, stop all motors immediately
            Serial.println("HOME command received during movement - aborting movement");
            stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
            stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
            stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
            stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
            return false; // Movement aborted
        }
        
        delay(5); // Reduced delay to check more frequently
    }
    
    Serial.printf("Move complete - Position: X:%ld Y_L:%ld Y_R:%ld Z:%ld\n", 
                 stepperX->getCurrentPosition(), 
                 stepperY_Left->getCurrentPosition(), 
                 stepperY_Right->getCurrentPosition(), 
                 stepperZ->getCurrentPosition());
    return true; // Movement completed successfully
}

void moveToXYZ_with_X_trigger(long target_x, unsigned int x_speed,
                              long target_y, unsigned int y_speed,
                              long target_z, unsigned int z_speed,
                              long trigger_x_pos, void (*trigger_action)()) {
    if (!stepperX || !stepperY_Left || !stepperY_Right || !stepperZ) {
        Serial.println("ERROR: Steppers not initialized in moveToXYZ_with_X_trigger");
        return;
    }

    stepperX->setSpeedInHz(x_speed);
    stepperY_Left->setSpeedInHz(y_speed);
    stepperY_Right->setSpeedInHz(y_speed); 
    stepperZ->setSpeedInHz(z_speed);
    
    stepperX->moveTo(target_x);
    stepperY_Left->moveTo(target_y);
    stepperY_Right->moveTo(target_y);
    stepperZ->moveTo(target_z);
    
    bool action_triggered = false;
    bool x_move_positive = (target_x > stepperX->getCurrentPosition());

    while ((stepperX && stepperX->isRunning()) ||
           (stepperY_Left && stepperY_Left->isRunning()) ||
           (stepperY_Right && stepperY_Right->isRunning()) ||
           (stepperZ && stepperZ->isRunning())) {

        while (isPaused) {
            webSocket.loop();
            delay(100);
        }

        checkMotors(); 

        if (checkForHomeCommand()) {
            Serial.println("HOME command received during movement - aborting movement (triggered)");
            stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
            stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
            stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
            stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
            return; // Abort function
        }

        if (stepperX && !action_triggered && trigger_action) {
            if (x_move_positive) {
                if (stepperX->getCurrentPosition() >= trigger_x_pos) {
                    trigger_action();
                    action_triggered = true;
                    Serial.println("X-trigger action performed (positive move).");
                }
            } else {
                if (stepperX->getCurrentPosition() <= trigger_x_pos) {
                    trigger_action();
                    action_triggered = true;
                    Serial.println("X-trigger action performed (negative move).");
                }
            }
        }
        delay(1); // Reduced delay for more responsive trigger checking, was 5ms
    }

    // Final check, in case the loop terminated exactly at the trigger point or trigger was slightly missed by discrete checks
    // This ensures the action is called if the final position itself qualifies for the trigger.
    if (stepperX && !action_triggered && trigger_action) {
        bool condition_met = (x_move_positive && stepperX->getCurrentPosition() >= trigger_x_pos) || 
                             (!x_move_positive && stepperX->getCurrentPosition() <= trigger_x_pos);
        // Also ensure we are at or past the trigger relative to the start, not just anywhere if target_x itself is trigger_x_pos and move was tiny
        // The check above should be sufficient if current position reflects the actual final stop position.
        if (condition_met) {
             trigger_action();
             action_triggered = true;
             Serial.println("X-trigger action performed (post-loop check).");
        }
    }

    if (!homeCommandReceived) { // Check homeCommandReceived as it might be set by checkForHomeCommand
      Serial.printf("Triggered move complete - Position: X:%ld Y_L:%ld Y_R:%ld Z:%ld\n", 
                   stepperX->getCurrentPosition(), 
                   stepperY_Left->getCurrentPosition(), 
                   stepperY_Right->getCurrentPosition(), 
                   stepperZ->getCurrentPosition());
    }
}

// This function replaces checkSwitches from Functionality.cpp
void checkMotors() {
    // Update debouncers
    debounceX.update();
    debounceY_Left.update(); // Renamed
    debounceY_Right.update(); // Added second Y debouncer update
    debounceZ.update();

    // Read switch states
    bool limitX = debounceX.read() == HIGH; // HIGH when triggered (assuming active high)
    bool limitY_Left = debounceY_Left.read() == HIGH; // Renamed & active high
    bool limitY_Right = debounceY_Right.read() == HIGH; // Added second Y switch read & active high
    bool limitZ = debounceZ.read() == HIGH; // active high
    
    // Example of using switch readings (add your own logic)
    if (limitX) {
        Serial.println("X limit switch triggered");
        // Take action like stopping X motor
        // stepperX->forceStop(); // Example action
    }
    
    // Similar handling for Y and Z
    if (limitY_Left) {
        Serial.println("Y Left limit switch triggered");
        // Take action like stopping Y motors during homing or if unexpected
        // stepperY_Left->forceStop(); // Example action
        // stepperY_Right->forceStop(); // Example action (if gantry safety requires stopping both)
    }
    if (limitY_Right) {
        Serial.println("Y Right limit switch triggered");
        // Take action like stopping Y motors during homing or if unexpected
        // stepperY_Left->forceStop(); // Example action (if gantry safety requires stopping both)
        // stepperY_Right->forceStop(); // Example action
    }
    if (limitZ) {
        Serial.println("Z limit switch triggered");
        // Take action like stopping Z motor
        // stepperZ->forceStop(); // Example action
    }
}

