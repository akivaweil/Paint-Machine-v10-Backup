#include "motors/Rotation_Motor.h"
#include "utils/settings.h"
#include "system/GlobalState.h"
#include <WebSocketsServer.h>

// Define the global rotation stepper pointer
FastAccelStepper *rotationStepper = NULL;
extern WebSocketsServer webSocket;

//* ************************************************************************
//* ************************* ROTATION MOTOR *************************
//* ************************************************************************

void setupRotationMotor(FastAccelStepperEngine& engine) {
    Serial.println("Initializing Rotation Stepper...");
    rotationStepper = engine.stepperConnectToPin(ROTATION_STEP_PIN);
    if (rotationStepper) {
        rotationStepper->setDirectionPin(ROTATION_DIR_PIN);
        rotationStepper->setSpeedInHz(DEFAULT_ROT_SPEED);
        rotationStepper->setAcceleration(DEFAULT_ROT_ACCEL);
        Serial.println("Rotation stepper initialized successfully.");
    } else {
        Serial.println("Failed to initialize Rotation stepper!");
        // Handle error appropriately, maybe halt or log
    }
}

/**
 * Rotates the turntable to a specific angle
 * @param angle The target angle in degrees (0-360)
 */
void rotateToAngle(float angle) {
    // Check if rotation stepper is initialized
    if (!rotationStepper) {
        Serial.println("ERROR: Rotation stepper not initialized!");
        return;
    }

    // Get current position and angle
    long currentPosition = rotationStepper->getCurrentPosition();
    float currentAngle = (float)currentPosition / STEPS_PER_DEGREE;

    // Calculate the difference in angle
    float deltaAngle = angle - currentAngle;

    // Normalize the delta angle to the range [-180, 180] to find the shortest path
    while (deltaAngle > 180.0f) deltaAngle -= 360.0f;
    while (deltaAngle <= -180.0f) deltaAngle += 360.0f;

    // Calculate the relative steps to move
    long relativeSteps = (long)(deltaAngle * STEPS_PER_DEGREE);

    // Calculate the target absolute angle for logging purposes (optional, but helpful)
    float targetAngleNormalized = currentAngle + deltaAngle;

    Serial.printf("Current Angle: %.2f, Target Angle: %.2f, Delta: %.2f degrees\n", currentAngle, angle, deltaAngle);
    Serial.printf("Rotating by %ld steps (relative) to reach %.2f degrees\n", relativeSteps, targetAngleNormalized);

    // Set speed and acceleration (optional, can be set once during setup if constant)
    rotationStepper->setSpeedInHz(DEFAULT_ROT_SPEED);
    rotationStepper->setAcceleration(DEFAULT_ROT_ACCEL); // Ensure acceleration is set

    // Add stopMove() before move()
    rotationStepper->stopMove(); 
    Serial.println("DEBUG: Called stopMove() before rotationStepper->move()");

    // Move the stepper relatively
    rotationStepper->move(relativeSteps);

    // Wait for rotation to complete
    while (rotationStepper->isRunning()) {
        //! Handle Pause
        while (isPaused) {
            webSocket.loop(); // Keep WebSocket responsive
            delay(100);       // Small delay to prevent busy-waiting
        }
        // Yield or delay briefly to allow background tasks and prevent busy-waiting
        delay(10); 
    }
    
    // Recalculate final angle based on actual final position for accuracy
    long finalPosition = rotationStepper->getCurrentPosition();
    float finalAngle = (float)finalPosition / STEPS_PER_DEGREE;
    Serial.printf("Rotation complete - Final Position: %ld steps (%.2f degrees)\n", finalPosition, finalAngle);
}

// Implement other rotation-specific functions here if needed 