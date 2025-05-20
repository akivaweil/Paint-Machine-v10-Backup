#ifndef ROTATION_MOTOR_H
#define ROTATION_MOTOR_H

#include <FastAccelStepper.h>
#include <Arduino.h>
#include "settings.h" // Include settings to get the defines

// // Define GPIO pins for the rotation motor - MOVED TO settings.h
// // !!! UPDATE THESE VALUES IF THEY ARE DIFFERENT !!!
// #define ROTATION_STEP_PIN 25 // Example pin
// #define ROTATION_DIR_PIN 26  // Example pin

// // Define default speed and acceleration for the rotation motor - MOVED TO settings.h
// // !!! UPDATE THESE VALUES AS NEEDED !!!
// #define DEFAULT_ROT_SPEED 5000 // Example speed in Hz
// #define DEFAULT_ROT_ACCEL 10000 // Example acceleration in steps/s^2


// Declare the rotation stepper motor object pointer
extern FastAccelStepper *rotationStepper;

/**
 * @brief Initializes the rotation stepper motor.
 * Connects the stepper to its pins using the provided engine instance.
 * Sets default speed and acceleration.
 * @param engine The FastAccelStepperEngine instance.
 */
void setupRotationMotor(FastAccelStepperEngine& engine);

/**
 * @brief Rotates to a specific angle.
 * @param angle The target angle in degrees.
 */
void rotateToAngle(float angle);

// Add any other rotation-specific functions here if needed in the future
// e.g., void rotateToAngle(float angle);
// e.g., void setRotationSpeed(uint32_t speed);

#endif // ROTATION_MOTOR_H 