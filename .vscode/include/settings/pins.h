#ifndef SETTINGS_PINS_H
#define SETTINGS_PINS_H

// ==========================================================================
//                            PIN DEFINITIONS
// ==========================================================================

// --- Stepper Motor Pins ---
// X Axis
#define X_STEP_PIN 48
#define X_DIR_PIN 45
#define X_HOME_SWITCH 18

// Y Axis Left
#define Y_LEFT_STEP_PIN 39
#define Y_LEFT_DIR_PIN 40
#define Y_LEFT_HOME_SWITCH 9

// Y Axis Right
#define Y_RIGHT_STEP_PIN 41
#define Y_RIGHT_DIR_PIN 42
#define Y_RIGHT_HOME_SWITCH 15

// Z Axis
#define Z_STEP_PIN 35
#define Z_DIR_PIN 36
#define Z_HOME_SWITCH 46

// Rotation Axis (Optional Homing)
#define ROTATION_STEP_PIN 37
#define ROTATION_DIR_PIN 38

// --- Servo & Actuator Pins ---
// Pitch Servo Removed
#define PICK_CYLINDER_PIN 12
#define SUCTION_PIN 10
#define PAINT_GUN_PIN 11
#define PRESSURE_POT_PIN 13

// --- Control Input Pins ---
#define PNP_CYCLE_SENSOR_PIN 21

#endif // SETTINGS_PINS_H 