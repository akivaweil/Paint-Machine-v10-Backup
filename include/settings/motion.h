#ifndef SETTINGS_MOTION_H
#define SETTINGS_MOTION_H

#include <stdint.h> // Include for uint32_t type

// ==========================================================================
//                          MOTION & CONVERSION
// ==========================================================================

// --- Conversion Factors ---
#define STEPS_PER_INCH_XYZ 254.0f         // Steps per inch for XYZ axes
#define STEPS_PER_DEGREE 11.11111f        // Steps per degree for rotation axis
#define Y_STEPS_PER_INCH 1625.6f        // Y Steps per inch (example: 200 steps/rev * 8 microsteps / (pulley teeth * pitch))
#define Z_STEPS_PER_INCH 812.8f         // Z Steps per inch

// --- Physical Dimensions & Travel Limits (inches) ---
#define X_MAX_TRAVEL_POS_INCH 35.0f       // Maximum X travel
#define Y_MAX_TRAVEL_POS_INCH 35.0f       // Maximum Y travel
#define Z_MAX_TRAVEL_POS_INCH 2.75f       // Maximum Z travel downwards from home


// ==========================================================================
//                     DEFAULT MOTION PARAMETERS
// ==========================================================================
// Speed is in steps/sec (Hz), Accel is in steps/sec^2

// --- Default Speeds ---
#define DEFAULT_X_SPEED 20000          // Default X axis speed
#define DEFAULT_Y_SPEED 30000          // Default Y axis speed
#define DEFAULT_Z_SPEED 5000           // Default Z axis speed
#define DEFAULT_ROT_SPEED 2000         // Default rotation axis speed

// --- Default Accelerations ---
#define DEFAULT_X_ACCEL 25000          // Default X axis acceleration
#define DEFAULT_Y_ACCEL 30000          // Default Y axis acceleration
#define DEFAULT_Z_ACCEL 13000          // Default Z axis acceleration
#define DEFAULT_ROT_ACCEL 3000         // Default rotation axis acceleration

// --- PNP Speeds ---
#define DEFAULT_PNP_X_SPEED 20000     // Default PNP X axis speed
#define DEFAULT_PNP_Y_SPEED 30000     // Default PNP Y axis speed

// --- PNP Accelerations ---
#define DEFAULT_PNP_X_ACCEL 20000      // Default PNP X axis acceleration
#define DEFAULT_PNP_Y_ACCEL 30000      // Default PNP Y axis acceleration

#endif // SETTINGS_MOTION_H 