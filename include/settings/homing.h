#ifndef SETTINGS_HOMING_H
#define SETTINGS_HOMING_H

// ==========================================================================
//                           HOMING PARAMETERS
// ==========================================================================

// --- Homing Speeds ---
#define HOMING_SPEED_X 2000            // X Homing speed
#define HOMING_SPEED_Y 2000            // Y Homing speed
#define HOMING_SPEED_Z 1000            // Z Homing speed

// --- Homing Accelerations ---
#define HOMING_ACCEL_X 12500           // X Homing acceleration
#define HOMING_ACCEL_Y 12500           // Y Homing acceleration
#define HOMING_ACCEL_Z 5000            // Z Homing acceleration
#define HOMING_MOVE_AWAY_ACCEL_X 3125  // X Homing move-away acceleration
#define HOMING_MOVE_AWAY_ACCEL_Y 3125  // Y Homing move-away acceleration
#define HOMING_MOVE_AWAY_ACCEL_Z 1250  // Z Homing move-away acceleration

// --- Homing Misc ---
#define HOMING_MOVE_AWAY_INCHES 0.2f             // Distance to move away from home switch (inches)
#define HOMING_TIMEOUT_MS 15000                  // Homing timeout (ms)

#endif // SETTINGS_HOMING_H 