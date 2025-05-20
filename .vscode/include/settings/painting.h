#ifndef SETTINGS_PAINTING_H
#define SETTINGS_PAINTING_H

// ==========================================================================
//                          PAINTING PARAMETERS
// ==========================================================================
// Defaults, can be changed via UI/saved settings

// --- Paint Gun Offsets (inches) ---
#define PAINTING_OFFSET_X -1.5f                // Paint gun X offset relative to tool center
#define PAINTING_OFFSET_Y -4.5f                // Paint gun Y offset relative to tool center

// --- Default Paint Z Heights (inches, relative to homed Z=0) -- (Order: 1, 2, 3, 4)
#define SIDE1_Z_HEIGHT -2.0f                   // Default Z height for side 1 pattern painting
#define SIDE2_Z_HEIGHT -1.0f                   // Default Z height for side 2 pattern painting
#define SIDE3_Z_HEIGHT -1.5f                   // Default Z height for side 3 pattern painting
#define SIDE4_Z_HEIGHT -1.0f                   // Default Z height for side 4 pattern painting

// --- Additional Side Z Heights (inches) --- (Order: 1, 2, 3, 4)
#define SIDE1_SIDE_Z_HEIGHT 0.0f               // Z height for side 1. Can move between 0 and -2.75 inches
#define SIDE2_SIDE_Z_HEIGHT 0.0f               // Z height for side 2. Can move between 0 and -2.75 inches
#define SIDE3_SIDE_Z_HEIGHT 0.0f                // Z height for side 3. Can move between 0 and -2.75 inches
#define SIDE4_SIDE_Z_HEIGHT 0.0f                // Z height for side 4. Can move between 0 and -2.75 inches

// --- Rotation Angles (degrees) ---
#define SIDE1_ROTATION_ANGLE 180                  // Default rotation angle for side 1 (front)
#define SIDE2_ROTATION_ANGLE -90               // Default rotation angle for side 2 (right)
#define SIDE3_ROTATION_ANGLE 0                // Default rotation angle for side 3 (back)
#define SIDE4_ROTATION_ANGLE 90                // Default rotation angle for side 4 (left)

// --- Default Painting Speeds (steps/sec) --- (Order: 1, 2, 3, 4)
// Side 1 speeds
#define SIDE1_PAINTING_X_SPEED 10000           // X axis speed during side 1 pattern painting (Shift Speed)
#define SIDE1_PAINTING_Y_SPEED 20000            // Y axis speed during side 1 pattern painting (Sweep Speed)
// Side 2 speeds
#define SIDE2_PAINTING_X_SPEED 20000           // X axis speed during side 2 pattern painting (Shift Speed)
#define SIDE2_PAINTING_Y_SPEED 10000           // Y axis speed during side 2 pattern painting (Sweep Speed)
// Side 3 speeds
#define SIDE3_PAINTING_X_SPEED 5000            // X axis speed during side 3 pattern painting (Now UI Sweep Speed)
#define SIDE3_PAINTING_Y_SPEED 20000            // Y axis speed during side 3 pattern painting (Now UI Shift Speed)
// Side 4 speeds
#define SIDE4_PAINTING_X_SPEED 20000          // X axis speed during side 4 pattern painting (Shift Speed)
#define SIDE4_PAINTING_Y_SPEED 10000           // Y axis speed during side 4 pattern painting (Sweep Speed)

// --- Pattern Start Positions (inches) --- (Order: 1, 2, 3, 4)
#define SIDE1_START_X 7.00f                    // Side 1 pattern starting X position
#define SIDE1_START_Y 3.00f                    // Side 1 pattern starting Y position
#define SIDE2_START_X 27.0f                    // Side 2 pattern starting X position
#define SIDE2_START_Y 23.5f                    // Side 2 pattern starting Y position
#define SIDE3_START_X 26.50f                   // Side 3 pattern starting X position
#define SIDE3_START_Y 21.50f                   // Side 3 pattern starting Y position
#define SIDE4_START_X 6.20f                    // Side 4 pattern starting X position
#define SIDE4_START_Y 1.5f                     // Side 4 pattern starting Y position

// --- Pattern Dimensions (inches) --- (Order: 1, 2, 3, 4)
#define SIDE1_SWEEP_Y 20.00f                   // Side 1 pattern Y sweep distance
#define SIDE1_SHIFT_X 5.00f                    // Side 1 pattern X shift distance
#define SIDE2_SWEEP_Y 20.00f                   // Side 2 pattern Y sweep distance
#define SIDE2_SHIFT_X 5.00f                    // Side 2 pattern X shift distance
#define SIDE3_SWEEP_Y 19.00f                   // Side 3 pattern Y sweep distance (Now UI Shift Distance)
#define SIDE3_SHIFT_X 5.00f                    // Side 3 pattern X shift distance (Now UI Sweep Distance)
#define SIDE4_SWEEP_Y 20.00f                   // Side 4 pattern Y sweep distance
#define SIDE4_SHIFT_X 5.00f                    // Side 4 pattern X shift distance

// Post-Print Pause
#define DEFAULT_POST_PRINT_PAUSE 0 // milliseconds

// Number of Paint Coats
#define DEFAULT_PAINT_COATS 3 // Number of coats

// Delay Between Coats
#define DEFAULT_COAT_DELAY_MS 10000 // Milliseconds (e.g., 30 seconds)

#endif // SETTINGS_PAINTING_H 