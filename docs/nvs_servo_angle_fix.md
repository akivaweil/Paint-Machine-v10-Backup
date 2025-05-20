# NVS Servo Angle Persistence Issue Fix

## Problem Description

The servo angle settings configured via the web dashboard were not persisting correctly. Although the `SET_SERVO_ANGLE_SIDEX` command appeared to save the value to NVS (Non-Volatile Storage), retrieving the settings after a page refresh or device reboot would show the default value (35) instead of the saved value.

## Root Causes

Several factors contributed to this issue:

1.  **Inconsistent Settings Management:** The `PaintingSettings` class, responsible for loading and saving most painting parameters to/from NVS, did not include variables or logic to handle the individual servo angles (`servoAngleSide1` to `servoAngleSide4`). The NVS keys used for saving servo angles individually (`srvAng_1`, etc.) were different from the main `saveSettings` logic.
2.  **Hardcoded Initialization:** The `setup()` function in `src/Main/Main.cpp` initialized the servo motor using a hardcoded value (`int initialServoAngle = 35;`) instead of retrieving the value loaded from NVS by the `PaintingSettings` system.
3.  **Incorrect Retrieval in Web Command:** The `GET_PAINT_SETTINGS` command handler in `src/Commands/Web_Dashboard_Commands.cpp` attempted to retrieve the servo angles by directly calling `persistence.loadInt()` with incorrect NVS keys (`SERVO_ANGLE_SIDEX_KEY`), bypassing the `PaintingSettings` object entirely. This meant it wasn't reading the values saved by the `SET_SERVO_ANGLE_SIDEX` command or the main `saveSettings` function.
4.  **Build/Cache Issues:** Initial compilation attempts failed after applying fixes, indicating potential stale build artifacts or include path resolution problems within the PlatformIO environment.

## Solution Implemented

1.  **Integrated Servo Angles into `PaintingSettings`:**
    *   Added `servoAngleSide1` through `servoAngleSide4` private member variables to `include/storage/PaintingSettings.h`, initialized to the default `35`.
    *   Added corresponding public `getServoAngleSideX()` and `setServoAngleSideX()` methods to the header.
    *   Defined a new key prefix `#define KEY_SERVO_ANGLE "srvAng_"` in `src/storage/PaintingSettings.cpp`.
    *   Updated `loadSettings()` in `PaintingSettings.cpp` to load values using `persistence.loadInt(KEY_SERVO_ANGLE SIDEX, 35)`.
    *   Updated `saveSettings()` in `PaintingSettings.cpp` to save values using `persistence.saveInt(KEY_SERVO_ANGLE SIDEX, servoAngleSideX)`.
    *   Updated `resetToDefaults()` in `PaintingSettings.cpp` to reset servo angles to `35`.
    *   Implemented the getter and setter functions in `PaintingSettings.cpp`.

2.  **Corrected Servo Initialization:**
    *   Modified `setup()` in `src/Main/Main.cpp` to initialize the servo using the loaded setting: `int initialServoAngle = paintingSettings.getServoAngleSide1();`.

3.  **Corrected Web Command Handlers:**
    *   Modified the `SET_SERVO_ANGLE_SIDEX` handlers in `src/Commands/Web_Dashboard_Commands.cpp` to use `paintingSettings.setServoAngleSideX(angle);` followed by `paintingSettings.saveSettings();`, removing the direct `persistence.saveInt` calls.
    *   Modified the `GET_PAINT_SETTINGS` handler in `src/Commands/Web_Dashboard_Commands.cpp` to retrieve servo angles using `paintingSettings.getServoAngleSideX()` instead of direct `persistence.loadInt` calls.

4.  **Resolved Build Issues:**
    *   Executed `pio run --target clean` to remove old build files before recompiling and uploading.

This ensures that servo angles are consistently managed within the `PaintingSettings` object, loaded correctly on startup, saved appropriately when changed, and retrieved accurately for the web dashboard. 