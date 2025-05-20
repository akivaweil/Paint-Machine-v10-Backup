#include "storage/PaintingSettings.h" // Updated include path
#include "settings/painting.h" // Explicitly include defaults
// #include <Preferences.h> // REMOVED - No longer needed directly

//* ************************************************************************
//* ************************* PAINTING SETTINGS ****************************
//* ************************************************************************

// Global instance
PaintingSettings paintingSettings;

// Key prefixes for different setting categories
#define KEY_OFFSET "po_"
#define KEY_Z_HEIGHT "pz_"
#define KEY_SIDE_Z "ps_"
#define KEY_ROT_ANGLE "pr_"
#define KEY_PAINT_SPEED_X "px_"
#define KEY_PAINT_SPEED_Y "py_"
#define KEY_START_X "sx_"
#define KEY_START_Y "sy_"
#define KEY_SWEEP_Y "sw_"
#define KEY_SHIFT_X "sh_"
#define KEY_POST_PRINT_PAUSE "pp_" // Key for post print pause
#define KEY_SERVO_ANGLE "srvAng_" // Key prefix for servo angles

// Side identifiers for key construction
#define SIDE_1 "1"
#define SIDE_3 "3"
#define SIDE_4 "4"
#define SIDE_2 "2"

void PaintingSettings::begin() {
    // Check if first time initialization is needed
    persistence.beginTransaction(true); // Start read-only transaction
    bool firstTime = !persistence.isInitialized();
    persistence.endTransaction(); // End read-only transaction

    if (firstTime) {
        Serial.println("First-time initialization of painting settings with defaults");
        persistence.beginTransaction(false); // Start read/write transaction
        resetToDefaults();
        saveSettings(); // This will also save the settings within the transaction
        persistence.saveFirstTimeFlag(); // Mark as initialized within the transaction
        persistence.endTransaction(); // End read/write transaction
        return;
    }
    
    // Check if migration is needed (only need to check one old key)
    persistence.beginTransaction(true);
    bool migrationNeeded = persistence.isKey("paint_offset_x");
    persistence.endTransaction();

    if (migrationNeeded) {
        Serial.println("Detected old format keys - migration will be performed");
        persistence.beginTransaction(false); // Need R/W for migration
        
        // Perform migration - using original longer key format definitions
        #define OLD_KEY_OFFSET "paint_offset_"
        #define OLD_KEY_Z_HEIGHT "paint_z_"
        #define OLD_KEY_SIDE_Z "paint_side_z_"
        #define OLD_KEY_ROT_ANGLE "paint_rot_"
        #define OLD_KEY_PAINT_SPEED_X "paint_spd_x_"
        #define OLD_KEY_PAINT_SPEED_Y "paint_spd_y_"
        #define OLD_KEY_START_X "paint_start_x_"
        #define OLD_KEY_START_Y "paint_start_y_"
        #define OLD_KEY_SWEEP_Y "paint_sweep_y_"
        #define OLD_KEY_SHIFT_X "paint_shift_x_"
        
        #define OLD_SIDE_1 "front"
        #define OLD_SIDE_3 "back"
        #define OLD_SIDE_4 "left"
        #define OLD_SIDE_2 "right"
        
        // Migrate each setting by reading from old key and storing in memory
        
        // Paint Gun Offsets
        paintingOffsetX = persistence.loadFloat(OLD_KEY_OFFSET "x", PAINTING_OFFSET_X);
        paintingOffsetY = persistence.loadFloat(OLD_KEY_OFFSET "y", PAINTING_OFFSET_Y);
        
        // Z Heights
        side1ZHeight = persistence.loadFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_1, SIDE1_Z_HEIGHT);
        side2ZHeight = persistence.loadFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_2, SIDE2_Z_HEIGHT);
        side3ZHeight = persistence.loadFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_3, SIDE3_Z_HEIGHT);
        side4ZHeight = persistence.loadFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_4, SIDE4_Z_HEIGHT);
        
        // Side Z Heights
        side1SideZHeight = persistence.loadFloat(OLD_KEY_SIDE_Z OLD_SIDE_1, SIDE1_SIDE_Z_HEIGHT);
        side2SideZHeight = persistence.loadFloat(OLD_KEY_SIDE_Z OLD_SIDE_2, SIDE2_SIDE_Z_HEIGHT);
        side3SideZHeight = persistence.loadFloat(OLD_KEY_SIDE_Z OLD_SIDE_3, SIDE3_SIDE_Z_HEIGHT);
        side4SideZHeight = persistence.loadFloat(OLD_KEY_SIDE_Z OLD_SIDE_4, SIDE4_SIDE_Z_HEIGHT);
        
        // Rotation Angles
        side1RotationAngle = persistence.loadInt(OLD_KEY_ROT_ANGLE OLD_SIDE_1, SIDE1_ROTATION_ANGLE);
        side2RotationAngle = persistence.loadInt(OLD_KEY_ROT_ANGLE OLD_SIDE_2, SIDE2_ROTATION_ANGLE);
        side3RotationAngle = persistence.loadInt(OLD_KEY_ROT_ANGLE OLD_SIDE_3, SIDE3_ROTATION_ANGLE);
        side4RotationAngle = persistence.loadInt(OLD_KEY_ROT_ANGLE OLD_SIDE_4, SIDE4_ROTATION_ANGLE);
        
        // Painting Speeds
        side1PaintingXSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_1, SIDE1_PAINTING_X_SPEED);
        side1PaintingYSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_1, SIDE1_PAINTING_Y_SPEED);
        side2PaintingXSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_2, SIDE2_PAINTING_X_SPEED);
        side2PaintingYSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_2, SIDE2_PAINTING_Y_SPEED);
        side3PaintingXSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_3, SIDE3_PAINTING_X_SPEED);
        side3PaintingYSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_3, SIDE3_PAINTING_Y_SPEED);
        side4PaintingXSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_4, SIDE4_PAINTING_X_SPEED);
        side4PaintingYSpeed = persistence.loadInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_4, SIDE4_PAINTING_Y_SPEED);
        
        // Pattern Start Positions
        side1StartX = persistence.loadFloat(OLD_KEY_START_X OLD_SIDE_1, SIDE1_START_X);
        side1StartY = persistence.loadFloat(OLD_KEY_START_Y OLD_SIDE_1, SIDE1_START_Y);
        side2StartX = persistence.loadFloat(OLD_KEY_START_X OLD_SIDE_2, SIDE2_START_X);
        side2StartY = persistence.loadFloat(OLD_KEY_START_Y OLD_SIDE_2, SIDE2_START_Y);
        side3StartX = persistence.loadFloat(OLD_KEY_START_X OLD_SIDE_3, SIDE3_START_X);
        side3StartY = persistence.loadFloat(OLD_KEY_START_Y OLD_SIDE_3, SIDE3_START_Y);
        side4StartX = persistence.loadFloat(OLD_KEY_START_X OLD_SIDE_4, SIDE4_START_X);
        side4StartY = persistence.loadFloat(OLD_KEY_START_Y OLD_SIDE_4, SIDE4_START_Y);
        
        // Pattern Dimensions
        side1SweepY = persistence.loadFloat(OLD_KEY_SWEEP_Y OLD_SIDE_1, SIDE1_SWEEP_Y);
        side1ShiftX = persistence.loadFloat(OLD_KEY_SHIFT_X OLD_SIDE_1, SIDE1_SHIFT_X);
        side2SweepY = persistence.loadFloat(OLD_KEY_SWEEP_Y OLD_SIDE_2, SIDE2_SWEEP_Y);
        side2ShiftX = persistence.loadFloat(OLD_KEY_SHIFT_X OLD_SIDE_2, SIDE2_SHIFT_X);
        side3SweepY = persistence.loadFloat(OLD_KEY_SWEEP_Y OLD_SIDE_3, SIDE3_SWEEP_Y);
        side3ShiftX = persistence.loadFloat(OLD_KEY_SHIFT_X OLD_SIDE_3, SIDE3_SHIFT_X);
        side4SweepY = persistence.loadFloat(OLD_KEY_SWEEP_Y OLD_SIDE_4, SIDE4_SWEEP_Y);
        side4ShiftX = persistence.loadFloat(OLD_KEY_SHIFT_X OLD_SIDE_4, SIDE4_SHIFT_X);

        // Post-Print Pause (load with new key, assuming it didn't exist before or was 0)
        postPrintPause = persistence.loadInt(KEY_POST_PRINT_PAUSE "val", 0); 
        
        // Save with new key format (inside the transaction)
        saveSettings();
        
        // Don't remove old keys to avoid data loss risk, they just won't be read anymore.
        persistence.endTransaction(); // End read/write transaction
        
        Serial.println("Settings migration completed - old format keys have been migrated to new format");
    } else {
        // No migration needed, just load settings with new key format
        persistence.beginTransaction(true); // Read-only needed here
        loadSettings();
        persistence.endTransaction();
        Serial.println("Painting settings loaded from persistent storage (new format)");
    }
}

void PaintingSettings::loadSettings() {
    persistence.beginTransaction(true); // Begin read-only transaction
    // Load using the new key format
    paintingOffsetX = persistence.loadFloat(KEY_OFFSET "x", PAINTING_OFFSET_X);
    paintingOffsetY = persistence.loadFloat(KEY_OFFSET "y", PAINTING_OFFSET_Y);
    
    side1ZHeight = persistence.loadFloat(KEY_Z_HEIGHT SIDE_1, SIDE1_Z_HEIGHT);
    side2ZHeight = persistence.loadFloat(KEY_Z_HEIGHT SIDE_2, SIDE2_Z_HEIGHT);
    side3ZHeight = persistence.loadFloat(KEY_Z_HEIGHT SIDE_3, SIDE3_Z_HEIGHT);
    side4ZHeight = persistence.loadFloat(KEY_Z_HEIGHT SIDE_4, SIDE4_Z_HEIGHT);
    
    side1SideZHeight = persistence.loadFloat(KEY_SIDE_Z SIDE_1, SIDE1_SIDE_Z_HEIGHT);
    side2SideZHeight = persistence.loadFloat(KEY_SIDE_Z SIDE_2, SIDE2_SIDE_Z_HEIGHT);
    side3SideZHeight = persistence.loadFloat(KEY_SIDE_Z SIDE_3, SIDE3_SIDE_Z_HEIGHT);
    side4SideZHeight = persistence.loadFloat(KEY_SIDE_Z SIDE_4, SIDE4_SIDE_Z_HEIGHT);
    
    side1RotationAngle = persistence.loadInt(KEY_ROT_ANGLE SIDE_1, SIDE1_ROTATION_ANGLE);
    side2RotationAngle = persistence.loadInt(KEY_ROT_ANGLE SIDE_2, SIDE2_ROTATION_ANGLE);
    side3RotationAngle = persistence.loadInt(KEY_ROT_ANGLE SIDE_3, SIDE3_ROTATION_ANGLE);
    side4RotationAngle = persistence.loadInt(KEY_ROT_ANGLE SIDE_4, SIDE4_ROTATION_ANGLE);
    
    side1PaintingXSpeed = persistence.loadInt(KEY_PAINT_SPEED_X SIDE_1, SIDE1_PAINTING_X_SPEED);
    side1PaintingYSpeed = persistence.loadInt(KEY_PAINT_SPEED_Y SIDE_1, SIDE1_PAINTING_Y_SPEED);
    side2PaintingXSpeed = persistence.loadInt(KEY_PAINT_SPEED_X SIDE_2, SIDE2_PAINTING_X_SPEED);
    side2PaintingYSpeed = persistence.loadInt(KEY_PAINT_SPEED_Y SIDE_2, SIDE2_PAINTING_Y_SPEED);
    side3PaintingXSpeed = persistence.loadInt(KEY_PAINT_SPEED_X SIDE_3, SIDE3_PAINTING_X_SPEED);
    side3PaintingYSpeed = persistence.loadInt(KEY_PAINT_SPEED_Y SIDE_3, SIDE3_PAINTING_Y_SPEED);
    side4PaintingXSpeed = persistence.loadInt(KEY_PAINT_SPEED_X SIDE_4, SIDE4_PAINTING_X_SPEED);
    side4PaintingYSpeed = persistence.loadInt(KEY_PAINT_SPEED_Y SIDE_4, SIDE4_PAINTING_Y_SPEED);
    
    side1StartX = persistence.loadFloat(KEY_START_X SIDE_1, SIDE1_START_X);
    side1StartY = persistence.loadFloat(KEY_START_Y SIDE_1, SIDE1_START_Y);
    side2StartX = persistence.loadFloat(KEY_START_X SIDE_2, SIDE2_START_X);
    side2StartY = persistence.loadFloat(KEY_START_Y SIDE_2, SIDE2_START_Y);
    side3StartX = persistence.loadFloat(KEY_START_X SIDE_3, SIDE3_START_X);
    side3StartY = persistence.loadFloat(KEY_START_Y SIDE_3, SIDE3_START_Y);
    side4StartX = persistence.loadFloat(KEY_START_X SIDE_4, SIDE4_START_X);
    side4StartY = persistence.loadFloat(KEY_START_Y SIDE_4, SIDE4_START_Y);
    
    side1SweepY = persistence.loadFloat(KEY_SWEEP_Y SIDE_1, SIDE1_SWEEP_Y);
    side1ShiftX = persistence.loadFloat(KEY_SHIFT_X SIDE_1, SIDE1_SHIFT_X);
    side2SweepY = persistence.loadFloat(KEY_SWEEP_Y SIDE_2, SIDE2_SWEEP_Y);
    side2ShiftX = persistence.loadFloat(KEY_SHIFT_X SIDE_2, SIDE2_SHIFT_X);
    side3SweepY = persistence.loadFloat(KEY_SWEEP_Y SIDE_3, SIDE3_SWEEP_Y);
    side3ShiftX = persistence.loadFloat(KEY_SHIFT_X SIDE_3, SIDE3_SHIFT_X);
    side4SweepY = persistence.loadFloat(KEY_SWEEP_Y SIDE_4, SIDE4_SWEEP_Y);
    side4ShiftX = persistence.loadFloat(KEY_SHIFT_X SIDE_4, SIDE4_SHIFT_X);

    // Load Post-Print Pause
    postPrintPause = persistence.loadInt(KEY_POST_PRINT_PAUSE "val", 0); // Use a simple key like "pp_val"

    // Servo Angles
    servoAngleSide1 = persistence.loadInt(KEY_SERVO_ANGLE SIDE_1, 35); // Default 35 if not found
    servoAngleSide2 = persistence.loadInt(KEY_SERVO_ANGLE SIDE_2, 35);
    servoAngleSide3 = persistence.loadInt(KEY_SERVO_ANGLE SIDE_3, 35);
    servoAngleSide4 = persistence.loadInt(KEY_SERVO_ANGLE SIDE_4, 35);

    persistence.endTransaction(); // End read-only transaction
}

void PaintingSettings::saveSettings() {
    persistence.beginTransaction(false); // Begin read/write transaction
    // Save using the new key format
    persistence.saveFloat(KEY_OFFSET "x", paintingOffsetX);
    persistence.saveFloat(KEY_OFFSET "y", paintingOffsetY);
    
    persistence.saveFloat(KEY_Z_HEIGHT SIDE_1, side1ZHeight);
    persistence.saveFloat(KEY_Z_HEIGHT SIDE_2, side2ZHeight);
    persistence.saveFloat(KEY_Z_HEIGHT SIDE_3, side3ZHeight);
    persistence.saveFloat(KEY_Z_HEIGHT SIDE_4, side4ZHeight);
    
    persistence.saveFloat(KEY_SIDE_Z SIDE_1, side1SideZHeight);
    persistence.saveFloat(KEY_SIDE_Z SIDE_2, side2SideZHeight);
    persistence.saveFloat(KEY_SIDE_Z SIDE_3, side3SideZHeight);
    persistence.saveFloat(KEY_SIDE_Z SIDE_4, side4SideZHeight);
    
    persistence.saveInt(KEY_ROT_ANGLE SIDE_1, side1RotationAngle);
    persistence.saveInt(KEY_ROT_ANGLE SIDE_2, side2RotationAngle);
    persistence.saveInt(KEY_ROT_ANGLE SIDE_3, side3RotationAngle);
    persistence.saveInt(KEY_ROT_ANGLE SIDE_4, side4RotationAngle);
    
    persistence.saveInt(KEY_PAINT_SPEED_X SIDE_1, side1PaintingXSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_Y SIDE_1, side1PaintingYSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_X SIDE_2, side2PaintingXSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_Y SIDE_2, side2PaintingYSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_X SIDE_3, side3PaintingXSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_Y SIDE_3, side3PaintingYSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_X SIDE_4, side4PaintingXSpeed);
    persistence.saveInt(KEY_PAINT_SPEED_Y SIDE_4, side4PaintingYSpeed);
    
    persistence.saveFloat(KEY_START_X SIDE_1, side1StartX);
    persistence.saveFloat(KEY_START_Y SIDE_1, side1StartY);
    persistence.saveFloat(KEY_START_X SIDE_2, side2StartX);
    persistence.saveFloat(KEY_START_Y SIDE_2, side2StartY);
    persistence.saveFloat(KEY_START_X SIDE_3, side3StartX);
    persistence.saveFloat(KEY_START_Y SIDE_3, side3StartY);
    persistence.saveFloat(KEY_START_X SIDE_4, side4StartX);
    persistence.saveFloat(KEY_START_Y SIDE_4, side4StartY);
    
    persistence.saveFloat(KEY_SWEEP_Y SIDE_1, side1SweepY);
    persistence.saveFloat(KEY_SHIFT_X SIDE_1, side1ShiftX);
    persistence.saveFloat(KEY_SWEEP_Y SIDE_2, side2SweepY);
    persistence.saveFloat(KEY_SHIFT_X SIDE_2, side2ShiftX);
    persistence.saveFloat(KEY_SWEEP_Y SIDE_3, side3SweepY);
    persistence.saveFloat(KEY_SHIFT_X SIDE_3, side3ShiftX);
    persistence.saveFloat(KEY_SWEEP_Y SIDE_4, side4SweepY);
    persistence.saveFloat(KEY_SHIFT_X SIDE_4, side4ShiftX);

    // Save Servo Angles
    persistence.saveInt(KEY_SERVO_ANGLE SIDE_1, servoAngleSide1);
    persistence.saveInt(KEY_SERVO_ANGLE SIDE_2, servoAngleSide2);
    persistence.saveInt(KEY_SERVO_ANGLE SIDE_3, servoAngleSide3);
    persistence.saveInt(KEY_SERVO_ANGLE SIDE_4, servoAngleSide4);

    // Save Post-Print Pause
    persistence.saveInt(KEY_POST_PRINT_PAUSE "val", postPrintPause);
    persistence.endTransaction(); // End read/write transaction
    Serial.println("Painting settings saved to NVS."); // Optional: Confirmation
}

void PaintingSettings::resetToDefaults() {
    // Reset all member variables to the default values from painting.h
    paintingOffsetX = PAINTING_OFFSET_X;
    paintingOffsetY = PAINTING_OFFSET_Y;
    
    side1ZHeight = SIDE1_Z_HEIGHT;
    side2ZHeight = SIDE2_Z_HEIGHT;
    side3ZHeight = SIDE3_Z_HEIGHT;
    side4ZHeight = SIDE4_Z_HEIGHT;
    
    side1SideZHeight = SIDE1_SIDE_Z_HEIGHT;
    side2SideZHeight = SIDE2_SIDE_Z_HEIGHT;
    side3SideZHeight = SIDE3_SIDE_Z_HEIGHT;
    side4SideZHeight = SIDE4_SIDE_Z_HEIGHT;
    
    side1RotationAngle = SIDE1_ROTATION_ANGLE;
    side2RotationAngle = SIDE2_ROTATION_ANGLE;
    side3RotationAngle = SIDE3_ROTATION_ANGLE;
    side4RotationAngle = SIDE4_ROTATION_ANGLE;
    
    side1PaintingXSpeed = SIDE1_PAINTING_X_SPEED;
    side1PaintingYSpeed = SIDE1_PAINTING_Y_SPEED;
    side2PaintingXSpeed = SIDE2_PAINTING_X_SPEED;
    side2PaintingYSpeed = SIDE2_PAINTING_Y_SPEED;
    side3PaintingXSpeed = SIDE3_PAINTING_X_SPEED;
    side3PaintingYSpeed = SIDE3_PAINTING_Y_SPEED;
    side4PaintingXSpeed = SIDE4_PAINTING_X_SPEED;
    side4PaintingYSpeed = SIDE4_PAINTING_Y_SPEED;
    
    side1StartX = SIDE1_START_X;
    side1StartY = SIDE1_START_Y;
    side2StartX = SIDE2_START_X;
    side2StartY = SIDE2_START_Y;
    side3StartX = SIDE3_START_X;
    side3StartY = SIDE3_START_Y;
    side4StartX = SIDE4_START_X;
    side4StartY = SIDE4_START_Y;
    
    side1SweepY = SIDE1_SWEEP_Y;
    side1ShiftX = SIDE1_SHIFT_X;
    side2SweepY = SIDE2_SWEEP_Y;
    side2ShiftX = SIDE2_SHIFT_X;
    side3SweepY = SIDE3_SWEEP_Y;
    side3ShiftX = SIDE3_SHIFT_X;
    side4SweepY = SIDE4_SWEEP_Y;
    side4ShiftX = SIDE4_SHIFT_X;

    // Reset Servo Angles
    servoAngleSide1 = 35;
    servoAngleSide2 = 35;
    servoAngleSide3 = 35;
    servoAngleSide4 = 35;

    postPrintPause = 0; // Reset post-print pause to 0 (or defined default)
}

// --- Getters ---
float PaintingSettings::getPaintingOffsetX() { return paintingOffsetX; }
float PaintingSettings::getPaintingOffsetY() { return paintingOffsetY; }

float PaintingSettings::getSide1ZHeight() { return side1ZHeight; }
float PaintingSettings::getSide2ZHeight() { return side2ZHeight; }
float PaintingSettings::getSide3ZHeight() { return side3ZHeight; }
float PaintingSettings::getSide4ZHeight() { return side4ZHeight; }

float PaintingSettings::getSide1SideZHeight() { return side1SideZHeight; }
float PaintingSettings::getSide2SideZHeight() { return side2SideZHeight; }
float PaintingSettings::getSide3SideZHeight() { return side3SideZHeight; }
float PaintingSettings::getSide4SideZHeight() { return side4SideZHeight; }

int PaintingSettings::getSide1RotationAngle() { return side1RotationAngle; }
int PaintingSettings::getSide2RotationAngle() { return side2RotationAngle; }
int PaintingSettings::getSide3RotationAngle() { return side3RotationAngle; }
int PaintingSettings::getSide4RotationAngle() { return side4RotationAngle; }

int PaintingSettings::getSide1PaintingXSpeed() { return side1PaintingXSpeed; }
int PaintingSettings::getSide1PaintingYSpeed() { return side1PaintingYSpeed; }
int PaintingSettings::getSide2PaintingXSpeed() { return side2PaintingXSpeed; }
int PaintingSettings::getSide2PaintingYSpeed() { return side2PaintingYSpeed; }
int PaintingSettings::getSide3PaintingXSpeed() { return side3PaintingXSpeed; }
int PaintingSettings::getSide3PaintingYSpeed() { return side3PaintingYSpeed; }
int PaintingSettings::getSide4PaintingXSpeed() { return side4PaintingXSpeed; }
int PaintingSettings::getSide4PaintingYSpeed() { return side4PaintingYSpeed; }

float PaintingSettings::getSide1StartX() { return side1StartX; }
float PaintingSettings::getSide1StartY() { return side1StartY; }
float PaintingSettings::getSide2StartX() { return side2StartX; }
float PaintingSettings::getSide2StartY() { return side2StartY; }
float PaintingSettings::getSide3StartX() { return side3StartX; }
float PaintingSettings::getSide3StartY() { return side3StartY; }
float PaintingSettings::getSide4StartX() { return side4StartX; }
float PaintingSettings::getSide4StartY() { return side4StartY; }

float PaintingSettings::getSide1SweepY() { return side1SweepY; }
float PaintingSettings::getSide1ShiftX() { return side1ShiftX; }
float PaintingSettings::getSide2SweepY() { return side2SweepY; }
float PaintingSettings::getSide2ShiftX() { return side2ShiftX; }
float PaintingSettings::getSide3SweepY() { return side3SweepY; }
float PaintingSettings::getSide3ShiftX() { return side3ShiftX; }
float PaintingSettings::getSide4SweepY() { return side4SweepY; }
float PaintingSettings::getSide4ShiftX() { return side4ShiftX; }

int PaintingSettings::getPostPrintPause() { return postPrintPause; }

// Servo Angle Getters
int PaintingSettings::getServoAngleSide1() { return servoAngleSide1; }
int PaintingSettings::getServoAngleSide2() { return servoAngleSide2; }
int PaintingSettings::getServoAngleSide3() { return servoAngleSide3; }
int PaintingSettings::getServoAngleSide4() { return servoAngleSide4; }

// --- Setters ---
// Note: Setters only update the value in memory. Call saveSettings() to persist.
void PaintingSettings::setPaintingOffsetX(float value) { paintingOffsetX = value; }
void PaintingSettings::setPaintingOffsetY(float value) { paintingOffsetY = value; }

void PaintingSettings::setSide1ZHeight(float value) { side1ZHeight = value; }
void PaintingSettings::setSide2ZHeight(float value) { side2ZHeight = value; }
void PaintingSettings::setSide3ZHeight(float value) { side3ZHeight = value; }
void PaintingSettings::setSide4ZHeight(float value) { side4ZHeight = value; }

void PaintingSettings::setSide1SideZHeight(float value) { side1SideZHeight = value; }
void PaintingSettings::setSide2SideZHeight(float value) { side2SideZHeight = value; }
void PaintingSettings::setSide3SideZHeight(float value) { side3SideZHeight = value; }
void PaintingSettings::setSide4SideZHeight(float value) { side4SideZHeight = value; }

void PaintingSettings::setSide1RotationAngle(int value) { side1RotationAngle = value; }
void PaintingSettings::setSide2RotationAngle(int value) { side2RotationAngle = value; }
void PaintingSettings::setSide3RotationAngle(int value) { side3RotationAngle = value; }
void PaintingSettings::setSide4RotationAngle(int value) { side4RotationAngle = value; }

void PaintingSettings::setSide1PaintingXSpeed(int value) { side1PaintingXSpeed = value; }
void PaintingSettings::setSide1PaintingYSpeed(int value) { side1PaintingYSpeed = value; }
void PaintingSettings::setSide2PaintingXSpeed(int value) { side2PaintingXSpeed = value; }
void PaintingSettings::setSide2PaintingYSpeed(int value) { side2PaintingYSpeed = value; }
void PaintingSettings::setSide3PaintingXSpeed(int value) { side3PaintingXSpeed = value; }
void PaintingSettings::setSide3PaintingYSpeed(int value) { side3PaintingYSpeed = value; }
void PaintingSettings::setSide4PaintingXSpeed(int value) { side4PaintingXSpeed = value; }
void PaintingSettings::setSide4PaintingYSpeed(int value) { side4PaintingYSpeed = value; }

void PaintingSettings::setSide1StartX(float value) { side1StartX = value; }
void PaintingSettings::setSide1StartY(float value) { side1StartY = value; }
void PaintingSettings::setSide2StartX(float value) { side2StartX = value; }
void PaintingSettings::setSide2StartY(float value) { side2StartY = value; }
void PaintingSettings::setSide3StartX(float value) { side3StartX = value; }
void PaintingSettings::setSide3StartY(float value) { side3StartY = value; }
void PaintingSettings::setSide4StartX(float value) { side4StartX = value; }
void PaintingSettings::setSide4StartY(float value) { side4StartY = value; }

void PaintingSettings::setSide1SweepY(float value) { side1SweepY = value; }
void PaintingSettings::setSide1ShiftX(float value) { side1ShiftX = value; }
void PaintingSettings::setSide2SweepY(float value) { side2SweepY = value; }
void PaintingSettings::setSide2ShiftX(float value) { side2ShiftX = value; }
void PaintingSettings::setSide3SweepY(float value) { side3SweepY = value; }
void PaintingSettings::setSide3ShiftX(float value) { side3ShiftX = value; }
void PaintingSettings::setSide4SweepY(float value) { side4SweepY = value; }
void PaintingSettings::setSide4ShiftX(float value) { side4ShiftX = value; }

void PaintingSettings::setPostPrintPause(int value) { postPrintPause = value; }

// Servo Angle Setters
void PaintingSettings::setServoAngleSide1(int value) { servoAngleSide1 = value; }
void PaintingSettings::setServoAngleSide2(int value) { servoAngleSide2 = value; }
void PaintingSettings::setServoAngleSide3(int value) { servoAngleSide3 = value; }
void PaintingSettings::setServoAngleSide4(int value) { servoAngleSide4 = value; } 