#include "storage/painting_settings.h"
#include "settings/painting.h"
#include "persistence/Persistence.h"

//* ************************************************************************
//* ************************* PAINTING SETTINGS ****************************
//* ************************************************************************

// Static variables to hold the settings (replacing class member variables)
static float paintingOffsetX = PAINTING_OFFSET_X;
static float paintingOffsetY = PAINTING_OFFSET_Y;

// Z Heights (Order: 1, 2, 3, 4)
static float side1ZHeight = SIDE1_Z_HEIGHT;
static float side2ZHeight = SIDE2_Z_HEIGHT;
static float side3ZHeight = SIDE3_Z_HEIGHT;
static float side4ZHeight = SIDE4_Z_HEIGHT;

// Side Z Heights (Order: 1, 2, 3, 4)
static float side1SideZHeight = SIDE1_SIDE_Z_HEIGHT;
static float side2SideZHeight = SIDE2_SIDE_Z_HEIGHT;
static float side3SideZHeight = SIDE3_SIDE_Z_HEIGHT;
static float side4SideZHeight = SIDE4_SIDE_Z_HEIGHT;

// Rotation Angles (Order: 1, 2, 3, 4)
static int side1RotationAngle = SIDE1_ROTATION_ANGLE;
static int side2RotationAngle = SIDE2_ROTATION_ANGLE;
static int side3RotationAngle = SIDE3_ROTATION_ANGLE;
static int side4RotationAngle = SIDE4_ROTATION_ANGLE;

// Servo Angles (Order: 1, 2, 3, 4) - Default to 35 as observed
static int servoAngleSide1 = 35;
static int servoAngleSide2 = 35;
static int servoAngleSide3 = 35;
static int servoAngleSide4 = 35;

// Painting Speeds (Order: 1, 2, 3, 4)
static int side1PaintingXSpeed = SIDE1_PAINTING_X_SPEED;
static int side1PaintingYSpeed = SIDE1_PAINTING_Y_SPEED;
static int side2PaintingXSpeed = SIDE2_PAINTING_X_SPEED;
static int side2PaintingYSpeed = SIDE2_PAINTING_Y_SPEED;
static int side3PaintingXSpeed = SIDE3_PAINTING_X_SPEED;
static int side3PaintingYSpeed = SIDE3_PAINTING_Y_SPEED;
static int side4PaintingXSpeed = SIDE4_PAINTING_X_SPEED;
static int side4PaintingYSpeed = SIDE4_PAINTING_Y_SPEED;

// Pattern Start Positions (Order: 1, 2, 3, 4)
static float side1StartX = SIDE1_START_X;
static float side1StartY = SIDE1_START_Y;
static float side2StartX = SIDE2_START_X;
static float side2StartY = SIDE2_START_Y;
static float side3StartX = SIDE3_START_X;
static float side3StartY = SIDE3_START_Y;
static float side4StartX = SIDE4_START_X;
static float side4StartY = SIDE4_START_Y;

// Pattern Dimensions (Order: 1, 2, 3, 4)
static float side1SweepY = SIDE1_SWEEP_Y;
static float side1ShiftX = SIDE1_SHIFT_X;
static float side2SweepY = SIDE2_SWEEP_Y;
static float side2ShiftX = SIDE2_SHIFT_X;
static float side3SweepY = SIDE3_SWEEP_Y;
static float side3ShiftX = SIDE3_SHIFT_X;
static float side4SweepY = SIDE4_SWEEP_Y;
static float side4ShiftX = SIDE4_SHIFT_X;

// Post-Print Pause
static int postPrintPause = 0; // Default pause after printing in milliseconds

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
#define KEY_POST_PRINT_PAUSE "pp_"
#define KEY_SERVO_ANGLE "srvAng_"

// Side identifiers for key construction
#define SIDE_1 "1"
#define SIDE_2 "2"
#define SIDE_3 "3"
#define SIDE_4 "4"

void initializePaintingSettings() {
    // Check if first time initialization is needed
    beginPersistenceTransaction(true); // Start read-only transaction
    bool firstTime = !isPersistenceInitialized();
    endPersistenceTransaction(); // End read-only transaction

    if (firstTime) {
        Serial.println("First-time initialization of painting settings with defaults");
        beginPersistenceTransaction(false); // Start read/write transaction
        resetPaintingSettingsToDefaults();
        savePaintingSettings(); // This will also save the settings within the transaction
        savePersistenceFirstTimeFlag(); // Mark as initialized within the transaction
        endPersistenceTransaction(); // End read/write transaction
        return;
    }
    
    // Check if migration is needed (only need to check one old key)
    beginPersistenceTransaction(true);
    bool migrationNeeded = isPersistenceKey("paint_offset_x");
    endPersistenceTransaction();

    if (migrationNeeded) {
        Serial.println("Detected old format keys - migration will be performed");
        beginPersistenceTransaction(false); // Need R/W for migration
        
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
        #define OLD_SIDE_2 "right"
        #define OLD_SIDE_3 "back"
        #define OLD_SIDE_4 "left"
        
        // Migrate each setting by reading from old key and storing in memory
        
        // Paint Gun Offsets
        paintingOffsetX = loadPersistenceFloat(OLD_KEY_OFFSET "x", PAINTING_OFFSET_X);
        paintingOffsetY = loadPersistenceFloat(OLD_KEY_OFFSET "y", PAINTING_OFFSET_Y);
        
        // Z Heights
        side1ZHeight = loadPersistenceFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_1, SIDE1_Z_HEIGHT);
        side2ZHeight = loadPersistenceFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_2, SIDE2_Z_HEIGHT);
        side3ZHeight = loadPersistenceFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_3, SIDE3_Z_HEIGHT);
        side4ZHeight = loadPersistenceFloat(OLD_KEY_Z_HEIGHT OLD_SIDE_4, SIDE4_Z_HEIGHT);
        
        // Side Z Heights
        side1SideZHeight = loadPersistenceFloat(OLD_KEY_SIDE_Z OLD_SIDE_1, SIDE1_SIDE_Z_HEIGHT);
        side2SideZHeight = loadPersistenceFloat(OLD_KEY_SIDE_Z OLD_SIDE_2, SIDE2_SIDE_Z_HEIGHT);
        side3SideZHeight = loadPersistenceFloat(OLD_KEY_SIDE_Z OLD_SIDE_3, SIDE3_SIDE_Z_HEIGHT);
        side4SideZHeight = loadPersistenceFloat(OLD_KEY_SIDE_Z OLD_SIDE_4, SIDE4_SIDE_Z_HEIGHT);
        
        // Rotation Angles
        side1RotationAngle = loadPersistenceInt(OLD_KEY_ROT_ANGLE OLD_SIDE_1, SIDE1_ROTATION_ANGLE);
        side2RotationAngle = loadPersistenceInt(OLD_KEY_ROT_ANGLE OLD_SIDE_2, SIDE2_ROTATION_ANGLE);
        side3RotationAngle = loadPersistenceInt(OLD_KEY_ROT_ANGLE OLD_SIDE_3, SIDE3_ROTATION_ANGLE);
        side4RotationAngle = loadPersistenceInt(OLD_KEY_ROT_ANGLE OLD_SIDE_4, SIDE4_ROTATION_ANGLE);
        
        // Painting Speeds
        side1PaintingXSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_1, SIDE1_PAINTING_X_SPEED);
        side1PaintingYSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_1, SIDE1_PAINTING_Y_SPEED);
        side2PaintingXSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_2, SIDE2_PAINTING_X_SPEED);
        side2PaintingYSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_2, SIDE2_PAINTING_Y_SPEED);
        side3PaintingXSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_3, SIDE3_PAINTING_X_SPEED);
        side3PaintingYSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_3, SIDE3_PAINTING_Y_SPEED);
        side4PaintingXSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_X OLD_SIDE_4, SIDE4_PAINTING_X_SPEED);
        side4PaintingYSpeed = loadPersistenceInt(OLD_KEY_PAINT_SPEED_Y OLD_SIDE_4, SIDE4_PAINTING_Y_SPEED);
        
        // Pattern Start Positions
        side1StartX = loadPersistenceFloat(OLD_KEY_START_X OLD_SIDE_1, SIDE1_START_X);
        side1StartY = loadPersistenceFloat(OLD_KEY_START_Y OLD_SIDE_1, SIDE1_START_Y);
        side2StartX = loadPersistenceFloat(OLD_KEY_START_X OLD_SIDE_2, SIDE2_START_X);
        side2StartY = loadPersistenceFloat(OLD_KEY_START_Y OLD_SIDE_2, SIDE2_START_Y);
        side3StartX = loadPersistenceFloat(OLD_KEY_START_X OLD_SIDE_3, SIDE3_START_X);
        side3StartY = loadPersistenceFloat(OLD_KEY_START_Y OLD_SIDE_3, SIDE3_START_Y);
        side4StartX = loadPersistenceFloat(OLD_KEY_START_X OLD_SIDE_4, SIDE4_START_X);
        side4StartY = loadPersistenceFloat(OLD_KEY_START_Y OLD_SIDE_4, SIDE4_START_Y);
        
        // Pattern Dimensions
        side1SweepY = loadPersistenceFloat(OLD_KEY_SWEEP_Y OLD_SIDE_1, SIDE1_SWEEP_Y);
        side1ShiftX = loadPersistenceFloat(OLD_KEY_SHIFT_X OLD_SIDE_1, SIDE1_SHIFT_X);
        side2SweepY = loadPersistenceFloat(OLD_KEY_SWEEP_Y OLD_SIDE_2, SIDE2_SWEEP_Y);
        side2ShiftX = loadPersistenceFloat(OLD_KEY_SHIFT_X OLD_SIDE_2, SIDE2_SHIFT_X);
        side3SweepY = loadPersistenceFloat(OLD_KEY_SWEEP_Y OLD_SIDE_3, SIDE3_SWEEP_Y);
        side3ShiftX = loadPersistenceFloat(OLD_KEY_SHIFT_X OLD_SIDE_3, SIDE3_SHIFT_X);
        side4SweepY = loadPersistenceFloat(OLD_KEY_SWEEP_Y OLD_SIDE_4, SIDE4_SWEEP_Y);
        side4ShiftX = loadPersistenceFloat(OLD_KEY_SHIFT_X OLD_SIDE_4, SIDE4_SHIFT_X);

        // Post-Print Pause (load with new key, assuming it didn't exist before or was 0)
        postPrintPause = loadPersistenceInt(KEY_POST_PRINT_PAUSE "val", 0); 
        
        // Save with new key format (inside the transaction)
        savePaintingSettings();
        
        endPersistenceTransaction(); // End read/write transaction
        
        Serial.println("Settings migration completed - old format keys have been migrated to new format");
    } else {
        // No migration needed, just load settings with new key format
        beginPersistenceTransaction(true); // Read-only needed here
        loadPaintingSettings();
        endPersistenceTransaction();
        Serial.println("Painting settings loaded from persistent storage (new format)");
    }
}

void loadPaintingSettings() {
    beginPersistenceTransaction(true); // Begin read-only transaction
    // Load using the new key format
    paintingOffsetX = loadPersistenceFloat(KEY_OFFSET "x", PAINTING_OFFSET_X);
    paintingOffsetY = loadPersistenceFloat(KEY_OFFSET "y", PAINTING_OFFSET_Y);
    
    side1ZHeight = loadPersistenceFloat(KEY_Z_HEIGHT SIDE_1, SIDE1_Z_HEIGHT);
    side2ZHeight = loadPersistenceFloat(KEY_Z_HEIGHT SIDE_2, SIDE2_Z_HEIGHT);
    side3ZHeight = loadPersistenceFloat(KEY_Z_HEIGHT SIDE_3, SIDE3_Z_HEIGHT);
    side4ZHeight = loadPersistenceFloat(KEY_Z_HEIGHT SIDE_4, SIDE4_Z_HEIGHT);
    
    side1SideZHeight = loadPersistenceFloat(KEY_SIDE_Z SIDE_1, SIDE1_SIDE_Z_HEIGHT);
    side2SideZHeight = loadPersistenceFloat(KEY_SIDE_Z SIDE_2, SIDE2_SIDE_Z_HEIGHT);
    side3SideZHeight = loadPersistenceFloat(KEY_SIDE_Z SIDE_3, SIDE3_SIDE_Z_HEIGHT);
    side4SideZHeight = loadPersistenceFloat(KEY_SIDE_Z SIDE_4, SIDE4_SIDE_Z_HEIGHT);
    
    side1RotationAngle = loadPersistenceInt(KEY_ROT_ANGLE SIDE_1, SIDE1_ROTATION_ANGLE);
    side2RotationAngle = loadPersistenceInt(KEY_ROT_ANGLE SIDE_2, SIDE2_ROTATION_ANGLE);
    side3RotationAngle = loadPersistenceInt(KEY_ROT_ANGLE SIDE_3, SIDE3_ROTATION_ANGLE);
    side4RotationAngle = loadPersistenceInt(KEY_ROT_ANGLE SIDE_4, SIDE4_ROTATION_ANGLE);
    
    side1PaintingXSpeed = loadPersistenceInt(KEY_PAINT_SPEED_X SIDE_1, SIDE1_PAINTING_X_SPEED);
    side1PaintingYSpeed = loadPersistenceInt(KEY_PAINT_SPEED_Y SIDE_1, SIDE1_PAINTING_Y_SPEED);
    side2PaintingXSpeed = loadPersistenceInt(KEY_PAINT_SPEED_X SIDE_2, SIDE2_PAINTING_X_SPEED);
    side2PaintingYSpeed = loadPersistenceInt(KEY_PAINT_SPEED_Y SIDE_2, SIDE2_PAINTING_Y_SPEED);
    side3PaintingXSpeed = loadPersistenceInt(KEY_PAINT_SPEED_X SIDE_3, SIDE3_PAINTING_X_SPEED);
    side3PaintingYSpeed = loadPersistenceInt(KEY_PAINT_SPEED_Y SIDE_3, SIDE3_PAINTING_Y_SPEED);
    side4PaintingXSpeed = loadPersistenceInt(KEY_PAINT_SPEED_X SIDE_4, SIDE4_PAINTING_X_SPEED);
    side4PaintingYSpeed = loadPersistenceInt(KEY_PAINT_SPEED_Y SIDE_4, SIDE4_PAINTING_Y_SPEED);
    
    side1StartX = loadPersistenceFloat(KEY_START_X SIDE_1, SIDE1_START_X);
    side1StartY = loadPersistenceFloat(KEY_START_Y SIDE_1, SIDE1_START_Y);
    side2StartX = loadPersistenceFloat(KEY_START_X SIDE_2, SIDE2_START_X);
    side2StartY = loadPersistenceFloat(KEY_START_Y SIDE_2, SIDE2_START_Y);
    side3StartX = loadPersistenceFloat(KEY_START_X SIDE_3, SIDE3_START_X);
    side3StartY = loadPersistenceFloat(KEY_START_Y SIDE_3, SIDE3_START_Y);
    side4StartX = loadPersistenceFloat(KEY_START_X SIDE_4, SIDE4_START_X);
    side4StartY = loadPersistenceFloat(KEY_START_Y SIDE_4, SIDE4_START_Y);
    
    side1SweepY = loadPersistenceFloat(KEY_SWEEP_Y SIDE_1, SIDE1_SWEEP_Y);
    side1ShiftX = loadPersistenceFloat(KEY_SHIFT_X SIDE_1, SIDE1_SHIFT_X);
    side2SweepY = loadPersistenceFloat(KEY_SWEEP_Y SIDE_2, SIDE2_SWEEP_Y);
    side2ShiftX = loadPersistenceFloat(KEY_SHIFT_X SIDE_2, SIDE2_SHIFT_X);
    side3SweepY = loadPersistenceFloat(KEY_SWEEP_Y SIDE_3, SIDE3_SWEEP_Y);
    side3ShiftX = loadPersistenceFloat(KEY_SHIFT_X SIDE_3, SIDE3_SHIFT_X);
    side4SweepY = loadPersistenceFloat(KEY_SWEEP_Y SIDE_4, SIDE4_SWEEP_Y);
    side4ShiftX = loadPersistenceFloat(KEY_SHIFT_X SIDE_4, SIDE4_SHIFT_X);

    // Load Post-Print Pause
    postPrintPause = loadPersistenceInt(KEY_POST_PRINT_PAUSE "val", 0);

    // Servo Angles
    servoAngleSide1 = loadPersistenceInt(KEY_SERVO_ANGLE SIDE_1, 35);
    servoAngleSide2 = loadPersistenceInt(KEY_SERVO_ANGLE SIDE_2, 35);
    servoAngleSide3 = loadPersistenceInt(KEY_SERVO_ANGLE SIDE_3, 35);
    servoAngleSide4 = loadPersistenceInt(KEY_SERVO_ANGLE SIDE_4, 35);

    endPersistenceTransaction(); // End read-only transaction
}

void savePaintingSettings() {
    beginPersistenceTransaction(false); // Begin read/write transaction
    // Save using the new key format
    savePersistenceFloat(KEY_OFFSET "x", paintingOffsetX);
    savePersistenceFloat(KEY_OFFSET "y", paintingOffsetY);
    
    savePersistenceFloat(KEY_Z_HEIGHT SIDE_1, side1ZHeight);
    savePersistenceFloat(KEY_Z_HEIGHT SIDE_2, side2ZHeight);
    savePersistenceFloat(KEY_Z_HEIGHT SIDE_3, side3ZHeight);
    savePersistenceFloat(KEY_Z_HEIGHT SIDE_4, side4ZHeight);
    
    savePersistenceFloat(KEY_SIDE_Z SIDE_1, side1SideZHeight);
    savePersistenceFloat(KEY_SIDE_Z SIDE_2, side2SideZHeight);
    savePersistenceFloat(KEY_SIDE_Z SIDE_3, side3SideZHeight);
    savePersistenceFloat(KEY_SIDE_Z SIDE_4, side4SideZHeight);
    
    savePersistenceInt(KEY_ROT_ANGLE SIDE_1, side1RotationAngle);
    savePersistenceInt(KEY_ROT_ANGLE SIDE_2, side2RotationAngle);
    savePersistenceInt(KEY_ROT_ANGLE SIDE_3, side3RotationAngle);
    savePersistenceInt(KEY_ROT_ANGLE SIDE_4, side4RotationAngle);
    
    savePersistenceInt(KEY_PAINT_SPEED_X SIDE_1, side1PaintingXSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_Y SIDE_1, side1PaintingYSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_X SIDE_2, side2PaintingXSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_Y SIDE_2, side2PaintingYSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_X SIDE_3, side3PaintingXSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_Y SIDE_3, side3PaintingYSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_X SIDE_4, side4PaintingXSpeed);
    savePersistenceInt(KEY_PAINT_SPEED_Y SIDE_4, side4PaintingYSpeed);
    
    savePersistenceFloat(KEY_START_X SIDE_1, side1StartX);
    savePersistenceFloat(KEY_START_Y SIDE_1, side1StartY);
    savePersistenceFloat(KEY_START_X SIDE_2, side2StartX);
    savePersistenceFloat(KEY_START_Y SIDE_2, side2StartY);
    savePersistenceFloat(KEY_START_X SIDE_3, side3StartX);
    savePersistenceFloat(KEY_START_Y SIDE_3, side3StartY);
    savePersistenceFloat(KEY_START_X SIDE_4, side4StartX);
    savePersistenceFloat(KEY_START_Y SIDE_4, side4StartY);
    
    savePersistenceFloat(KEY_SWEEP_Y SIDE_1, side1SweepY);
    savePersistenceFloat(KEY_SHIFT_X SIDE_1, side1ShiftX);
    savePersistenceFloat(KEY_SWEEP_Y SIDE_2, side2SweepY);
    savePersistenceFloat(KEY_SHIFT_X SIDE_2, side2ShiftX);
    savePersistenceFloat(KEY_SWEEP_Y SIDE_3, side3SweepY);
    savePersistenceFloat(KEY_SHIFT_X SIDE_3, side3ShiftX);
    savePersistenceFloat(KEY_SWEEP_Y SIDE_4, side4SweepY);
    savePersistenceFloat(KEY_SHIFT_X SIDE_4, side4ShiftX);

    // Save Servo Angles
    savePersistenceInt(KEY_SERVO_ANGLE SIDE_1, servoAngleSide1);
    savePersistenceInt(KEY_SERVO_ANGLE SIDE_2, servoAngleSide2);
    savePersistenceInt(KEY_SERVO_ANGLE SIDE_3, servoAngleSide3);
    savePersistenceInt(KEY_SERVO_ANGLE SIDE_4, servoAngleSide4);

    // Save Post-Print Pause
    savePersistenceInt(KEY_POST_PRINT_PAUSE "val", postPrintPause);
    endPersistenceTransaction(); // End read/write transaction
    Serial.println("Painting settings saved to NVS.");
}

void resetPaintingSettingsToDefaults() {
    // Reset all values to defaults from painting.h
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
    
    servoAngleSide1 = 35;
    servoAngleSide2 = 35;
    servoAngleSide3 = 35;
    servoAngleSide4 = 35;
    
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

    postPrintPause = 0;
}

// Getter and setter functions
float getPaintingOffsetX() { return paintingOffsetX; }
void setPaintingOffsetX(float value) { paintingOffsetX = value; }
float getPaintingOffsetY() { return paintingOffsetY; }
void setPaintingOffsetY(float value) { paintingOffsetY = value; }

float getSide1ZHeight() { return side1ZHeight; }
void setSide1ZHeight(float value) { side1ZHeight = value; }
float getSide2ZHeight() { return side2ZHeight; }
void setSide2ZHeight(float value) { side2ZHeight = value; }
float getSide3ZHeight() { return side3ZHeight; }
void setSide3ZHeight(float value) { side3ZHeight = value; }
float getSide4ZHeight() { return side4ZHeight; }
void setSide4ZHeight(float value) { side4ZHeight = value; }

float getSide1SideZHeight() { return side1SideZHeight; }
void setSide1SideZHeight(float value) { side1SideZHeight = value; }
float getSide2SideZHeight() { return side2SideZHeight; }
void setSide2SideZHeight(float value) { side2SideZHeight = value; }
float getSide3SideZHeight() { return side3SideZHeight; }
void setSide3SideZHeight(float value) { side3SideZHeight = value; }
float getSide4SideZHeight() { return side4SideZHeight; }
void setSide4SideZHeight(float value) { side4SideZHeight = value; }

int getSide1RotationAngle() { return side1RotationAngle; }
void setSide1RotationAngle(int value) { side1RotationAngle = value; }
int getSide2RotationAngle() { return side2RotationAngle; }
void setSide2RotationAngle(int value) { side2RotationAngle = value; }
int getSide3RotationAngle() { return side3RotationAngle; }
void setSide3RotationAngle(int value) { side3RotationAngle = value; }
int getSide4RotationAngle() { return side4RotationAngle; }
void setSide4RotationAngle(int value) { side4RotationAngle = value; }

int getServoAngleSide1() { return servoAngleSide1; }
void setServoAngleSide1(int value) { servoAngleSide1 = value; }
int getServoAngleSide2() { return servoAngleSide2; }
void setServoAngleSide2(int value) { servoAngleSide2 = value; }
int getServoAngleSide3() { return servoAngleSide3; }
void setServoAngleSide3(int value) { servoAngleSide3 = value; }
int getServoAngleSide4() { return servoAngleSide4; }
void setServoAngleSide4(int value) { servoAngleSide4 = value; }

int getSide1PaintingXSpeed() { return side1PaintingXSpeed; }
void setSide1PaintingXSpeed(int value) { side1PaintingXSpeed = value; }
int getSide1PaintingYSpeed() { return side1PaintingYSpeed; }
void setSide1PaintingYSpeed(int value) { side1PaintingYSpeed = value; }
int getSide2PaintingXSpeed() { return side2PaintingXSpeed; }
void setSide2PaintingXSpeed(int value) { side2PaintingXSpeed = value; }
int getSide2PaintingYSpeed() { return side2PaintingYSpeed; }
void setSide2PaintingYSpeed(int value) { side2PaintingYSpeed = value; }
int getSide3PaintingXSpeed() { return side3PaintingXSpeed; }
void setSide3PaintingXSpeed(int value) { side3PaintingXSpeed = value; }
int getSide3PaintingYSpeed() { return side3PaintingYSpeed; }
void setSide3PaintingYSpeed(int value) { side3PaintingYSpeed = value; }
int getSide4PaintingXSpeed() { return side4PaintingXSpeed; }
void setSide4PaintingXSpeed(int value) { side4PaintingXSpeed = value; }
int getSide4PaintingYSpeed() { return side4PaintingYSpeed; }
void setSide4PaintingYSpeed(int value) { side4PaintingYSpeed = value; }

float getSide1StartX() { return side1StartX; }
void setSide1StartX(float value) { side1StartX = value; }
float getSide1StartY() { return side1StartY; }
void setSide1StartY(float value) { side1StartY = value; }
float getSide2StartX() { return side2StartX; }
void setSide2StartX(float value) { side2StartX = value; }
float getSide2StartY() { return side2StartY; }
void setSide2StartY(float value) { side2StartY = value; }
float getSide3StartX() { return side3StartX; }
void setSide3StartX(float value) { side3StartX = value; }
float getSide3StartY() { return side3StartY; }
void setSide3StartY(float value) { side3StartY = value; }
float getSide4StartX() { return side4StartX; }
void setSide4StartX(float value) { side4StartX = value; }
float getSide4StartY() { return side4StartY; }
void setSide4StartY(float value) { side4StartY = value; }

float getSide1SweepY() { return side1SweepY; }
void setSide1SweepY(float value) { side1SweepY = value; }
float getSide1ShiftX() { return side1ShiftX; }
void setSide1ShiftX(float value) { side1ShiftX = value; }
float getSide2SweepY() { return side2SweepY; }
void setSide2SweepY(float value) { side2SweepY = value; }
float getSide2ShiftX() { return side2ShiftX; }
void setSide2ShiftX(float value) { side2ShiftX = value; }
float getSide3SweepY() { return side3SweepY; }
void setSide3SweepY(float value) { side3SweepY = value; }
float getSide3ShiftX() { return side3ShiftX; }
void setSide3ShiftX(float value) { side3ShiftX = value; }
float getSide4SweepY() { return side4SweepY; }
void setSide4SweepY(float value) { side4SweepY = value; }
float getSide4ShiftX() { return side4ShiftX; }
void setSide4ShiftX(float value) { side4ShiftX = value; }

int getPostPrintPause() { return postPrintPause; }
void setPostPrintPause(int value) { postPrintPause = value; } 