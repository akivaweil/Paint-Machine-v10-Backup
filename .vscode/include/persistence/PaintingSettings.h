#ifndef PAINTING_SETTINGS_H
#define PAINTING_SETTINGS_H

#include <Arduino.h>
#include "settings/painting.h"
#include "persistence/Persistence.h"

//* ************************************************************************
//* ************************* PAINTING SETTINGS ****************************
//* ************************************************************************

class PaintingSettings {
private:
    // Default values are defined in painting.h
    
    // Paint Gun Offsets
    float paintingOffsetX = PAINTING_OFFSET_X;
    float paintingOffsetY = PAINTING_OFFSET_Y;
    
    // Z Heights (Order: 1, 2, 3, 4)
    float side1ZHeight = SIDE1_Z_HEIGHT;
    float side2ZHeight = SIDE2_Z_HEIGHT;
    float side3ZHeight = SIDE3_Z_HEIGHT;
    float side4ZHeight = SIDE4_Z_HEIGHT;
    
    // Side Z Heights (Order: 1, 2, 3, 4)
    float side1SideZHeight = SIDE1_SIDE_Z_HEIGHT;
    float side2SideZHeight = SIDE2_SIDE_Z_HEIGHT;
    float side3SideZHeight = SIDE3_SIDE_Z_HEIGHT;
    float side4SideZHeight = SIDE4_SIDE_Z_HEIGHT;
    
    // Rotation Angles (Order: 1, 2, 3, 4)
    int side1RotationAngle = SIDE1_ROTATION_ANGLE;
    int side2RotationAngle = SIDE2_ROTATION_ANGLE;
    int side3RotationAngle = SIDE3_ROTATION_ANGLE;
    int side4RotationAngle = SIDE4_ROTATION_ANGLE;
    
    // Servo Angles (Order: 1, 2, 3, 4) - Default to 35 as observed
    int servoAngleSide1 = 35; 
    int servoAngleSide2 = 35;
    int servoAngleSide3 = 35;
    int servoAngleSide4 = 35;
    
    // Painting Speeds (Order: 1, 2, 3, 4)
    int side1PaintingXSpeed = SIDE1_PAINTING_X_SPEED;
    int side1PaintingYSpeed = SIDE1_PAINTING_Y_SPEED;
    int side2PaintingXSpeed = SIDE2_PAINTING_X_SPEED;
    int side2PaintingYSpeed = SIDE2_PAINTING_Y_SPEED;
    int side3PaintingXSpeed = SIDE3_PAINTING_X_SPEED;
    int side3PaintingYSpeed = SIDE3_PAINTING_Y_SPEED;
    int side4PaintingXSpeed = SIDE4_PAINTING_X_SPEED;
    int side4PaintingYSpeed = SIDE4_PAINTING_Y_SPEED;
    
    // Pattern Start Positions (Order: 1, 2, 3, 4)
    float side1StartX = SIDE1_START_X;
    float side1StartY = SIDE1_START_Y;
    float side2StartX = SIDE2_START_X;
    float side2StartY = SIDE2_START_Y;
    float side3StartX = SIDE3_START_X;
    float side3StartY = SIDE3_START_Y;
    float side4StartX = SIDE4_START_X;
    float side4StartY = SIDE4_START_Y;
    
    // Pattern Dimensions (Order: 1, 2, 3, 4)
    float side1SweepY = SIDE1_SWEEP_Y;
    float side1ShiftX = SIDE1_SHIFT_X;
    float side2SweepY = SIDE2_SWEEP_Y;
    float side2ShiftX = SIDE2_SHIFT_X;
    float side3SweepY = SIDE3_SWEEP_Y;
    float side3ShiftX = SIDE3_SHIFT_X;
    float side4SweepY = SIDE4_SWEEP_Y;
    float side4ShiftX = SIDE4_SHIFT_X;

    // Post-Print Pause
    int postPrintPause = 0; // Default pause after printing in milliseconds

public:
    // Initialize with defaults or load saved values
    void begin();
    
    // Load settings from non-volatile memory
    void loadSettings();
    
    // Save current settings to non-volatile memory
    void saveSettings();
    
    // Reset to default values from painting.h
    void resetToDefaults();
    
    // Getters and setters for each setting
    
    // Paint Gun Offsets
    float getPaintingOffsetX();
    void setPaintingOffsetX(float value);
    
    float getPaintingOffsetY();
    void setPaintingOffsetY(float value);
    
    // Z Heights (Order: 1, 2, 3, 4)
    float getSide1ZHeight();
    void setSide1ZHeight(float value);
    float getSide2ZHeight();
    void setSide2ZHeight(float value);
    float getSide3ZHeight();
    void setSide3ZHeight(float value);
    float getSide4ZHeight();
    void setSide4ZHeight(float value);
    
    // Side Z Heights (Order: 1, 2, 3, 4)
    float getSide1SideZHeight();
    void setSide1SideZHeight(float value);
    float getSide2SideZHeight();
    void setSide2SideZHeight(float value);
    float getSide3SideZHeight();
    void setSide3SideZHeight(float value);
    float getSide4SideZHeight();
    void setSide4SideZHeight(float value);
    
    // Rotation Angles (Order: 1, 2, 3, 4)
    int getSide1RotationAngle();
    void setSide1RotationAngle(int value);
    int getSide2RotationAngle();
    void setSide2RotationAngle(int value);
    int getSide3RotationAngle();
    void setSide3RotationAngle(int value);
    int getSide4RotationAngle();
    void setSide4RotationAngle(int value);
    
    // Servo Angles (Order: 1, 2, 3, 4)
    int getServoAngleSide1();
    void setServoAngleSide1(int value);
    int getServoAngleSide2();
    void setServoAngleSide2(int value);
    int getServoAngleSide3();
    void setServoAngleSide3(int value);
    int getServoAngleSide4();
    void setServoAngleSide4(int value);
    
    // Painting Speeds (Order: 1, 2, 3, 4)
    int getSide1PaintingXSpeed();
    void setSide1PaintingXSpeed(int value);
    int getSide1PaintingYSpeed();
    void setSide1PaintingYSpeed(int value);
    int getSide2PaintingXSpeed();
    void setSide2PaintingXSpeed(int value);
    int getSide2PaintingYSpeed();
    void setSide2PaintingYSpeed(int value);
    int getSide3PaintingXSpeed();
    void setSide3PaintingXSpeed(int value);
    int getSide3PaintingYSpeed();
    void setSide3PaintingYSpeed(int value);
    int getSide4PaintingXSpeed();
    void setSide4PaintingXSpeed(int value);
    int getSide4PaintingYSpeed();
    void setSide4PaintingYSpeed(int value);
    
    // Pattern Start Positions (Order: 1, 2, 3, 4)
    float getSide1StartX();
    void setSide1StartX(float value);
    float getSide1StartY();
    void setSide1StartY(float value);
    float getSide2StartX();
    void setSide2StartX(float value);
    float getSide2StartY();
    void setSide2StartY(float value);
    float getSide3StartX();
    void setSide3StartX(float value);
    float getSide3StartY();
    void setSide3StartY(float value);
    float getSide4StartX();
    void setSide4StartX(float value);
    float getSide4StartY();
    void setSide4StartY(float value);
    
    // Pattern Dimensions (Order: 1, 2, 3, 4)
    float getSide1SweepY();
    void setSide1SweepY(float value);
    float getSide1ShiftX();
    void setSide1ShiftX(float value);
    float getSide2SweepY();
    void setSide2SweepY(float value);
    float getSide2ShiftX();
    void setSide2ShiftX(float value);
    float getSide3SweepY();
    void setSide3SweepY(float value);
    float getSide3ShiftX();
    void setSide3ShiftX(float value);
    float getSide4SweepY();
    void setSide4SweepY(float value);
    float getSide4ShiftX();
    void setSide4ShiftX(float value);
    
    // Post-Print Pause
    int getPostPrintPause();
    void setPostPrintPause(int value);
};

// Global instance
extern PaintingSettings paintingSettings;

#endif // PAINTING_SETTINGS_H 