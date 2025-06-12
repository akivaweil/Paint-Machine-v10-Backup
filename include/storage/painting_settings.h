#ifndef PAINTING_SETTINGS_H
#define PAINTING_SETTINGS_H

#include <Arduino.h>
#include "settings/painting.h"
#include "storage/persistence.h"

//* ************************************************************************
//* ************************* PAINTING SETTINGS ****************************
//* ************************************************************************

// Initialize painting settings system
void initializePaintingSettings();

// Load settings from non-volatile memory
void loadPaintingSettings();

// Save current settings to non-volatile memory
void savePaintingSettings();

// Reset to default values from painting.h
void resetPaintingSettingsToDefaults();

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

#endif // PAINTING_SETTINGS_H 