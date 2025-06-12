#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#include <Arduino.h>
#include <Preferences.h>

//* ************************************************************************
//* ************************* PERSISTENCE *********************************
//* ************************************************************************

// Transaction Management
void beginPersistenceTransaction(bool readOnly = false);
void endPersistenceTransaction();

// First-time initialization check
bool isPersistenceInitialized();
void savePersistenceFirstTimeFlag();

// Check if a key exists
bool isPersistenceKey(const char* key);

// General methods
void savePersistenceInt(const char* key, int value);
int loadPersistenceInt(const char* key, int defaultValue);

void savePersistenceFloat(const char* key, float value);
float loadPersistenceFloat(const char* key, float defaultValue);

void savePersistenceString(const char* key, const String& value);
String loadPersistenceString(const char* key, const String& defaultValue);

void savePersistenceBool(const char* key, bool value);
bool loadPersistenceBool(const char* key, bool defaultValue);

// Clear all settings
void clearAllPersistence();

// Define keys for settings
const char* const SERVO_ANGLE_SIDE1_KEY = "srvAng1";
const char* const SERVO_ANGLE_SIDE2_KEY = "srvAng2";
const char* const SERVO_ANGLE_SIDE3_KEY = "srvAng3";
const char* const SERVO_ANGLE_SIDE4_KEY = "srvAng4";

// Keys for other painting settings
const char* const PAINT_SPEED_KEY = "pntSpd";
const char* const EDGE_OFFSET_KEY = "edgOff";
const char* const Z_CLEARANCE_KEY = "zClr";
const char* const X_OVERLAP_KEY = "xOvr";

#endif // PERSISTENCE_H 