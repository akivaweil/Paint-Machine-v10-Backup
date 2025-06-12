#ifndef PERSISTENCE_H
#define PERSISTENCE_H

#include <Arduino.h>
#include <Preferences.h>

//* ************************************************************************
//* ************************* PERSISTENCE *********************************
//* ************************************************************************

class Persistence {
public:
    Preferences preferences;
    const char* namespace_name = "paintmach"; // Namespace for Preferences library (max 15 chars)
    const char* INIT_FLAG_KEY = "initialized"; // Key for initialization flag

public:
    // Initialize settings manager
    // void begin();
    
    // First-time initialization check
    bool isInitialized();
    void saveFirstTimeFlag();
    
    // Check if a key exists
    bool isKey(const char* key);
    
    // General methods
    void saveInt(const char* key, int value);
    int loadInt(const char* key, int defaultValue);
    
    void saveFloat(const char* key, float value);
    float loadFloat(const char* key, float defaultValue);
    
    void saveString(const char* key, const String& value);
    String loadString(const char* key, const String& defaultValue);
    
    void saveBool(const char* key, bool value);
    bool loadBool(const char* key, bool defaultValue);
    
    // Clear all settings
    void clearAll();
    
    // Commit changes to NVS
    void commitChanges();

    // Close preferences (call when done)
    // void end();
};

// Global persistence instance
extern Persistence persistence;

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