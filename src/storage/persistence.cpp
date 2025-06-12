#include "storage/Persistence.h" // Updated include path

//* ************************************************************************
//* ************************* PERSISTENCE ***************************
//* ************************************************************************

Persistence persistence;

bool Persistence::isInitialized() {
    return preferences.getBool(INIT_FLAG_KEY, false);
}

void Persistence::saveFirstTimeFlag() {
    // Ensure preferences is open before writing
    // Always set the flag to true, regardless of whether it existed before.
    preferences.putBool(INIT_FLAG_KEY, true);
    Serial.println("Initialization flag set/confirmed");
}

bool Persistence::isKey(const char* key) {
    return preferences.isKey(key);
}

void Persistence::saveInt(const char* key, int value) {
    preferences.putInt(key, value);
    Serial.printf("Saved integer setting %s: %d\n", key, value);
}

int Persistence::loadInt(const char* key, int defaultValue) {
    if (!preferences.isKey(key)) {
        // First use, key doesn't exist yet, just use default
        return defaultValue;
    }
    int value = preferences.getInt(key, defaultValue);
    // Serial.printf("Loaded integer setting %s: %d\\n", key, value);
    return value;
}

void Persistence::saveFloat(const char* key, float value) {
    preferences.putFloat(key, value);
    Serial.printf("Saved float setting %s: %.2f\n", key, value);
}

float Persistence::loadFloat(const char* key, float defaultValue) {
    if (!preferences.isKey(key)) {
        // First use, key doesn't exist yet, just use default silently
        return defaultValue;
    }
    float value = preferences.getFloat(key, defaultValue);
    // Serial.printf("Loaded float setting %s: %.2f\\n", key, value);
    return value;
}

void Persistence::saveString(const char* key, const String& value) {
    preferences.putString(key, value);
    Serial.printf("Saved string setting %s: %s\n", key, value.c_str());
}

String Persistence::loadString(const char* key, const String& defaultValue) {
    if (!preferences.isKey(key)) {
        // First use, key doesn't exist yet, just use default
        return defaultValue;
    }
    String value = preferences.getString(key, defaultValue);
    // Serial.printf("Loaded string setting %s: %s\\n", key, value.c_str());
    return value;
}

void Persistence::saveBool(const char* key, bool value) {
    preferences.putBool(key, value);
    Serial.printf("Saved bool setting %s: %s\n", key, value ? "true" : "false");
}

bool Persistence::loadBool(const char* key, bool defaultValue) {
    if (!preferences.isKey(key)) {
        // First use, key doesn't exist yet, just use default
        return defaultValue;
    }
    bool value = preferences.getBool(key, defaultValue);
    // Serial.printf("Loaded bool setting %s: %s\\n", key, value ? "true" : "false");
    return value;
}

void Persistence::clearAll() {
    preferences.clear();
    Serial.println("All settings cleared");
}

void Persistence::beginTransaction(bool readOnly /*= false*/) {
    // Namespace is already member, readOnly controls mode
    preferences.begin(namespace_name, readOnly); 
    if (!readOnly) {
        Serial.println("NVS transaction begun (Read/Write).");
    } else {
        // Serial.println("NVS transaction begun (Read Only)."); // Optional: Less verbose for reads
    }
}

void Persistence::endTransaction() { // Renamed from commitChanges
    preferences.end(); // Close and commit (if opened R/W)
    Serial.println("NVS transaction ended/committed.");
} 