#include "utils/settings.h"
#include <Arduino.h>

//* ************************************************************************
//* ************************* VACUUM CONTROL ***************************
//* ************************************************************************

/**
 * @brief Turn on the vacuum
 * 
 * This function activates the vacuum by setting the pin HIGH
 * */
void vacuumOn() {
    digitalWrite(SUCTION_PIN, HIGH);
}

/**
 * @brief Turn off the vacuum
 * 
 * This function deactivates the vacuum by setting the pin LOW
 */
void vacuumOff() {
    digitalWrite(SUCTION_PIN, LOW);
}

/* REMOVED - Logic moved to Setup.cpp
 * @brief Initialize vacuum system
 * 
 * Configure the vacuum control pin and ensure it starts OFF.
 
void initializeVacuum() {
    pinMode(SUCTION_PIN, OUTPUT);
    vacuumOff(); // Start with vacuum off
} 
*/ 