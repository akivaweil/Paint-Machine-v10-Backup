#ifndef DEBOUNCE_SETTINGS_H
#define DEBOUNCE_SETTINGS_H

// General debounce interval for limit switches, buttons, etc.
const unsigned long GENERAL_DEBOUNCE_MS = 10;

// Specific debounce interval for homing switches
const unsigned long HOMING_SWITCH_DEBOUNCE_MS = 20; // Was 3ms, then 20ms, keeping 20ms

// Specific debounce interval for the PNP Cycle Sensor
const unsigned long PNP_CYCLE_SENSOR_DEBOUNCE_MS = 40;

#endif // DEBOUNCE_SETTINGS_H 