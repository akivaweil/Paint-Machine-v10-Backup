#include "hardware/GlobalDebouncers.h"
#include "settings/pins.h" // For PNP_CYCLE_SENSOR_PIN
#include <Arduino.h>       // For INPUT, pinMode

// Definition of the global PNP Cycle Sensor debouncer
Bounce g_pnpCycleSensorDebouncer;

/**
 * @brief Initializes all global debouncer objects.
 * Configures pin modes and attaches debouncers.
 */
void initializeGlobalDebouncers() {
    // PNP_CYCLE_SENSOR_PIN is an active LOW IR sensor.
    // Use INPUT_PULLUP so the pin is HIGH when idle and LOW when active.
    // Bounce2's attach(pin, mode) sets the pinMode internally.
    g_pnpCycleSensorDebouncer.attach(PNP_CYCLE_SENSOR_PIN, INPUT_PULLUP);
    g_pnpCycleSensorDebouncer.interval(20); // Debounce interval of 20 milliseconds
}