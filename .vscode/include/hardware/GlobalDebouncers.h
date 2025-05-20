#ifndef GLOBAL_DEBOUNCERS_H
#define GLOBAL_DEBOUNCERS_H

#include <Bounce2.h>

// Global debouncer for the PNP Cycle Sensor/Switch
extern Bounce g_pnpCycleSensorDebouncer;

// Add other global debouncers here if needed in the future

/**
 * @brief Initializes all global debouncer objects.
 * This function should be called once during setup.
 */
void initializeGlobalDebouncers();

#endif // GLOBAL_DEBOUNCERS_H 