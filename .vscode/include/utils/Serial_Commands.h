#ifndef SERIAL_COMMANDS_H
#define SERIAL_COMMANDS_H

#include <Arduino.h>
#include "system/machine_state.h"
#include "utils/settings.h"

// Initialize serial communication
void setupSerial();

// Process incoming serial commands
void processSerialCommands();

// Helper functions
void printHelp();
void executeCommand(String command);

#endif // SERIAL_COMMANDS_H 