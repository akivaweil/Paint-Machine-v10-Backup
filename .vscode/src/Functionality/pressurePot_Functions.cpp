#include <Arduino.h>
#include "utils/settings.h"
#include "hardware/pressurePot_Functions.h"
// #include "web_command_adapter.h" // Removing this include
#include <WebSocketsServer.h>

// External reference to the WebSocket instance in webserver.cpp
extern WebSocketsServer webSocket;
bool isPressurePot_ON = false;

// Function declaration from removed web_command_adapter.h
extern void sendWebStatus(WebSocketsServer* webSocket, const char* message);

void PressurePot_OFF() {
    pinMode(PRESSURE_POT_PIN, OUTPUT);
    digitalWrite(PRESSURE_POT_PIN, LOW);
    Serial.println("Pressure pot OFF");

    isPressurePot_ON = false;
    
    // Send status update to web clients to sync UI toggle
    sendWebStatus(&webSocket, "PRESSURE_POT_STATUS:OFF");
}

void PressurePot_ON() {
    pinMode(PRESSURE_POT_PIN, OUTPUT);
    digitalWrite(PRESSURE_POT_PIN, HIGH);
    Serial.println("Pressure pot ON"); 

    isPressurePot_ON = true;
    
    // Send status update to web clients to sync UI toggle
    sendWebStatus(&webSocket, "PRESSURE_POT_STATUS:ON");
}

/* REMOVED - Logic moved to Setup.cpp
void setupPressurePot() {
    Serial.println("Setting up Pressure Pot...");
    pinMode(PRESSURE_POT_PIN, OUTPUT);
    digitalWrite(PRESSURE_POT_PIN, LOW); // Ensure it's off initially
}
*/