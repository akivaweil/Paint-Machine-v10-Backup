#include <Arduino.h>
#include "utils/settings.h" // Include settings for pin definitions
#include "hardware/paintGun_Functions.h" // Corrected path
// #include "web_command_adapter.h" // Removing this include
#include <WebSocketsServer.h>

// External reference to the WebSocket instance in webserver.cpp
extern WebSocketsServer webSocket;

// Track the paint gun state
bool isPaintGun_ON = false;

// Function Implementations
/* REMOVED - Logic moved to Setup.cpp
void setupPaintGun() {
    Serial.println("Setting up Paint Gun...");
    pinMode(PAINT_GUN_PIN, OUTPUT);
    digitalWrite(PAINT_GUN_PIN, LOW); // Ensure it's off initially
}
*/

// Moved from web_command_adapter.cpp
void sendWebStatus(WebSocketsServer* webSocket, const char* message) {
    if (webSocket) {
        // Basic implementation: broadcast the message to all connected clients
        webSocket->broadcastTXT(message);
        Serial.print("Sent to WebSocket: "); // Debug output
        Serial.println(message);
    }
}

void paintGun_OFF() {
    pinMode(PAINT_GUN_PIN, OUTPUT);
    digitalWrite(PAINT_GUN_PIN, LOW);
    Serial.println("Paint gun OFF");
    Serial.printf("Paint gun pin %d set to LOW\n", PAINT_GUN_PIN);
    
    // Update the state
    isPaintGun_ON = false;
    
    // Send status update to web clients to sync UI toggle
    sendWebStatus(&webSocket, "PAINT_GUN_STATUS:OFF");
}

void paintGun_ON() {
    pinMode(PAINT_GUN_PIN, OUTPUT);
    digitalWrite(PAINT_GUN_PIN, HIGH);
    Serial.println("Paint gun ON");
    Serial.printf("Paint gun pin %d set to HIGH\n", PAINT_GUN_PIN);
    
    // Update the state
    isPaintGun_ON = true;
    
    // Send status update to web clients to sync UI toggle
    sendWebStatus(&webSocket, "PAINT_GUN_STATUS:ON");
}