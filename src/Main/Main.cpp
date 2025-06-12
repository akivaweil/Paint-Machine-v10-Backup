#include <Arduino.h>
// #include "Main.h" // REMOVED - File not found
#include "core/Setup.h"
#include "utils/machine_state.h"
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include "system/StateMachine.h"  // Now in include directory!
#include "motors/servo_motor.h"
#include "storage/persistence.h"
#include "storage/painting_settings.h"

// Include headers for functions called in loop
#include "web/Web_Dashboard_Commands.h" // For runDashboardServer()

// Function-based state machine (no class needed)

extern WebSocketsServer webSocket;
extern bool webSocketServerStarted;

// Global variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 10;
const int servoPin = 4;

// Removed ServoMotor class - using function-based approach
// Removed extern PaintingSettings - using function-based approach

// Define the global flag previously in machine_state.cpp
volatile bool homeCommandReceived = false;

// Add global flag for immediate command execution (remove volatile for String)
bool immediateCommandPending = false;
String pendingCommand = "";
uint8_t pendingCommandClientNum = 0;

// Forward declaration for immediate command processing
void processImmediateCommand();

//* ************************************************************************
//* ***************************** MAIN *******************************
//* ************************************************************************

void setup() {
  // Initialize function-based state machine
  initializeSystem();
  initializeStateMachine();
  setupWebDashboardCommands(); // Initialize pins and settings for web commands
  
  // Initialize servo after settings are loaded
  int initialServoAngle = getServoAngleSide1(); // Get initial angle from loaded settings
  initServoMotor(servoPin, initialServoAngle);
  Serial.printf("Servo Initialized at: %d degrees\n", initialServoAngle);

  // Any setup code that *must* run after initializeSystem()
  Serial.println("Setup complete. Enhanced immediate command processing ready!");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // **REVOLUTIONARY CHANGE**: Process WebSocket events MULTIPLE times per loop
  runDashboardServer(); // Now processes WebSocket events 15+ times per call!
  
  // Update function-based state machine with immediate command processing
  updateStateMachine();
  
  // **ADDITIONAL WebSocket processing** after state update for maximum responsiveness
  webSocket.loop();
  webSocket.loop();
  
  // **ENHANCED ARCHITECTURE**: Main loop now processes WebSocket events 17+ times per iteration
  // Combined with aggressive processing during painting operations for immediate responses
  
  delay(1); // Minimal delay - main loop now runs hundreds of times per second
}

//* ************************************************************************
//* ******************** IMMEDIATE COMMAND PROCESSING ******************
//* ************************************************************************

void processImmediateCommand() {
  Serial.print("Processing immediate command: ");
  Serial.println(pendingCommand);
  
  // Process the command immediately by calling the WebSocket command processor directly
  extern void processWebCommand(WebSocketsServer* webSocket, uint8_t num, String commandPayload);
  processWebCommand(&webSocket, pendingCommandClientNum, pendingCommand);
  
  // Clear the pending command
  pendingCommand = "";
  pendingCommandClientNum = 0;
}
