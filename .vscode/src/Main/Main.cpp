#include <Arduino.h>
// #include "Main.h" // REMOVED - File not found
#include "core/Setup.h"
#include "utils/machine_state.h"
#include <ArduinoOTA.h>
#include <WebSocketsServer.h>
#include "system/StateMachine.h"
#include "motors/ServoMotor.h"
#include "storage/Persistence.h"
#include "storage/PaintingSettings.h"

// Include headers for functions called in loop
#include "web/Web_Dashboard_Commands.h" // For runDashboardServer()
// Add other headers as needed

extern WebSocketsServer webSocket;
extern bool webSocketServerStarted;

// Global variables
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 10;
const int servoPin = 4;

ServoMotor myServo(servoPin);
extern PaintingSettings paintingSettings;

// State machine
extern StateMachine* stateMachine;

// Define the global flag previously in machine_state.cpp
volatile bool homeCommandReceived = false;

//* ************************************************************************
//* ***************************** MAIN *******************************
//* ************************************************************************

void setup() {
  // Initialize state machine *before* system initialization, so it's available
  stateMachine = new StateMachine();

  initializeSystem();
  setupWebDashboardCommands(); // Initialize pins and settings for web commands
  
  // Initialize servo after settings are loaded
  int initialServoAngle = paintingSettings.getServoAngleSide1(); // Get initial angle from loaded settings
  myServo.init(initialServoAngle);
  Serial.printf("Servo Initialized at: %d degrees\n", initialServoAngle);

  // Any setup code that *must* run after initializeSystem()
  Serial.println("Setup complete. Entering main loop...");
}

void loop() {
  // Handle OTA updates
  ArduinoOTA.handle();
  
  // Update machine state
  // updateMachineState();
  
  // Update state machine
  if (stateMachine) {
    stateMachine->update();
  }
  
  //! Handle web server and WebSocket communication
  runDashboardServer(); // Handles incoming client connections and WebSocket messages
  
  // Add calls to other main loop functions here
  // For example, state machine updates, periodic checks, etc.
  
  delay(1); // Small delay to prevent tight loop, adjust as necessary
}
