#include "web/html_content.h"
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiServer.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include "hardware/paintGun_Functions.h" // Added to access sendWebStatus
#include "motors/PaintingSides.h" // Add the new header for painting patterns
#include "storage/PaintingSettings.h" // Corrected path
#include "system/StateMachine.h" // Include StateMachine for state transitions
#include "functionality/ManualControl.h" // ADDED
#include "storage/Persistence.h" // Corrected path (was persistence/persistence.h)
#include "motors/XYZ_Movements.h" // Need for moveToZ
#include "motors/ServoMotor.h" // Need for servo control
#include "motors/stepper_globals.h" // Need for stepperX, stepperY_Left etc.
#include "utils/settings.h" // Need for DEFAULT_Z_SPEED
#include <FastAccelStepper.h> // Include the full library header
#include "web/Web_Dashboard_Commands.h" // Corrected Path to header
#include <ArduinoJson.h>
#include "config.h" // Assuming this is directly under include/
#include "states/IdleState.h" // Include IdleState for comparison
#include "settings/motion.h" // Include for default PNP values
#include "states/CleaningState.h" // Include for setShortMode
#include <limits.h> // ADDED For LONG_MIN, INT_MIN

// --- PNP Settings Keys for NVS ---
#define PNP_X_SPEED_KEY "pnpXSpd"
#define PNP_X_ACCEL_KEY "pnpXAcc"
#define PNP_Y_SPEED_KEY "pnpYSpd"
#define PNP_Y_ACCEL_KEY "pnpYAcc"

// Global variables for PNP settings
float g_pnp_x_speed = DEFAULT_PNP_X_SPEED;
float g_pnp_x_accel = DEFAULT_PNP_X_ACCEL;
float g_pnp_y_speed = DEFAULT_PNP_Y_SPEED;
float g_pnp_y_accel = DEFAULT_PNP_Y_ACCEL;

// Global variable to store requested number of coats for All Sides painting
extern int g_requestedCoats; // Declaration added
extern int g_interCoatDelaySeconds; // ADDED: For delay between coats

//* ************************************************************************
//* ********************* PNP SETTINGS NVS FUNCTIONS ***********************
//* ************************************************************************

void savePnpSettingsToNVS() {
    persistence.beginTransaction(false); // Open NVS for writing
    persistence.saveFloat(PNP_X_SPEED_KEY, g_pnp_x_speed);
    persistence.saveFloat(PNP_X_ACCEL_KEY, g_pnp_x_accel);
    persistence.saveFloat(PNP_Y_SPEED_KEY, g_pnp_y_speed);
    persistence.saveFloat(PNP_Y_ACCEL_KEY, g_pnp_y_accel);
    persistence.endTransaction(); // Close NVS
    Serial.println("PNP motion settings saved to NVS.");
}

void loadPnpSettingsFromNVS() {
    persistence.beginTransaction(true); // Open NVS for reading
    g_pnp_x_speed = persistence.loadFloat(PNP_X_SPEED_KEY, DEFAULT_PNP_X_SPEED);
    g_pnp_x_accel = persistence.loadFloat(PNP_X_ACCEL_KEY, DEFAULT_PNP_X_ACCEL);
    g_pnp_y_speed = persistence.loadFloat(PNP_Y_SPEED_KEY, DEFAULT_PNP_Y_SPEED);
    g_pnp_y_accel = persistence.loadFloat(PNP_Y_ACCEL_KEY, DEFAULT_PNP_Y_ACCEL);
    persistence.endTransaction(); // Close NVS
    Serial.println("PNP motion settings loaded from NVS.");
    Serial.printf("Loaded PNP Settings: X_Speed=%.0f, X_Accel=%.0f, Y_Speed=%.0f, Y_Accel=%.0f\\n",
                  g_pnp_x_speed, g_pnp_x_accel, g_pnp_y_speed, g_pnp_y_accel);
}

//* ************************************************************************
//* ************************* WEB DASHBOARD ***************************
//* ************************************************************************

// Create a simple WiFi server on port 80
// WiFiServer dashboardServer(80); // MOVED TO SETUP.CPP
extern WiFiServer dashboardServer;
WiFiClient dashboardClient;

// Create WebSocket server on port 81
// WebSocketsServer webSocket = WebSocketsServer(81); // MOVED TO SETUP.CPP
extern WebSocketsServer webSocket;

// Flag to track WebSocket server status
// bool webSocketServerStarted = false; // MOVED TO SETUP.CPP
extern bool webSocketServerStarted; // Use global flag from Setup.cpp

// Reference to the global state machine instance (assuming it's defined in Setup.cpp or main.cpp)
extern StateMachine* stateMachine;

// Reference to the global servo motor instance
extern ServoMotor myServo;

// Declarations for functions now that Commands.h is removed
void processWebCommand(WebSocketsServer* webSocket, uint8_t num, String commandPayload);

// Remove the placeholder implementations
// Declarations for painting functions now that PaintingSides.h is removed
// void paintLeftPattern() {
//     Serial.println("Painting Left Pattern...");
//     // Implementation here
// }

// void paintRightPattern() {
//     Serial.println("Painting Right Pattern...");
//     // Implementation here  
// }

// void paintFrontPattern() {
//     Serial.println("Painting Front Pattern...");
//     // Implementation here
// }

// void paintBackPattern() {
//     Serial.println("Painting Back Pattern...");
//     // Implementation here
// }

// void paintAllSides() {
//     Serial.println("Painting All Sides...");
//     paintLeftPattern();
//     paintRightPattern();
//     paintFrontPattern();
//     paintBackPattern();
// }

// Response HTML after painting
const char* response_html = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Paint Machine Dashboard</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="5;url=/" />
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      padding: 20px;
      text-align: center;
      background-color: #f0f0f0;
    }
    .message {
      margin-top: 20px;
      padding: 15px;
      background-color: #fff;
      border-radius: 8px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    .home-link {
      margin-top: 20px;
      display: inline-block;
      color: #4CAF50;
    }
  </style>
</head>
<body>
  <div class="message">
    %MESSAGE%
  </div>
  <a href="/" class="home-link">Back to Dashboard</a>
  <p>Redirecting in 5 seconds...</p>
</body>
</html>
)rawliteral";

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;
    
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        
        // --- Send current state to newly connected client --- 
        if (stateMachine && stateMachine->getCurrentState()) {
            String stateMessage = "STATE:";
            stateMessage += stateMachine->getCurrentState()->getName();
            webSocket.sendTXT(num, stateMessage);
            Serial.print("Sent current state to client #");
            Serial.print(num);
            Serial.print(": ");
            Serial.println(stateMessage);
        } else {
            Serial.println("[WS] Could not send initial state: StateMachine or current state is null.");
            webSocket.sendTXT(num, "STATE:UNKNOWN"); // Send a default
        }
        // ------------------------------------------------------
      }
      break;
    
    case WStype_TEXT:
      {
        String command = String((char*)payload);
        processWebCommand(&webSocket, num, command); 
      }
      break;
      
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      break;
  }
}

// Implementation of processWebCommand
void processWebCommand(WebSocketsServer* webSocket, uint8_t num, String commandPayload) {
    Serial.print("Received payload from client ");
    Serial.print(num);
    Serial.print(": ");
    Serial.println(commandPayload);

    String commandToProcess = commandPayload; // This will hold the actual command string for colon-parsing

    JsonDocument doc; // Use JsonDocument for dynamic allocation
    DeserializationError error = deserializeJson(doc, commandPayload);

    if (!error) { // JSON parsing was successful
        if (doc["command"].is<const char*>()) {
            String json_command_field = doc["command"].as<String>();
            json_command_field.trim(); // Trim whitespace

            Serial.print("Parsed JSON command field: ");
            Serial.println(json_command_field);

            if (json_command_field.equalsIgnoreCase("UPDATE_PNP_SETTINGS")) {
                if (doc["pnp_x_speed"].is<float>()) g_pnp_x_speed = doc["pnp_x_speed"].as<float>();
                if (doc["pnp_x_accel"].is<float>()) g_pnp_x_accel = doc["pnp_x_accel"].as<float>();
                if (doc["pnp_y_speed"].is<float>()) g_pnp_y_speed = doc["pnp_y_speed"].as<float>();
                if (doc["pnp_y_accel"].is<float>()) g_pnp_y_accel = doc["pnp_y_accel"].as<float>();
                
                Serial.printf("Updated PNP Settings (in memory): X_Speed=%.0f, X_Accel=%.0f, Y_Speed=%.0f, Y_Accel=%.0f\n", 
                              g_pnp_x_speed, g_pnp_x_accel, g_pnp_y_speed, g_pnp_y_accel);
                
                savePnpSettingsToNVS(); // Save to NVS

                JsonDocument pnpSettingsDoc;
                pnpSettingsDoc["event"] = "pnp_settings";
                pnpSettingsDoc["pnp_x_speed"] = g_pnp_x_speed;
                pnpSettingsDoc["pnp_x_accel"] = g_pnp_x_accel;
                pnpSettingsDoc["pnp_y_speed"] = g_pnp_y_speed;
                pnpSettingsDoc["pnp_y_accel"] = g_pnp_y_accel;
                
                String output;
                serializeJson(pnpSettingsDoc, output);
                webSocket->sendTXT(num, output);
                Serial.println("Sent current PNP settings to client after update.");
                return; // Command processed
            } else if (json_command_field.equalsIgnoreCase("GET_PNP_SETTINGS")) {
                JsonDocument settings_doc;
                settings_doc["event"] = "pnp_settings";
                settings_doc["pnp_x_speed"] = g_pnp_x_speed;
                settings_doc["pnp_x_accel"] = g_pnp_x_accel;
                settings_doc["pnp_y_speed"] = g_pnp_y_speed;
                settings_doc["pnp_y_accel"] = g_pnp_y_accel;
                
                String output;
                serializeJson(settings_doc, output);
                webSocket->sendTXT(num, output);
                Serial.println("Sent current PNP settings to client on request.");
                return; // Command processed
            } else {
                // It's a different command, but was wrapped in JSON.
                // Use the value of the "command" field for further parsing.
                commandToProcess = json_command_field;
                Serial.print("Using extracted JSON command for further processing: ");
                Serial.println(commandToProcess);
            }
        } else {
             // JSON was valid, but "command" field was missing or not a string.
             Serial.println("[WS] JSON received, but 'command' field is missing, null, or not a string.");
             webSocket->sendTXT(num, "CMD_ERROR: Malformed JSON command structure.");
             return; // Reject this
        }
    } else {
        // JSON parsing failed. commandToProcess remains the original commandPayload.
        // This is expected for non-JSON commands (if any were still being sent directly, though JS sends all as JSON)
        // Or it's a genuine malformed JSON string.
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        Serial.println("Proceeding with commandToProcess as plain string (if applicable).");
    }

    // Now, commandToProcess contains the string to be processed by colon-logic.
    String baseCommandAction;
    String valueStr;
    float value1 = 0.0f;
    String message; 

    int colonIndex = commandToProcess.indexOf(':');
    
    if (colonIndex == -1) {
        baseCommandAction = commandToProcess;
    } else {
        baseCommandAction = commandToProcess.substring(0, colonIndex);
        valueStr = commandToProcess.substring(colonIndex + 1);
        // Attempt to parse value1 if valueStr is not empty and is expected to be a number for some commands
        if (valueStr.length() > 0) {
             // For commands that expect a float value after ':', like SET_PAINT_SPEED:value
             // For commands like MANUAL_MOVE_TO:x,y,z,angle, valueStr itself is parsed later.
            value1 = valueStr.toFloat(); // toFloat returns 0.0 if conversion fails
        }
    }
    baseCommandAction.toUpperCase(); // Standardize the action part of the command

    Serial.print("Final baseCommandAction: '");
    Serial.print(baseCommandAction);
    Serial.print("', valueStr: '");
    Serial.print(valueStr);
    Serial.print("', value1: ");
    Serial.println(value1);

    // --- STATE MACHINE CHECK --- 
    // Note: Using baseCommandAction here
    if (stateMachine && stateMachine->getCurrentState() != stateMachine->getIdleState() && 
        (baseCommandAction == "HOME_ALL" || 
         baseCommandAction == "START_PNP" ||
         baseCommandAction == "PAINT_SIDE_1" || 
         baseCommandAction == "PAINT_SIDE_4" || 
         baseCommandAction == "PAINT_SIDE_2" || 
         baseCommandAction == "PAINT_SIDE_3" || 
         baseCommandAction == "PAINT_ALL_SIDES" ||
         baseCommandAction == "PAINT_ALL_SIDES_MULTIPLE")) { // Added PAINT_ALL_SIDES_MULTIPLE here
        Serial.print("Command ");
        Serial.print(baseCommandAction);
        Serial.println(" rejected. Machine must be in IDLE state.");
        webSocket->sendTXT(num, "CMD_ERROR: Machine not in IDLE state.");
        return;
    }

    // --- COMMAND PROCESSING ---
    // Ensure all subsequent checks use 'baseCommandAction'
    if (baseCommandAction == "STATUS") {
        // Send back current status (e.g., state, positions)
        // sendWebStatus(webSocket, "STATUS_UPDATE"); // Call with a message or update internal logic
                                                   // Assuming sendWebStatus broadcasts or uses webSocket internally
                                                   // If it MUST send to 'num', sendWebStatus needs modification.
    }
    else if (baseCommandAction == "HOME_ALL") {
        // Trigger homing state
        if (stateMachine) {
            // Force stop any running motors first
            if (stepperX->isRunning()) stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
            if (stepperY_Left->isRunning()) stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
            if (stepperY_Right->isRunning()) stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
            if (stepperZ->isRunning()) stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
            
            // Set the home command received flag to interrupt any ongoing painting operations
            homeCommandReceived = true;
            
            // Change to homing state immediately
            stateMachine->changeState(stateMachine->getHomingState());
            webSocket->sendTXT(num, "CMD_ACK: Homing sequence initiated.");
        } else {
             webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }
    }
    else if (baseCommandAction == "START_PNP") { // Changed command name
        // Trigger PnP state via StateMachine - NEW WAY
        Serial.println("Transitioning to PnP State via web command...");
        if (stateMachine) {
            stateMachine->changeState(stateMachine->getPnpState());
            webSocket->sendTXT(num, "CMD_ACK: PnP State initiated.");
        } else {
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }
    }
    else if (baseCommandAction == "PAINT_GUN_ON") {
        // Turn on paint gun
        paintGun_ON();
        webSocket->sendTXT(num, "CMD_ACK: Paint Gun ON");
    }
    else if (baseCommandAction == "PAINT_GUN_OFF") {
        // Turn off paint gun
        paintGun_OFF();
        webSocket->sendTXT(num, "CMD_ACK: Paint Gun OFF");
    }
    else if (baseCommandAction == "PRESSURE_POT_ON") {
        Serial.println("Turning Pressure Pot ON via web command");
        digitalWrite(PRESSURE_POT_PIN, HIGH);
        // It's good practice to also update an internal state variable if you have one for pressure pot
        webSocket->sendTXT(num, "PRESSURE_POT_STATUS:ON"); // Send status back to UI
        Serial.printf("Pressure Pot Pin %d set to HIGH\n", PRESSURE_POT_PIN);
    }
    else if (baseCommandAction == "PRESSURE_POT_OFF") {
        Serial.println("Turning Pressure Pot OFF via web command");
        digitalWrite(PRESSURE_POT_PIN, LOW);
        // Update internal state variable if applicable
        webSocket->sendTXT(num, "PRESSURE_POT_STATUS:OFF"); // Send status back to UI
        Serial.printf("Pressure Pot Pin %d set to LOW\n", PRESSURE_POT_PIN);
    }
    else if (baseCommandAction == "PAINT_SIDE_1") {
        Serial.println("Painting side 1...");
        paintSide1Pattern(); // Call the function directly
        
        // Simplified: Assume painting starts, homing is handled by state machine or user
        webSocket->sendTXT(num, "CMD_ACK: Paint Side 1 initiated.");
        // Transition to Homing state should be handled by the PaintingState or user interaction
    }
    else if (baseCommandAction == "PAINT_SIDE_2") {
        Serial.println("Painting side 2...");
        paintSide2Pattern(); // Call directly
        
        // Simplified: Assume painting starts, homing is handled by state machine or user
        webSocket->sendTXT(num, "CMD_ACK: Paint Side 2 initiated.");
    }
    else if (baseCommandAction == "PAINT_SIDE_3") {
        Serial.println("Painting side 3...");
        paintSide3Pattern(); // Call directly
        
        // Simplified: Assume painting starts, homing is handled by state machine or user
        webSocket->sendTXT(num, "CMD_ACK: Paint Side 3 initiated.");
    }
    else if (baseCommandAction == "PAINT_SIDE_4") {
        Serial.println("Painting side 4...");
        paintSide4Pattern(); // Call directly
        
        // Simplified: Assume painting starts, homing is handled by state machine or user
        webSocket->sendTXT(num, "CMD_ACK: Paint Side 4 initiated.");
    }
    else if (baseCommandAction == "PAINT_ALL_SIDES") {
        Serial.println("Painting all sides (single coat request)...");
        g_requestedCoats = 1; // Explicitly set 1 coat for this command
        if (stateMachine) {
            stateMachine->setTransitioningToPaintAllSides(true); // Set the flag
            if (stateMachine->getCleaningState()) { // Ensure cleaning state exists
                 static_cast<CleaningState*>(stateMachine->getCleaningState())->setShortMode(true); // Set short mode for initial clean
            } else {
                Serial.println("ERROR: WebDashboard - CleaningState not available to set short mode.");
                // Optionally, handle this error, e.g., don't proceed or log more verbosely
            }
            stateMachine->setNextStateOverride(stateMachine->getPaintingState());
            stateMachine->changeState(stateMachine->getCleaningState()); // Start with cleaning
            webSocket->sendTXT(num, "CMD_ACK: Single All Sides paint sequence initiated."); // Inform user
        } else {
            Serial.println("ERROR: StateMachine pointer null. Cannot start Paint All Sides.");
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available."); // Inform user
        }
    }
    else if (baseCommandAction.equalsIgnoreCase("PAINT_ALL_SIDES_MULTIPLE") || baseCommandAction.equalsIgnoreCase("PAINT_MULTIPLE_COATS")) { 
        int numCoats = 1;
        int interCoatDelaySec = 10; // Default delay, matches HTML default

        if (colonIndex != -1 && valueStr.length() > 0) { // valueStr is "COATS:DELAY" or just "COATS"
            int secondColonIndex = valueStr.indexOf(':');
            if (secondColonIndex != -1) { // Format is COATS:DELAY
                String coatsStr = valueStr.substring(0, secondColonIndex);
                String delayStr = valueStr.substring(secondColonIndex + 1);
                numCoats = coatsStr.toInt();
                interCoatDelaySec = delayStr.toInt();
                Serial.printf("Parsed coats: %d, delay: %d from \"%s\"\n", numCoats, interCoatDelaySec, valueStr.c_str());
            } else { // Format is just COATS (backward compatibility or error)
                numCoats = valueStr.toInt();
                Serial.printf("Parsed coats: %d (delay not provided, using default %ds) from \"%s\"\n", numCoats, interCoatDelaySec, valueStr.c_str());
            }

            if (numCoats <= 0) {
                Serial.println("Invalid number of coats received, defaulting to 1.");
                numCoats = 1;
            }
            if (interCoatDelaySec < 0) {
                 Serial.println("Invalid negative delay received, defaulting to 0s.");
                interCoatDelaySec = 0; 
            }
            if (interCoatDelaySec > 600) { // Max 10 minutes, matches JS validation
                Serial.printf("Delay %ds exceeds max 600s, capping.\n", interCoatDelaySec);
                interCoatDelaySec = 600; 
            }
            
        } else {
            Serial.println("Could not parse coats/delay for PAINT_MULTIPLE_COATS, defaulting to 1 coat, 10s delay.");
            // numCoats and interCoatDelaySec retain their defaults (1 and 10)
        }

        Serial.printf("Painting all sides (%d coats, %ds delay request)...\n", numCoats, interCoatDelaySec);
        g_requestedCoats = numCoats;
        g_interCoatDelaySeconds = interCoatDelaySec; 

        if (stateMachine) {
            // Check if machine is IDLE before starting multi-coat
            if (stateMachine->getCurrentState() == stateMachine->getIdleState()) {
                stateMachine->setTransitioningToPaintAllSides(true); // Set the flag
                if (stateMachine->getCleaningState()) { // Ensure cleaning state exists
                    static_cast<CleaningState*>(stateMachine->getCleaningState())->setShortMode(true); // Set short mode for initial clean
                } else {
                    Serial.println("ERROR: WebDashboard - CleaningState not available to set short mode for multiple coats.");
                }
                stateMachine->setNextStateOverride(stateMachine->getPaintingState());
                stateMachine->changeState(stateMachine->getCleaningState()); // Start with cleaning
                webSocket->sendTXT(num, "CMD_ACK: Multiple All Sides paint sequence initiated (" + String(numCoats) + " coats, " + String(interCoatDelaySec) + "s delay).");
            } else {
                 Serial.print("Command ");
                 Serial.print(baseCommandAction);
                 Serial.println(" rejected. Machine must be in IDLE state.");
                 webSocket->sendTXT(num, "CMD_ERROR: Machine not in IDLE state.");
            }
        } else {
            Serial.println("ERROR: StateMachine pointer null. Cannot start Paint All Sides Multiple.");
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }
    }
    else if (baseCommandAction == "CLEAN_GUN") {
        // Enter cleaning state
        Serial.println("Entering cleaning state...");
        // Set machine state directly
        // extern void setMachineState(int state); // No need to set directly, state machine handles it
        // setMachineState(MACHINE_CLEANING);
        if (stateMachine) {
            stateMachine->changeState(stateMachine->getCleaningState());
            webSocket->sendTXT(num, "CMD_ACK: Entering Cleaning Mode");
        } else {
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }
    }
    else if (baseCommandAction == "ENTER_PICKPLACE") {
        // Enter pick and place mode via web command
        // NEW WAY: Transition using StateMachine
        Serial.println("Websocket: ENTER_PICKPLACE command received. Transitioning to PnPState...");
        if (stateMachine) {
            stateMachine->changeState(stateMachine->getPnpState());
            webSocket->sendTXT(num, "CMD_ACK: PnP State initiated.");
        } else {
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }

        // Note: The actual PnP cycling will be handled by the PnPState update() method.
    }
    else if (baseCommandAction == "HOME") {
        // Home all axes
        Serial.println("Homing all axes immediately...");
        
        // Force stop any running motors first
        if (stepperX->isRunning()) stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
        if (stepperY_Left->isRunning()) stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
        if (stepperY_Right->isRunning()) stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
        if (stepperZ->isRunning()) stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
        
        // Set the home command received flag to interrupt any ongoing painting operations
        homeCommandReceived = true;
        
        // Change to homing state immediately
        if (stateMachine) {
            stateMachine->changeState(stateMachine->getHomingState());
            webSocket->sendTXT(num, "CMD_ACK: Homing sequence initiated.");
        } else {
            webSocket->sendTXT(num, "CMD_ERROR: StateMachine not available.");
        }
    }
    else if (baseCommandAction == "MOVE_Z_PREVIEW") {
        float z_pos_inch = value1;
        long z_pos_steps = (long)(z_pos_inch * STEPS_PER_INCH_XYZ);
        // Compare with StateMachine state
        if (stateMachine && (stateMachine->getCurrentState() == stateMachine->getIdleState() || stateMachine->getCurrentState() == stateMachine->getPnpState())) { 
            Serial.printf("Preview move Z to: %.2f inches (%ld steps)\n", z_pos_inch, z_pos_steps);
            // Get current X and Y to maintain position
            long currentX = stepperX->getCurrentPosition();
            long currentY = stepperY_Left->getCurrentPosition(); // Assuming Left/Right are synced
            moveToXYZ(currentX, 1, currentY, 1, z_pos_steps, DEFAULT_Z_SPEED); // Use DEFAULT_Z_SPEED, wait for completion is implicit
        } else {
            Serial.println("Preview move ignored: Machine not idle or in PnP mode.");
            webSocket->broadcastTXT("STATUS:Preview move ignored: Machine not idle or in PnP mode.");
        }
    }
    else if (baseCommandAction == "MOVE_SERVO_PREVIEW") {
        int angle = (int)value1;
        // Compare with StateMachine state
        if (stateMachine && (stateMachine->getCurrentState() == stateMachine->getIdleState() || stateMachine->getCurrentState() == stateMachine->getPnpState())) { 
             if (angle >= 0 && angle <= 180) {
                 Serial.printf("Preview move Servo to: %d\n", angle);
                 myServo.setAngle(angle);
             } else {
                 Serial.println("Invalid servo angle received for preview.");
                 webSocket->broadcastTXT("STATUS:Invalid servo angle received for preview.");
             }
        } else {
             Serial.println("Preview move ignored: Machine not idle or in PnP mode.");
             webSocket->broadcastTXT("STATUS:Preview move ignored: Machine not idle or in PnP mode.");
        }
    }
    else if (baseCommandAction == "GET_STATUS") {
        // REMOVED - State updates are handled by StateMachine broadcasts
        Serial.println("GET_STATUS command received - Handler removed (redundant).");
        webSocket->sendTXT(num, "CMD_NOTE: GET_STATUS is redundant; state is pushed automatically.");
    }
    else if (baseCommandAction == "GET_PATTERN_SETTINGS") {
        // Load and send existing pattern settings using persistence
        // persistence.begin(); // REMOVED - Not needed for load operations
        String settingsMsg = "PATTERN_SETTINGS:";
        settingsMsg += "paintSpeed=";
        settingsMsg += String(persistence.loadFloat(PAINT_SPEED_KEY, 10.0)); // Default 10.0
        settingsMsg += ",edgeOffset=";
        settingsMsg += String(persistence.loadFloat(EDGE_OFFSET_KEY, 0.5)); // Default 0.5
        settingsMsg += ",zClearance=";
        settingsMsg += String(persistence.loadFloat(Z_CLEARANCE_KEY, 1.0)); // Default 1.0
        settingsMsg += ",xOverlap=";
        settingsMsg += String(persistence.loadFloat(X_OVERLAP_KEY, 0.2)); // Default 0.2
        // persistence.end();
        webSocket->broadcastTXT(settingsMsg);
        Serial.println("Sent pattern settings: " + settingsMsg);
    }
    else if (baseCommandAction == "GET_SERVO_ANGLES") { // Handle request for servo angles
        // persistence.begin(); // REMOVED - Not needed for load operations
        String anglesMsg = "SERVO_ANGLES:";
        anglesMsg += "side1="; // Changed from top
        anglesMsg += String(paintingSettings.getSide1RotationAngle()); // NEW WAY
        anglesMsg += ",side3="; // Changed from bottom
        anglesMsg += String(paintingSettings.getSide3RotationAngle()); // NEW WAY
        anglesMsg += ",side4="; // Changed from left
        anglesMsg += String(paintingSettings.getSide4RotationAngle()); // NEW WAY
        anglesMsg += ",side2="; // Changed from right
        anglesMsg += String(paintingSettings.getSide2RotationAngle()); // NEW WAY
        // persistence.end(); // Keep open if other operations might follow quickly
        webSocket->broadcastTXT(anglesMsg);
        Serial.println("Sent servo angles: " + anglesMsg);
    }
    else if (baseCommandAction == "SET_PAINT_SPEED") {
         persistence.beginTransaction(false);
         persistence.saveFloat(PAINT_SPEED_KEY, value1);
         persistence.endTransaction();
         Serial.println("Saved Paint Speed: " + String(value1));
    }
    else if (baseCommandAction == "SET_EDGE_OFFSET") {
        persistence.beginTransaction(false);
        persistence.saveFloat(EDGE_OFFSET_KEY, value1);
        persistence.endTransaction();
        Serial.println("Saved Edge Offset: " + String(value1));
    }
    else if (baseCommandAction == "SET_Z_CLEARANCE") {
        persistence.beginTransaction(false);
        persistence.saveFloat(Z_CLEARANCE_KEY, value1);
        persistence.endTransaction();
        Serial.println("Saved Z Clearance: " + String(value1));
    }
    else if (baseCommandAction == "SET_X_OVERLAP") {
        persistence.beginTransaction(false);
        persistence.saveFloat(X_OVERLAP_KEY, value1);
        persistence.endTransaction();
        Serial.println("Saved X Overlap: " + String(value1));
    }
    else if (baseCommandAction == "SET_SERVO_ANGLE_SIDE1") {
        int angle = valueStr.toInt();
        paintingSettings.setServoAngleSide1(angle); // Update in memory
        paintingSettings.saveSettings(); // Save all settings
        Serial.print("Servo Angle Side 1 set to (and saved): "); // Added debug
        Serial.println(angle);
        webSocket->sendTXT(num, "CMD_ACK: Servo Angle Side 1 set and saved");
    }
    else if (baseCommandAction == "SET_SERVO_ANGLE_SIDE2") {
        int angle = valueStr.toInt();
        paintingSettings.setServoAngleSide2(angle); // Update in memory
        paintingSettings.saveSettings(); // Save all settings
        Serial.print("Servo Angle Side 2 set to (and saved): "); // Added debug
        Serial.println(angle);
        webSocket->sendTXT(num, "CMD_ACK: Servo Angle Side 2 set and saved");
    }
    else if (baseCommandAction == "SET_SERVO_ANGLE_SIDE3") {
        int angle = valueStr.toInt();
        paintingSettings.setServoAngleSide3(angle); // Update in memory
        paintingSettings.saveSettings(); // Save all settings
        Serial.print("Servo Angle Side 3 set to (and saved): "); // Added debug
        Serial.println(angle);
        webSocket->sendTXT(num, "CMD_ACK: Servo Angle Side 3 set and saved");
    }
    else if (baseCommandAction == "SET_SERVO_ANGLE_SIDE4") {
        int angle = valueStr.toInt();
        paintingSettings.setServoAngleSide4(angle); // Update in memory
        paintingSettings.saveSettings(); // Save all settings
        Serial.print("Servo Angle Side 4 set to (and saved): "); // Added debug
        Serial.println(angle);
        webSocket->sendTXT(num, "CMD_ACK: Servo Angle Side 4 set and saved");
    }
    else if (baseCommandAction == "SAVE_PAINT_SETTINGS") {
        // Save current settings to NVS
        persistence.beginTransaction(false); // Start write transaction
        paintingSettings.saveSettings(); // Save all settings managed by PaintingSettings
        persistence.endTransaction(); // End write transaction
        Serial.println("Painting settings saved to NVS via SAVE_PAINT_SETTINGS command.");

        // Send confirmation message to client
        // String message = "STATUS:Settings saved successfully"; // message is already declared
        message = "STATUS:Settings saved successfully"; 
        webSocket->broadcastTXT(message);
    }
    else if (baseCommandAction == "RESET_PAINT_SETTINGS") {
        // Reset painting settings to defaults
        paintingSettings.resetToDefaults();
        paintingSettings.saveSettings(); // Save defaults immediately
        webSocket->broadcastTXT("Painting settings reset to defaults");
        Serial.println("Painting settings reset to defaults");
    }
    else if (baseCommandAction == "SET_PAINTING_OFFSET_X") { 
        float value = value1;
        paintingSettings.setPaintingOffsetX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Painting Offset X set to (in memory): ");
        Serial.println(paintingSettings.getPaintingOffsetX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_PAINTING_OFFSET_Y") { 
        float value = value1;
        paintingSettings.setPaintingOffsetY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Painting Offset Y set to (in memory): ");
        Serial.println(paintingSettings.getPaintingOffsetY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1ZHEIGHT") {
        float value = value1;
        paintingSettings.setSide1ZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide1ZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2ZHEIGHT") {
        float value = value1;
        paintingSettings.setSide2ZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide2ZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3ZHEIGHT") {
        float value = value1;
        paintingSettings.setSide3ZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide3ZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4ZHEIGHT") {
        float value = value1;
        paintingSettings.setSide4ZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide4ZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1SIDEZHEIGHT") {
        float value = value1;
        paintingSettings.setSide1SideZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Side Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide1SideZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2SIDEZHEIGHT") {
        float value = value1;
        paintingSettings.setSide2SideZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Side Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide2SideZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3SIDEZHEIGHT") {
        float value = value1;
        paintingSettings.setSide3SideZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Side Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide3SideZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4SIDEZHEIGHT") {
        float value = value1;
        paintingSettings.setSide4SideZHeight(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Side Z Height set to (in memory): ");
        Serial.println(paintingSettings.getSide4SideZHeight(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1SWEEPY") {
        float value = value1;
        paintingSettings.setSide1SweepY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Sweep Y set to (in memory): ");
        Serial.println(paintingSettings.getSide1SweepY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1SHIFTX") {
        float value = value1;
        paintingSettings.setSide1ShiftX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Shift X set to (in memory): ");
        Serial.println(paintingSettings.getSide1ShiftX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2SWEEPY") {
        float value = value1;
        paintingSettings.setSide2SweepY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Sweep Y set to (in memory): ");
        Serial.println(paintingSettings.getSide2SweepY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2SHIFTX") {
        float value = value1;
        paintingSettings.setSide2ShiftX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Shift X set to (in memory): ");
        Serial.println(paintingSettings.getSide2ShiftX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3SWEEPY") {
        float value = value1;
        paintingSettings.setSide3SweepY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Sweep Y set to (in memory): ");
        Serial.println(paintingSettings.getSide3SweepY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3SHIFTX") {
        float value = value1;
        paintingSettings.setSide3ShiftX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Shift X set to (in memory): ");
        Serial.println(paintingSettings.getSide3ShiftX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4SWEEPY") {
        float value = value1;
        paintingSettings.setSide4SweepY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Sweep Y set to (in memory): ");
        Serial.println(paintingSettings.getSide4SweepY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4SHIFTX") {
        float value = value1;
        paintingSettings.setSide4ShiftX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Shift X set to (in memory): ");
        Serial.println(paintingSettings.getSide4ShiftX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1_ROTATION") {
        int value = (int)value1;
        paintingSettings.setSide1RotationAngle(value);
        Serial.print("Side 1 Rotation Angle set to (in memory): ");
        Serial.println(paintingSettings.getSide1RotationAngle());
        paintingSettings.saveSettings(); // CORRECT - Keep save here for actual rotation setting
    }
    else if (baseCommandAction == "SET_SIDE2_ROTATION") {
        int value = (int)value1;
        paintingSettings.setSide2RotationAngle(value);
        Serial.print("Side 2 Rotation Angle set to (in memory): ");
        Serial.println(paintingSettings.getSide2RotationAngle());
        paintingSettings.saveSettings(); // CORRECT - Keep save here for actual rotation setting
    }
    else if (baseCommandAction == "SET_SIDE3_ROTATION") {
        int value = (int)value1;
        paintingSettings.setSide3RotationAngle(value);
        Serial.print("Side 3 Rotation Angle set to (in memory): ");
        Serial.println(paintingSettings.getSide3RotationAngle());
        paintingSettings.saveSettings(); // CORRECT - Keep save here for actual rotation setting
    }
    else if (baseCommandAction == "SET_SIDE4_ROTATION") {
        int value = (int)value1;
        paintingSettings.setSide4RotationAngle(value);
        Serial.print("Side 4 Rotation Angle set to (in memory): ");
        Serial.println(paintingSettings.getSide4RotationAngle());
        paintingSettings.saveSettings(); // CORRECT - Keep save here for actual rotation setting
    }
    else if (baseCommandAction == "SET_SIDE1PAINTINGXSPEED") {
        int value = (int)value1;
        paintingSettings.setSide1PaintingXSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Painting X Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide1PaintingXSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1PAINTINGYSPEED") {
        int value = (int)value1;
        paintingSettings.setSide1PaintingYSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Painting Y Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide1PaintingYSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2PAINTINGXSPEED") {
        int value = (int)value1;
        paintingSettings.setSide2PaintingXSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Painting X Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide2PaintingXSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2PAINTINGYSPEED") {
        int value = (int)value1;
        paintingSettings.setSide2PaintingYSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Painting Y Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide2PaintingYSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3PAINTINGXSPEED") {
        int value = (int)value1;
        paintingSettings.setSide3PaintingXSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Painting X Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide3PaintingXSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3PAINTINGYSPEED") {
        int value = (int)value1;
        paintingSettings.setSide3PaintingYSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Painting Y Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide3PaintingYSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4PAINTINGXSPEED") {
        int value = (int)value1;
        paintingSettings.setSide4PaintingXSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Painting X Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide4PaintingXSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4PAINTINGYSPEED") {
        int value = (int)value1;
        paintingSettings.setSide4PaintingYSpeed(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Painting Y Speed set to (in memory): ");
        Serial.println(paintingSettings.getSide4PaintingYSpeed());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1STARTX") {
        float value = value1;
        paintingSettings.setSide1StartX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Start X set to (in memory): ");
        Serial.println(paintingSettings.getSide1StartX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE1STARTY") {
        float value = value1;
        paintingSettings.setSide1StartY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 1 Start Y set to (in memory): ");
        Serial.println(paintingSettings.getSide1StartY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2STARTX") {
        float value = value1;
        paintingSettings.setSide2StartX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Start X set to (in memory): ");
        Serial.println(paintingSettings.getSide2StartX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE2STARTY") {
        float value = value1;
        paintingSettings.setSide2StartY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 2 Start Y set to (in memory): ");
        Serial.println(paintingSettings.getSide2StartY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3STARTX") {
        float value = value1;
        paintingSettings.setSide3StartX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Start X set to (in memory): ");
        Serial.println(paintingSettings.getSide3StartX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE3STARTY") {
        float value = value1;
        paintingSettings.setSide3StartY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 3 Start Y set to (in memory): ");
        Serial.println(paintingSettings.getSide3StartY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4STARTX") {
        float value = value1;
        paintingSettings.setSide4StartX(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Start X set to (in memory): ");
        Serial.println(paintingSettings.getSide4StartX(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_SIDE4STARTY") {
        float value = value1;
        paintingSettings.setSide4StartY(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Side 4 Start Y set to (in memory): ");
        Serial.println(paintingSettings.getSide4StartY(), 2);
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "SET_POSTPRINTPAUSE") { 
        int value = (int)value1;
        paintingSettings.setPostPrintPause(value);
        // paintingSettings.saveSettings(); // Remove internal save
        Serial.print("Post-Print Pause set to (in memory): ");
        Serial.println(paintingSettings.getPostPrintPause());
        paintingSettings.saveSettings(); // Save after setting
    }
    else if (baseCommandAction == "GET_PAINT_SETTINGS") {
        // Send all current painting settings to the client
        Serial.println("Sending current painting settings to client");
        
        // Paint Gun Offsets
        // String message = "SETTING:paintingOffsetX:" + String(paintingSettings.getPaintingOffsetX(), 2); // message declared above
        message = "SETTING:paintingOffsetX:" + String(paintingSettings.getPaintingOffsetX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:paintingOffsetY:" + String(paintingSettings.getPaintingOffsetY(), 2);
        webSocket->broadcastTXT(message);
        
        // Z Heights (Order: 1, 2, 3, 4)
        message = "SETTING:side1ZHeight:" + String(paintingSettings.getSide1ZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2ZHeight:" + String(paintingSettings.getSide2ZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3ZHeight:" + String(paintingSettings.getSide3ZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4ZHeight:" + String(paintingSettings.getSide4ZHeight(), 2);
        webSocket->broadcastTXT(message);
        
        // Side Z Heights (Order: 1, 2, 3, 4)
        message = "SETTING:side1SideZHeight:" + String(paintingSettings.getSide1SideZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2SideZHeight:" + String(paintingSettings.getSide2SideZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3SideZHeight:" + String(paintingSettings.getSide3SideZHeight(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4SideZHeight:" + String(paintingSettings.getSide4SideZHeight(), 2);
        webSocket->broadcastTXT(message);
        
        // Rotation Angles (Order: 1, 2, 3, 4)
        message = "SETTING:side1RotationAngle:" + String(paintingSettings.getSide1RotationAngle());
        webSocket->broadcastTXT(message);
        message = "SETTING:side2RotationAngle:" + String(paintingSettings.getSide2RotationAngle());
        webSocket->broadcastTXT(message);
        message = "SETTING:side3RotationAngle:" + String(paintingSettings.getSide3RotationAngle());
        webSocket->broadcastTXT(message);
        message = "SETTING:side4RotationAngle:" + String(paintingSettings.getSide4RotationAngle());
        webSocket->broadcastTXT(message);
        
        // Painting Speeds (Order: 1, 2, 3, 4)
        message = "SETTING:side1PaintingXSpeed:" + String(paintingSettings.getSide1PaintingXSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side1PaintingYSpeed:" + String(paintingSettings.getSide1PaintingYSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side2PaintingXSpeed:" + String(paintingSettings.getSide2PaintingXSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side2PaintingYSpeed:" + String(paintingSettings.getSide2PaintingYSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side3PaintingXSpeed:" + String(paintingSettings.getSide3PaintingXSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side3PaintingYSpeed:" + String(paintingSettings.getSide3PaintingYSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side4PaintingXSpeed:" + String(paintingSettings.getSide4PaintingXSpeed());
        webSocket->broadcastTXT(message);
        message = "SETTING:side4PaintingYSpeed:" + String(paintingSettings.getSide4PaintingYSpeed());
        webSocket->broadcastTXT(message);
        
        // Pattern Start Positions (Order: 1, 2, 3, 4)
        message = "SETTING:side1StartX:" + String(paintingSettings.getSide1StartX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side1StartY:" + String(paintingSettings.getSide1StartY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2StartX:" + String(paintingSettings.getSide2StartX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2StartY:" + String(paintingSettings.getSide2StartY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3StartX:" + String(paintingSettings.getSide3StartX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3StartY:" + String(paintingSettings.getSide3StartY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4StartX:" + String(paintingSettings.getSide4StartX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4StartY:" + String(paintingSettings.getSide4StartY(), 2);
        webSocket->broadcastTXT(message);
        
        // Pattern Dimensions (Order: 1, 2, 3, 4)
        message = "SETTING:side1SweepY:" + String(paintingSettings.getSide1SweepY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side1ShiftX:" + String(paintingSettings.getSide1ShiftX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2SweepY:" + String(paintingSettings.getSide2SweepY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side2ShiftX:" + String(paintingSettings.getSide2ShiftX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3SweepY:" + String(paintingSettings.getSide3SweepY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side3ShiftX:" + String(paintingSettings.getSide3ShiftX(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4SweepY:" + String(paintingSettings.getSide4SweepY(), 2);
        webSocket->broadcastTXT(message);
        message = "SETTING:side4ShiftX:" + String(paintingSettings.getSide4ShiftX(), 2);
        webSocket->broadcastTXT(message);
        
        // Post-Print Pause
        message = "SETTING:postPrintPause:" + String(paintingSettings.getPostPrintPause());
        webSocket->broadcastTXT(message);
        
        // Servo Angles (Order: 1, 2, 3, 4)
        // NOTE: Originally read directly from NVS using old keys. Changed to use getters 
        // from the paintingSettings object to ensure consistency and fix persistence issue.
        message = "SETTING:servoAngleSide1:" + String(paintingSettings.getServoAngleSide1()); // Use getter
        webSocket->broadcastTXT(message);
        message = "SETTING:servoAngleSide2:" + String(paintingSettings.getServoAngleSide2()); // Use getter
        webSocket->broadcastTXT(message);
        message = "SETTING:servoAngleSide3:" + String(paintingSettings.getServoAngleSide3()); // Use getter
        webSocket->broadcastTXT(message);
        message = "SETTING:servoAngleSide4:" + String(paintingSettings.getServoAngleSide4()); // Use getter
        webSocket->broadcastTXT(message);
    }
    else if (baseCommandAction == "GOTO_PNP_PICK_LOCATION") {
        // Implement the logic to go to PNP Pick Location
        Serial.println("GOTO_PNP_PICK_LOCATION command received");
        // Example: stateMachine->changeState(stateMachine->getGoToPnpPickLocationState()); // You'll need to create such a state or function
        // For now, just an ACK.
        webSocket->sendTXT(num, "CMD_ACK: GOTO_PNP_PICK_LOCATION command received (not fully implemented).");
    }
    else if (baseCommandAction == "MANUAL_MOVE_TO") {
        if (canPerformManualMove()) {
            long targetX_steps = 0, targetY_steps = 0;
            long targetZ_steps = LONG_MIN;   // Default to not provided
            int targetAngle_deg = INT_MIN; // Default to not provided

            // Tokenize the string by comma
            int currentIndex = 0;
            int nextComma = -1;
            int valueCount = 0;
            String part;

            do {
                nextComma = valueStr.indexOf(',', currentIndex);
                if (nextComma == -1) {
                    part = valueStr.substring(currentIndex); // Last part
                } else {
                    part = valueStr.substring(currentIndex, nextComma);
                }
                part.trim(); // Trim whitespace from part

                if (valueCount == 0) { // X value
                    if (part.length() > 0) targetX_steps = part.toFloat() * STEPS_PER_INCH_XYZ;
                    else {
                        Serial.println("MANUAL_MOVE_TO: X value cannot be empty.");
                        webSocket->sendTXT(num, "CMD_ERROR: X value for MANUAL_MOVE_TO cannot be empty.");
                        return; // Exit if X is empty
                    }
                } else if (valueCount == 1) { // Y value
                    if (part.length() > 0) targetY_steps = part.toFloat() * STEPS_PER_INCH_XYZ;
                    else {
                        Serial.println("MANUAL_MOVE_TO: Y value cannot be empty.");
                        webSocket->sendTXT(num, "CMD_ERROR: Y value for MANUAL_MOVE_TO cannot be empty.");
                        return; // Exit if Y is empty
                    }
                } else if (valueCount == 2) { // Z value
                    if (part.length() > 0) targetZ_steps = part.toFloat() * STEPS_PER_INCH_XYZ;
                    // If part is empty, targetZ_steps remains LONG_MIN (not provided)
                } else if (valueCount == 3) { // Angle value
                    if (part.length() > 0) targetAngle_deg = part.toInt();
                    // If part is empty, targetAngle_deg remains INT_MIN (not provided)
                }
                valueCount++;
                currentIndex = nextComma + 1;
            } while (nextComma != -1 && valueCount < 4);
            
            // Ensure at least X and Y were processed
            if (valueCount < 2) { // Should have been caught by empty part check, but as a safeguard
                 Serial.println("Invalid format for MANUAL_MOVE_TO. Required: X,Y. Optional: Z,Angle. Input: " + valueStr);
                 webSocket->sendTXT(num, "CMD_ERROR: Invalid format for MANUAL_MOVE_TO. Required: X,Y. Optional: Z,Angle.");
                 return;
            }

            handleManualMoveToPosition(targetX_steps, targetY_steps, targetZ_steps, targetAngle_deg);
            webSocket->sendTXT(num, "CMD_ACK: Manual move executed.");

        } else {
            Serial.println("MANUAL_MOVE_TO command ignored: Manual moves not allowed in current state.");
            webSocket->sendTXT(num, "CMD_ERROR: Manual moves not allowed in current state.");
        }
    }
    else if (baseCommandAction == "MANUAL_ROTATE_CW") {
        if (canPerformManualMove()) {
            handleManualRotateCounterClockwise90(); // Swapped: was handleManualRotateClockwise90()
            webSocket->sendTXT(num, "CMD_ACK: Manual rotate CW executed (now CCW behavior)."); // Updated ACK message
            Serial.println("Manual rotate CW command executed (now CCW behavior).");
        } else {
            Serial.println("MANUAL_ROTATE_CW command ignored: Manual moves not allowed in current state.");
            webSocket->sendTXT(num, "CMD_ERROR: Manual moves not allowed in current state.");
        }
    }
    else if (baseCommandAction == "MANUAL_ROTATE_CCW") {
        if (canPerformManualMove()) { 
            handleManualRotateClockwise90(); // Swapped: was handleManualRotateCounterClockwise90()
            webSocket->sendTXT(num, "CMD_ACK: Manual rotate CCW executed (now CW behavior)."); // Updated ACK message
            Serial.println("Manual rotate CCW command executed (now CW behavior).");
        } else {
            Serial.println("MANUAL_ROTATE_CCW command ignored: Manual moves not allowed in current state.");
            webSocket->sendTXT(num, "CMD_ERROR: Manual moves not allowed in current state.");
        }
    }
    else if (baseCommandAction == "PNP_SAVE_POSITION") {
        // Placeholder - Add implementation if needed
        Serial.println("PNP_SAVE_POSITION command received - Not implemented");
        webSocket->sendTXT(num, "CMD_ERROR: PNP_SAVE_POSITION not implemented");
    }
    else if (baseCommandAction == "TOGGLE_PRESSURE_POT") {
        // Implement the logic to toggle the pressure pot
        Serial.println("Toggling Pressure Pot via web command");
        digitalWrite(PRESSURE_POT_PIN, !digitalRead(PRESSURE_POT_PIN));
        webSocket->sendTXT(num, "CMD_ACK: Pressure Pot toggled");
    }
    else {
        // Unknown command
        Serial.print("Unknown command received: ");
        Serial.println(commandToProcess); // Log the string that was attempted to be processed
        webSocket->sendTXT(num, "CMD_ERROR: Unknown command");
    }
}

// Function that handles WebSocket events without executing commands
// This is used during painting to check for HOME commands
void processWebSocketEvents() {
  // Process WebSocket events to allow receiving commands
  // This is called during operations that need to be interruptible
  webSocket.loop();
  
  // Give some time for any WebSocket messages to be processed
  // This ensures the webSocketEvent handler can set flags like homeCommandReceived
  for (int i = 0; i < 10; i++) {
    webSocket.loop();
    delay(1);
  }
}

// Function to check for HOME command during painting operations
// Returns true if a home command was received
extern volatile bool homeCommandReceived; // Assume declared globally
extern FastAccelStepper *stepperX;
extern FastAccelStepper *stepperY_Left;
extern FastAccelStepper *stepperY_Right;
extern FastAccelStepper *stepperZ;

bool checkForHomeCommand() {
  // Process any pending WebSocket events
  processWebSocketEvents();
  
  // Check if a home command was received
  if (homeCommandReceived) {
    Serial.println("HOME command received - immediately aborting all operations");
    
    // IMMEDIATELY stop all motors
    if (stepperX->isRunning()) stepperX->forceStopAndNewPosition(stepperX->getCurrentPosition());
    if (stepperY_Left->isRunning()) stepperY_Left->forceStopAndNewPosition(stepperY_Left->getCurrentPosition());
    if (stepperY_Right->isRunning()) stepperY_Right->forceStopAndNewPosition(stepperY_Right->getCurrentPosition());
    if (stepperZ->isRunning()) stepperZ->forceStopAndNewPosition(stepperZ->getCurrentPosition());
    
    // If we have a state machine, immediately change to homing state
    if (stateMachine) {
      Serial.println("Changing to homing state immediately due to HOME command");
      stateMachine->changeState(stateMachine->getHomingState());
    }
    
    return true;
  }
  
  return false;
}

// Function to check and restart WebSocket if needed
void ensureWebSocketRunning() {
    if (!webSocketServerStarted || WiFi.status() != WL_CONNECTED) { // Check if server not started OR WiFi disconnected
        if (WiFi.status() == WL_CONNECTED) { // Only try to start/restart if WiFi is actually connected
            // Stop WebSocket if it was already running (e.g., WiFi reconnected)
            if (webSocketServerStarted) { // Check if it was running before closing
                webSocket.close();
                delay(5); // CHANGED FROM 50. Allow time for proper closure before restarting
            }
            
            // Restart WebSocket
            webSocket.begin();
            webSocket.onEvent(webSocketEvent); // Attach event handler
            webSocketServerStarted = true;
            Serial.println("WebSocket server (re)started on port 81");
        } else { // WiFi is not connected
            if (webSocketServerStarted) { // If it was running, stop it
                webSocket.close();
                webSocketServerStarted = false;
                Serial.println("WebSocket server stopped due to WiFi disconnection");
            }
        }
    }
}

// Parse the HTTP request line
String parseRequestLine(String line) {
  // Extract the URL from the request line
  // Typical format: "GET /path HTTP/1.1"
  int startPos = line.indexOf(' ');
  if (startPos < 0) return "";
  
  int endPos = line.indexOf(' ', startPos + 1);
  if (endPos < 0) return "";
  
  return line.substring(startPos + 1, endPos);
}

// Extract value of URL parameter
String getParameter(String url, String param) {
  int startPos = url.indexOf(param + "=");
  if (startPos < 0) return "";
  
  startPos += param.length() + 1;
  int endPos = url.indexOf('&', startPos);
  if (endPos < 0) endPos = url.length();
  
  return url.substring(startPos, endPos);
}

// Handle HTTP request
void handleDashboardClient() {
  dashboardClient = dashboardServer.available();
  if (dashboardClient) {
    Serial.println("[HTTP] New client connected"); // Added for debugging
    String currentLine = "";
    String request = ""; // Initialize request string
    unsigned long clientConnectTime = millis(); // Record connection time
    const unsigned long clientReadTimeout = 5000; // 5 seconds timeout for reading request

    while (dashboardClient.connected()) {
      if (dashboardClient.available()) {
        char c = dashboardClient.read();
        clientConnectTime = millis(); // Reset timeout counter on data received

        // Store the first line of the HTTP request
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // End of HTTP headers, send response based on the request
            
            // Parse the request line *here*
            int firstSpace = request.indexOf(' ');
            int secondSpace = request.indexOf(' ', firstSpace + 1);
            String requestPath = "";
            if (firstSpace != -1 && secondSpace != -1) {
                requestPath = request.substring(firstSpace + 1, secondSpace);
            }
            
            if (requestPath == "/") {
              // Root page - send the dashboard
              dashboardClient.println("HTTP/1.1 200 OK");
              dashboardClient.println("Content-Type: text/html");
              dashboardClient.println("Connection: close");
              dashboardClient.println();
              dashboardClient.println(HTML_PROGMEM);
              // break; // Don't break here, let the client close or timeout
            } 
            else if (requestPath == "/settings") {
              // Settings page - send the same HTML but the client-side JS will show settings
              dashboardClient.println("HTTP/1.1 200 OK");
              dashboardClient.println("Content-Type: text/html");
              dashboardClient.println("Connection: close");
              dashboardClient.println();
              dashboardClient.println(HTML_PROGMEM);
              // break; // Headers processed, response sent, let client close
            }
            else if (requestPath.startsWith("/paint?")) { // Check for /paint with parameters
              // Paint command - Parse parameters from requestPath
              String side = "";
              int qPos = requestPath.indexOf('?');
              if (qPos != -1) {
                  String params = requestPath.substring(qPos + 1);
                  int eqPos = params.indexOf("side=");
                  if (eqPos != -1) {
                      side = params.substring(eqPos + 5);
                      int ampersandPos = side.indexOf('&'); // Remove potential extra params
                      if (ampersandPos != -1) {
                          side = side.substring(0, ampersandPos);
                      }
                  }
              }
                
              String message = "Unknown side: " + side;
              bool painted = false;
              
              // Process the paint command
              if (side == "side4") {
                message = "Painting left side...";
                painted = true;
              }
              else if (side == "side2") {
                message = "Painting right side...";
                painted = true;
              }
              else if (side == "side1") {
                message = "Painting front side...";
                painted = true;
              }
              else if (side == "side3") {
                message = "Painting back side...";
                painted = true;
              }
              else if (side == "all") {
                message = "Painting all sides...";
                painted = true; // Or handle separately if needed
              }
              
              // Send response first
              dashboardClient.println("HTTP/1.1 200 OK");
              dashboardClient.println("Content-Type: text/html");
              dashboardClient.println("Connection: close"); // Important: signal connection close
              dashboardClient.println();
              String response = String(response_html);
              response.replace("%MESSAGE%", message);
              dashboardClient.println(response);
              dashboardClient.flush(); // Ensure response is sent 
              // dashboardClient.stop();  // Let the client close the connection after receiving
              // Delay slightly to ensure data is sent before potentially long operation
              delay(50);
              
              // Now execute the paint job if a valid side was given
              if (painted) {
                  Serial.println(message); // Log the action
                  if (side == "side4") paintSide4Pattern();
                  else if (side == "side2") paintSide2Pattern();
                  else if (side == "side1") paintSide1Pattern();
                  else if (side == "side3") paintSide3Pattern();
                  else if (side == "all") { /* Call paintAllSides() or similar */ }
                  Serial.println(side + " side painting completed");
              }
              break; // Break after handling paint request completely
            }
            else {
              // Not found
              dashboardClient.println("HTTP/1.1 404 Not Found");
              dashboardClient.println("Content-Type: text/html");
              dashboardClient.println("Connection: close");
              dashboardClient.println();
              dashboardClient.println("<html><body><h1>404 Not Found</h1><p>The requested resource was not found on this server.</p><a href='/'>Back to Dashboard</a></body></html>");
              // break; // Let client close
            }
            // End of request handling, break the inner while loop
            break; 
          } else {
            // If we got a newline, check if this is the request line
            if (currentLine.startsWith("GET ") || currentLine.startsWith("POST ")) { // Handle GET or POST
              request = currentLine; // Store the request line
              Serial.print("[HTTP] Request Line: "); // Added for debugging
              Serial.println(request);
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      } // if dashboardClient.available()
      else { // No data available
        if (millis() - clientConnectTime > clientReadTimeout) {
          Serial.println("[HTTP] Client read timeout. Closing connection.");
          dashboardClient.stop();
          break; 
        }
      }
    } // while dashboardClient.connected()
    
    // Close the connection if not already closed by timeout
    if (dashboardClient.connected()) { // Check if still connected
        dashboardClient.stop();
        Serial.println("[HTTP] Client connection closed normally."); // Added for debugging
    } else {
        Serial.println("[HTTP] Client connection was already closed (likely by timeout)."); // Added for debugging
    }
  } // if dashboardClient
}

void runDashboardServer() {
    // Ensure WebSocket is running if WiFi is connected
    ensureWebSocketRunning();
    
    // Handle client requests
    handleDashboardClient();
    
    // Handle WebSocket events
    webSocket.loop();
    delay(5); // CHANGED FROM 50. Main loop delay for server tasks
}

void stopDashboardServer() {
  // Stop the HTTP server
  dashboardServer.end();
  
  // Stop the WebSocket server
  webSocket.close();
  webSocketServerStarted = false;
  
  Serial.println("Dashboard web server and WebSocket server stopped");
}

// Global variables for WiFi, WebServer, WebSockets
// WiFiServer server(80); // Duplicate - Remove
// String header; // Likely unused - Remove

// Function Implementations
/* REMOVED - Logic moved to Setup.cpp and called via initializeWebCommunications()
void setupWebDashboard() {
    const char *ssid = "Everwood"; // Replace with your network SSID
    const char *password = "Everwood-Staff"; // Replace with your network password

    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        Serial.print(".");
    }
    Serial.println(" Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Setup mDNS
    if (!MDNS.begin("paint-machine")) { // paint-machine.local
        Serial.println("Error setting up MDNS responder!");
    } else {
        Serial.println("mDNS responder started: http://paint-machine.local");
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 81);
    }

    // Start HTTP server
    dashboardServer.begin();
    Serial.println("HTTP server started on port 80");

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocketServerStarted = true;
    Serial.println("WebSocket server started on port 81");
}
*/ 

void setupWebDashboardCommands() {
    Serial.println("Setting up Web Dashboard Commands...");
    pinMode(PRESSURE_POT_PIN, OUTPUT);
    digitalWrite(PRESSURE_POT_PIN, LOW); // Ensure pressure pot is off initially
    Serial.printf("Pressure Pot Pin %d initialized as OUTPUT and set to LOW.\n", PRESSURE_POT_PIN);
}