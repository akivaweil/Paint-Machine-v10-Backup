#include <Arduino.h>
#include "Setup.h"
#include "utils/settings.h"

// Include necessary libraries and headers
#include <FastAccelStepper.h>
#include <Bounce2.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <ESPmDNS.h>
#include <WiFiServer.h>
#include <WebSocketsServer.h>
#include <WebServer.h>
#include <ArduinoOTA.h>
#include "motors/XYZ_Movements.h"
#include "motors/Rotation_Motor.h"
#include "persistence/Persistence.h"
#include "persistence/PaintingSettings.h"
#include "states/HomingState.h"
#include "states/StateMachine.h"
#include <Preferences.h>
#include "web/Web_Dashboard_Commands.h" // For loadPnpSettingsFromNVS
#include "hardware/GlobalDebouncers.h" // For initializeGlobalDebouncers

//* ************************************************************************
//* ************************* SYSTEM SETUP ***************************
//* ************************************************************************

// --- Global Object Definitions ---
// Defined here, declared extern where needed elsewhere
FastAccelStepperEngine engine;
FastAccelStepper *stepperX = NULL;
FastAccelStepper *stepperY_Left = NULL;
FastAccelStepper *stepperY_Right = NULL;
FastAccelStepper *stepperZ = NULL;
Bounce debounceX;
Bounce debounceY_Left;
Bounce debounceY_Right;
Bounce debounceZ;
WiFiServer dashboardServer(80);
WebSocketsServer webSocket(81);
bool webSocketServerStarted = false;

// --- External Function Declarations ---
extern void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

// Reference to the global state machine instance
extern StateMachine* stateMachine;

//! Initialize Web Communications (WiFi, WebServer, WebSocket)
void initializeWebCommunications() {
    // Serial.println("Initializing Web Communications...");
    
    const char *ssid = "Everwood";
    const char *password = "Everwood-Staff";

    // Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(50);
        // Serial.print(".");
    }
    // Serial.println(" Connected!");
    // Serial.print("IP Address: ");
    // Serial.println(WiFi.localIP());

    // Setup mDNS
    if (!MDNS.begin("paint-machine")) {
        Serial.println("Error setting up MDNS responder!");
    } else {
        // Serial.println("mDNS responder started: http://paint-machine.local");
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("ws", "tcp", 81);
        MDNS.addService("arduino", "tcp", 3232); // Add service for OTA
    }

    // Initialize OTA updates
    ArduinoOTA.setHostname("paint-machine");
    // ArduinoOTA.setPassword("your-ota-password"); // Uncomment and set a password if needed
    
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else {
            type = "filesystem";
        }
        Serial.println("OTA: Start updating " + type);
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\\nOTA: Update complete");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("OTA: Progress: %u%%\\r", (progress / (total / 100)));
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA: Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
    
    ArduinoOTA.begin();
    // Serial.println("OTA Updates initialized and ready to receive updates");
    // Serial.println("IP: " + WiFi.localIP().toString() + " Port: 3232");
    
    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    webSocketServerStarted = true;
    // Serial.println("WebSocket server started on port 81");

    // Start HTTP server
    dashboardServer.begin();
    // Serial.println("HTTP server started on port 80");

    // Serial.println("Web Communications Initialized.");
}

//! Initialize Motors and Limit Switches
void initializeMotorsAndSwitches() {
    // Serial.println("Initializing Motors and Switches...");
    engine.init();
    
    stepperX = engine.stepperConnectToPin(X_STEP_PIN);
    if (stepperX) {
        stepperX->setDirectionPin(X_DIR_PIN);
        stepperX->setSpeedInHz(DEFAULT_X_SPEED);
        stepperX->setAcceleration(DEFAULT_X_ACCEL);
    }

    stepperY_Left = engine.stepperConnectToPin(Y_LEFT_STEP_PIN);
    if (stepperY_Left) {
        stepperY_Left->setDirectionPin(Y_LEFT_DIR_PIN);
        stepperY_Left->setAcceleration(DEFAULT_Y_ACCEL);
    }

    stepperY_Right = engine.stepperConnectToPin(Y_RIGHT_STEP_PIN);
    if (stepperY_Right) {
        stepperY_Right->setDirectionPin(Y_RIGHT_DIR_PIN);
        stepperY_Right->setAcceleration(DEFAULT_Y_ACCEL);
    }

    stepperZ = engine.stepperConnectToPin(Z_STEP_PIN);
    if (stepperZ) {
        stepperZ->setDirectionPin(Z_DIR_PIN);
        stepperZ->setAcceleration(DEFAULT_Z_ACCEL);
    }
    
    // Limit Switch Pin Configuration
    pinMode(X_HOME_SWITCH, INPUT_PULLDOWN);
    pinMode(Y_LEFT_HOME_SWITCH, INPUT_PULLDOWN);
    pinMode(Y_RIGHT_HOME_SWITCH, INPUT_PULLDOWN);
    pinMode(Z_HOME_SWITCH, INPUT_PULLDOWN);
    // pinMode(PNP_CYCLE_SENSOR_PIN, INPUT_PULLUP); // REMOVED: Handled by initializeGlobalDebouncers()
    
    // Debouncer Setup
    debounceX.attach(X_HOME_SWITCH);
    debounceX.interval(DEBOUNCE_INTERVAL);
    
    debounceY_Left.attach(Y_LEFT_HOME_SWITCH);
    debounceY_Left.interval(DEBOUNCE_INTERVAL);
    
    debounceY_Right.attach(Y_RIGHT_HOME_SWITCH);
    debounceY_Right.interval(DEBOUNCE_INTERVAL);

    debounceZ.attach(Z_HOME_SWITCH);
    debounceZ.interval(DEBOUNCE_INTERVAL);

    // Initialize global debouncers (includes PNP Cycle Sensor)
    initializeGlobalDebouncers(); 

    // Serial.println("Motors and Switches Initialized.");
}

//! Initialize Paint Gun
void initializePaintGun() {
    // Serial.println("Initializing Paint Gun...");
    pinMode(PAINT_GUN_PIN, OUTPUT);
    digitalWrite(PAINT_GUN_PIN, LOW); // Ensure it's off initially
    // Serial.println("Paint Gun Initialized.");
}

//! Initialize Pressure Pot
void initializePressurePot() {
    // Serial.println("Initializing Pressure Pot...");
    pinMode(PRESSURE_POT_PIN, OUTPUT);
    digitalWrite(PRESSURE_POT_PIN, LOW); // Ensure it's off initially
    // Serial.println("Pressure Pot Initialized.");
}

//! Initialize Vacuum System
void initializeVacuumSystem() {
    // Serial.println("Initializing Vacuum System...");
    pinMode(SUCTION_PIN, OUTPUT);
    digitalWrite(SUCTION_PIN, LOW); // Start with vacuum off
    // Serial.println("Vacuum System Initialized.");
}

//! Initialize Cylinder System
void initializeCylinderSystem() {
    // Serial.println("Initializing Cylinder System...");
    pinMode(PICK_CYLINDER_PIN, OUTPUT);
    digitalWrite(PICK_CYLINDER_PIN, LOW); // Start with cylinder retracted (UP)
    // Serial.println("Cylinder System Initialized.");
}

//! Initialize Settings System
void initializeSettings() {
    // Persistence needs to be initialized first if not done elsewhere
    // persistence.begin(); // REMOVED - begin/end handled transactionally now
    
    // Now initialize PaintingSettings, which might load or reset/save
    paintingSettings.begin();

    // Load PNP motion settings from NVS
    loadPnpSettingsFromNVS();
    
    // No need to explicitly close persistence here, 
    // paintingSettings.begin() handles its own NVS operations if needed.
    // persistence.end(); // REMOVED
    Serial.println("Settings initialized.");
}

//! Initialize all system components
void initializeSystem() {
    Serial.begin(115200);
    Serial.println("\\n\\n--- System Initialization Starting ---");

    // Connect to WiFi and start WebSocket first
    initializeWebCommunications();
    Serial.println("Web Communications Initialized."); // Keep one summary message

    // Then initialize the rest of the systems
    initializeMotorsAndSwitches();
    Serial.println("Motors and Switches Initialized."); // Keep one summary message
    
    setupRotationMotor(engine);
    // setupRotationMotor logs its own success/fail
    
    initializePaintGun();
    // Serial.println("Paint Gun Initialized."); // Remove individual init message
    initializePressurePot();
    // Serial.println("Pressure Pot Initialized."); // Remove individual init message
    initializeVacuumSystem();
    // Serial.println("Vacuum System Initialized."); // Remove individual init message
    initializeCylinderSystem();
    // Serial.println("Cylinder System Initialized."); // Remove individual init message
    initializeSettings();
    Serial.println("Settings System Initialized."); // Keep one summary message

    Serial.println("--- System Initialization Complete ---");

    // homeAllAxes(); // Remove direct call
    // homeAllAxes will log its completion or errors internally
    
    // Instead of calling homeAllAxes directly, transition to HomingState
    if (stateMachine) {
        Serial.println("Initiating homing sequence via State Machine...");
        stateMachine->changeState(stateMachine->getHomingState());
    } else {
        Serial.println("ERROR: StateMachine pointer is null. Cannot initiate homing!");
        // Consider setting an error state or handling this
    }
}

void setupHardware() {
    Serial.println("--- Setting up Hardware ---");

    // Initialize hardware components
    Serial.println("Hardware setup completed.");
}

/*
// Original setup functions from other files (for reference, now integrated above)

// From XYZ_Movements.cpp:
void setupMotors() { ... }

// From paintGun_Functions.cpp:
void setupPaintGun() { ... }

// From pressurePot_Functions.cpp:
void setupPressurePot() { ... }

// From vacuum_Functions.cpp:
void initializeVacuum() { ... }

// From cylinder_Functions.cpp:
void initializeCylinder() { ... }

// From Web_Dashboard_Commands.cpp:
void setupWebDashboard() { ... } // This function likely handles WiFi, mDNS, WebSocket, etc.
*/
