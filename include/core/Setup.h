#ifndef SETUP_H
#define SETUP_H

// #include <FastAccelStepper.h> // Include if objects are defined here
// #include <Bounce2.h>         // Include if objects are defined here
// #include <WebSocketsServer.h> // Include if objects are defined here

// Forward declarations for initialization functions defined in Setup.cpp
void initializeMotorsAndSwitches();
void initializePaintGun();
void initializePressurePot();
void initializeVacuumSystem();
void initializeCylinderSystem();
void initializeWebCommunications(); // Includes WiFi, WebServer, WebSocket
void initializeSystem(); // Main setup orchestrator

#endif // SETUP_H 