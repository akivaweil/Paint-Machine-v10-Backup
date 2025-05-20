#ifndef PAINTGUN_FUNCTIONS_H
#define PAINTGUN_FUNCTIONS_H

#include <Arduino.h>
#include <WebSocketsServer.h>

// Function declarations for paint gun control
void setupPaintGun();
void paintGun_ON();
void paintGun_OFF();

// Web status function (moved from web_command_adapter.h)
void sendWebStatus(WebSocketsServer* webSocket, const char* message);

#endif // PAINTGUN_FUNCTIONS_H 