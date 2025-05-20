#include <Arduino.h>
#include "utils/Serial_Commands.h"
// #include "system/machine_state.h" // No longer needed
#include "utils/settings.h"
#include "system/StateMachine.h" // Needed for state access

// Reference to the global state machine instance
extern StateMachine* stateMachine;

// Function declarations for commands
void cmdHome();
void cmdMoveX(float inches);
void cmdMoveY(float inches);
void cmdMoveZ(float inches);
void cmdRotate(float degrees);
void cmdPaintGun(bool state);
void cmdPressurePot(bool state);
void cmdPickCylinder(bool state);
void cmdVacuum(bool state);
void cmdStatus();

void setupSerial() {
  Serial.begin(115200);
  Serial.println("Paint Machine Serial Interface");
  Serial.println("Type 'help' for a list of commands");
}

void processSerialCommands() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.length() > 0) {
      Serial.print("> ");
      Serial.println(command);
      executeCommand(command);
    }
  }
}

void executeCommand(String command) {
  // Split into command and parameter parts
  int spaceIndex = command.indexOf(' ');
  String cmd;
  String param = "";
  
  if (spaceIndex != -1) {
    cmd = command.substring(0, spaceIndex);
    param = command.substring(spaceIndex + 1);
    cmd.trim();
    param.trim();
  } else {
    cmd = command;
  }
  
  // Convert command to lowercase for case-insensitive comparisons
  cmd.toLowerCase();

  // Basic commands
  if (cmd == "help") {
    printHelp();
    return;
  }
  
  else if (cmd == "home") {
    cmdHome();
    return;
  }
  
  else if (cmd == "status") {
    cmdStatus();
    return;
  }
  
  // Movement commands
  else if (cmd == "x") {
    if (param.length() > 0) {
      cmdMoveX(param.toFloat());
    } else {
      Serial.println("Error: Missing position parameter for X axis");
    }
    return;
  }
  
  else if (cmd == "y") {
    if (param.length() > 0) {
      cmdMoveY(param.toFloat());
    } else {
      Serial.println("Error: Missing position parameter for Y axis");
    }
    return;
  }
  
  else if (cmd == "z") {
    if (param.length() > 0) {
      cmdMoveZ(param.toFloat());
    } else {
      Serial.println("Error: Missing position parameter for Z axis");
    }
    return;
  }
  
  else if (cmd == "rotate" || cmd == "r") {
    if (param.length() > 0) {
      cmdRotate(param.toFloat());
    } else {
      Serial.println("Error: Missing angle parameter for rotation");
    }
    return;
  }
  
  // Tool control commands
  else if (cmd == "gun") {
    if (param == "on" || param == "1") {
      cmdPaintGun(true);
    } else if (param == "off" || param == "0") {
      cmdPaintGun(false);
    } else {
      Serial.println("Error: Use 'gun on/off' or 'gun 1/0'");
    }
    return;
  }
  
  else if (cmd == "pressure" || cmd == "pot") {
    if (param == "on" || param == "1") {
      cmdPressurePot(true);
    } else if (param == "off" || param == "0") {
      cmdPressurePot(false);
    } else {
      Serial.println("Error: Use 'pressure on/off' or 'pressure 1/0'");
    }
    return;
  }
  
  else if (cmd == "cylinder") {
    if (param == "on" || param == "1" || param == "extend") {
      cmdPickCylinder(true);
    } else if (param == "off" || param == "0" || param == "retract") {
      cmdPickCylinder(false);
    } else {
      Serial.println("Error: Use 'cylinder on/off' or 'cylinder extend/retract'");
    }
    return;
  }
  
  else if (cmd == "vacuum" || cmd == "suction") {
    if (param == "on" || param == "1") {
      cmdVacuum(true);
    } else if (param == "off" || param == "0") {
      cmdVacuum(false);
    } else {
      Serial.println("Error: Use 'vacuum on/off' or 'vacuum 1/0'");
    }
    return;
  }
  
  // If we got here, the command wasn't recognized
  Serial.print("Unknown command: ");
  Serial.println(command);
  Serial.println("Type 'help' for a list of commands");
}

void printHelp() {
  Serial.println("Available commands:");
  Serial.println("  help                - Show this help message");
  Serial.println("  status              - Show machine status");
  Serial.println("  home                - Home all axes");
  Serial.println("Movement commands:");
  Serial.println("  x <inches>          - Move X axis to position");
  Serial.println("  y <inches>          - Move Y axis to position");
  Serial.println("  z <inches>          - Move Z axis to position");
  Serial.println("  rotate <degrees>    - Move rotation axis to angle");
  Serial.println("  r <degrees>         - Shorthand for rotate");
  Serial.println("Tool control:");
  Serial.println("  gun on/off          - Turn paint gun on/off");
  Serial.println("  pressure on/off     - Turn pressure pot on/off");
  Serial.println("  pot on/off          - Alias for pressure pot");
  Serial.println("  cylinder extend/retract - Extend/retract pick cylinder");
  Serial.println("  vacuum on/off       - Turn vacuum on/off");
  Serial.println("  suction on/off      - Alias for vacuum");
}

// Command implementations
// Note: These are stubs and should call the actual functionality
// from other parts of the project

void cmdHome() {
  Serial.println("Homing all axes...");
  // Call the actual homing function
  // homeAllAxes();
  Serial.println("Homing complete");
}

void cmdMoveX(float inches) {
  if (inches < 0 || inches > X_MAX_TRAVEL_POS_INCH) {
    Serial.println("Error: X position out of range (0-" + String(X_MAX_TRAVEL_POS_INCH) + ")");
    return;
  }
  
  Serial.print("Moving X to ");
  Serial.print(inches);
  Serial.println(" inches");
  
  // Call the actual move function
  // moveXToInches(inches);
}

void cmdMoveY(float inches) {
  if (inches < 0 || inches > Y_MAX_TRAVEL_POS_INCH) {
    Serial.println("Error: Y position out of range (0-" + String(Y_MAX_TRAVEL_POS_INCH) + ")");
    return;
  }
  
  Serial.print("Moving Y to ");
  Serial.print(inches);
  Serial.println(" inches");
  
  // Call the actual move function
  // moveYToInches(inches);
}

void cmdMoveZ(float inches) {
  if (inches < -Z_MAX_TRAVEL_POS_INCH || inches > 0) {
    Serial.println("Error: Z position out of range (-" + String(Z_MAX_TRAVEL_POS_INCH) + "-0)");
    return;
  }
  
  Serial.print("Moving Z to ");
  Serial.print(inches);
  Serial.println(" inches");
  
  // Call the actual move function
  // moveZToInches(inches);
}

void cmdRotate(float degrees) {
  while (degrees >= 360.0f) degrees -= 360.0f;
  while (degrees < 0.0f) degrees += 360.0f;
  
  Serial.print("Rotating to ");
  Serial.print(degrees);
  Serial.println(" degrees");
  
  // Call the actual rotate function
  // rotateToDegrees(degrees);
}

void cmdPaintGun(bool state) {
  Serial.print("Paint gun ");
  Serial.println(state ? "ON" : "OFF");
  
  // Call the actual function
  // setPaintGun(state);
}

void cmdPressurePot(bool state) {
  Serial.print("Pressure pot ");
  Serial.println(state ? "ON" : "OFF");
  
  // Call the actual function
  // setPressurePot(state);
}

void cmdPickCylinder(bool state) {
  Serial.print("Pick cylinder ");
  Serial.println(state ? "EXTENDED" : "RETRACTED");
  
  // Call the actual function
  // setPickCylinder(state);
}

void cmdVacuum(bool state) {
  Serial.print("Vacuum ");
  Serial.println(state ? "ON" : "OFF");
  
  // Call the actual function
  // setVacuum(state);
}

void cmdStatus() {
  Serial.println("Machine Status:");
  
  if (stateMachine && stateMachine->getCurrentState()) {
    const char* stateName = stateMachine->getCurrentState()->getName();
    Serial.print("Current State: ");
    Serial.println(stateName ? stateName : "Unknown (getName failed)");
  } else {
    Serial.println("Current State: UNKNOWN (StateMachine unavailable)");
  }
  
  // Display positions (these would need to be implemented elsewhere)
  // and referenced here
  /*
  Serial.print("X Position: ");
  Serial.print(getCurrentXPositionInches());
  Serial.println(" inches");
  
  Serial.print("Y Position: ");
  Serial.print(getCurrentYPositionInches());
  Serial.println(" inches");
  
  Serial.print("Z Position: ");
  Serial.print(getCurrentZPositionInches());
  Serial.println(" inches");
  
  Serial.print("Rotation: ");
  Serial.print(getCurrentRotationDegrees());
  Serial.println(" degrees");
  */
  
  // Show tool states
  /*
  Serial.print("Paint Gun: ");
  Serial.println(isPaintGunOn() ? "ON" : "OFF");
  
  Serial.print("Pressure Pot: ");
  Serial.println(isPressurePotOn() ? "ON" : "OFF");
  
  Serial.print("Pick Cylinder: ");
  Serial.println(isPickCylinderExtended() ? "EXTENDED" : "RETRACTED");
  
  Serial.print("Vacuum: ");
  Serial.println(isVacuumOn() ? "ON" : "OFF");
  */
}
