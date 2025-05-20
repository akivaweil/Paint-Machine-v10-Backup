#ifndef MANUAL_CONTROL_H
#define MANUAL_CONTROL_H

// Forward declarations if needed (e.g., for StateMachine, ServoMotor, FastAccelStepper)
// However, actual includes will be in the .cpp file generally.

/**
 * @brief Checks if manual movement commands can be performed in the current machine state.
 * Typically, this means the machine is in an IDLE state.
 * @return true if manual moves are allowed, false otherwise.
 */
bool canPerformManualMove();

/**
 * @brief Handles a command to move the machine to a specific XYZ position and servo angle.
 * This function will fetch current positions if needed for any internal logic,
 * set the servo angle, and then command the XYZ axes to move.
 * @param targetX_steps Target X position in steps.
 * @param targetY_steps Target Y position in steps.
 *   @param targetZ_steps Target Z position in steps. Use LONG_MIN if not provided (current Z will be used).
 * @param targetAngle_deg Target servo angle in degrees. Use INT_MIN if not provided (current servo angle will be maintained).
 */
void handleManualMoveToPosition(long targetX_steps, long targetY_steps, long targetZ_steps, int targetAngle_deg);

/**
 * @brief Handles a command to rotate the machine's tray clockwise by 90 degrees.
 * It will fetch the current X, Y, Z positions and servo angle, calculate the new angle,
 * and then call handleManualMoveToPosition.
 */
void handleManualRotateClockwise90();

/**
 * @brief Handles a command to rotate the machine's tray counter-clockwise by 90 degrees.
 * It will fetch the current X, Y, Z positions and servo angle, calculate the new angle,
 * and then call handleManualMoveToPosition.
 */
void handleManualRotateCounterClockwise90();

#endif // MANUAL_CONTROL_H 