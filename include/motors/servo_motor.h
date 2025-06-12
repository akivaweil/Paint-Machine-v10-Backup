#ifndef SERVO_MOTOR_H
#define SERVO_MOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>

//* ************************************************************************
//* **************************** SERVO MOTOR *******************************
//* ************************************************************************

// Initialize servo motor
void initServoMotor(int pin, int initialAngle);

// Set servo angle
void setServoAngle(int angle);

// Get current servo angle
int getCurrentServoAngle();

#endif // SERVO_MOTOR_H 