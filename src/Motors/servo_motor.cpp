#include "motors/servo_motor.h"

//* ************************************************************************
//* **************************** SERVO MOTOR *******************************
//* ************************************************************************

// Static variables to replace class members
static Servo servo;
static int servoPin = -1;
static int currentAngle = 0;

void initServoMotor(int pin, int initialAngle) {
    servoPin = pin;
    servo.attach(servoPin);
    setServoAngle(initialAngle);
    Serial.println("Servo Initialized at: " + String(initialAngle) + " degrees");
}

void setServoAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo.write(angle);
    currentAngle = angle;
    //Serial.println("Servo moved to: " + String(angle) + " degrees"); // Avoid serial print during potential motor movement
}

int getCurrentServoAngle() {
    return currentAngle;
} 