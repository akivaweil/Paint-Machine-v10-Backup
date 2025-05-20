#include "motors/ServoMotor.h"
#include <ESP32Servo.h>

//* ************************************************************************
//* **************************** SERVO MOTOR *******************************
//* ************************************************************************

ServoMotor::ServoMotor(int pin) : servoPin(pin), currentAngle(0) {}

void ServoMotor::init(int initialAngle) {
    servo.attach(servoPin);
    setAngle(initialAngle);
    Serial.println("Servo Initialized at: " + String(initialAngle) + " degrees");
}

void ServoMotor::setAngle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo.write(angle);
    currentAngle = angle;
    //Serial.println("Servo moved to: " + String(angle) + " degrees"); // Avoid serial print during potential motor movement
}

int ServoMotor::getCurrentAngle() {
    return currentAngle;
} 