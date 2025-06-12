#ifndef SERVOMOTOR_H
#define SERVOMOTOR_H

#include <Arduino.h>
#include <ESP32Servo.h>

class ServoMotor {
public:
    ServoMotor(int pin);
    void init(int initialAngle);
    void setAngle(int angle);
    int getCurrentAngle();

private:
    Servo servo;
    int servoPin;
    int currentAngle;
};

#endif // SERVOMOTOR_H 