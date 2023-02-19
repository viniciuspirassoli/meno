#include "motor.h"

void Motor::setup() {

    pinMode(ENC1, INPUT);
    pinMode(ENC2, INPUT);
    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM_PIN, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENC1), Motor::readEncoder, RISING);

}

void debugMode (bool debug) {
    this->debugFlag = debug;
}

void Motor::readEncoder() {
    if (digitalRead(ENC2)) {
        pos_i += 1;
    }
    else {
        pos_i -= 1;
    }
}

void Motor::update() {

    ATOMIC() {
        pos = pos_i;
    }

    unsigned long currT = micros();

    float deltaT = ((float) (currT - prevT)/1000000);

    velocity = 2*PI*(pos - prevPos)/(360*deltaT); //in rad/s

    if (debugFlag) {
        Serial.println("---------");

        Serial.println("pos: ");
        Serial.print(pos);
        Serial.println("");

        Serial.println("deltaT: ");
        Serial.print(deltaT);
        Serial.println("");

        Serial.println("deltaPos: ");
        Serial.print(pos-prevPos);
        Serial.println("");
        
        Serial.println("velocity: ");
        Serial.print(velocity);
        Serial.println("");

        Serial.println("---------");
    }

    //for next loop
    prevT = currT;
    prevPos = pos;
    //no delay here, that's main.cpp's problem lmao

}

void Motor::setMotor(uint8_t dir, int pwm) {
    //TODO: make this function private once the PID controller is done.
    if (pwm < 0) {
        pwm = 0;
    }
    else if (pwm > 255) {
        pwm = 255;
    }

    if (dir == FORWARD) {
        digitalWrite(DIR1, HIGH);
        digitalWrite(DIR2, LOW);
    }
    else if (dir == BACKWARD) {
        digitalWrite(DIR1, LOW);
        digitalWrite(DIR2, HIGH);
    }
    else {
        digitalWrite(DIR1, LOW);
        digitalWrite(DIR2, LOW);
    }

    analogWrite(PWM_PIN, pwm);
}

float Motor::getVelocity() {
    //TODO: change this to filtVelocity once the filter is done.
    return velocity;
}