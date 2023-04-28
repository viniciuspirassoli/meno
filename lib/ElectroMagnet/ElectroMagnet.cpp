#include "ElectroMagnet.h"

ElectroMagnet::ElectroMagnet(int PIN) {
    this->pin = PIN;
}

ElectroMagnet::~ElectroMagnet() {
    
}

void ElectroMagnet::setup() {
    pinMode(this->pin, OUTPUT);
}

void ElectroMagnet::setPWM(int PWM) {
    if (PWM < 0) PWM = 0;
    else if (PWM > MAX_EM_PWM) PWM = 255;
    analogWrite(this->pin, PWM);
}

void ElectroMagnet::turnON() {
    digitalWrite(this->pin, HIGH);
}

void ElectroMagnet::turnOFF() {
    digitalWrite(this->pin, LOW);
}