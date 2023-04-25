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
    else if (PWM > MAX_EM_PWM) PWM = 170;
    analogWrite(this->pin, PWM);
}

void ElectroMagnet::turnON() {
    this->setPWM(MAX_EM_PWM);
}

void ElectroMagnet::turnOFF() {
    digitalWrite(this->pin, LOW);
}