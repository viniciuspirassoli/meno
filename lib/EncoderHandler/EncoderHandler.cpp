#include "EncoderHandler.h"

EncoderHandler::EncoderHandler(int encoderPin1, int encoderPin2) : ENC_1(encoderPin1), ENC_2(encoderPin2) {
    this->resetCount();
    attachInterrupt(ENC_1, EncoderHandler::handleInterrupt, RISING, this);
}

void EncoderHandler::setup() {
    pinMode(this->ENC_1, INPUT);
    pinMode(this->ENC_2, INPUT);
}


void EncoderHandler::handleInterrupt(EncoderHandler* obj) {
    if (digitalRead(obj->ENC_2)) {
        obj->encoderPulseCount--;
    }
    else  {
        obj->encoderPulseCount++;
    }
}

void EncoderHandler::resetCount() {
    this->encoderPulseCount = 0;
}

int EncoderHandler::getCount() {
    return this->encoderPulseCount;
}