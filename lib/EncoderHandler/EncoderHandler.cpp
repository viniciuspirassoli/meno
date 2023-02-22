#include "EncoderHandler.h"

void EncoderHandler::setupEncoder(int ENC_A1, int ENC_A2, int ENC_B1, int ENC_B2) {
    EncoderHandler::ENCA_1 = ENC_A1;
    EncoderHandler::ENCB_1 = ENC_B1;
    EncoderHandler::ENCA_2 = ENC_A2;
    EncoderHandler::ENCB_2 = ENC_B2;
    EncoderHandler::resetCounters();

    attachInterrupt(EncoderHandler::ENCA_1, EncoderHandler::handleInterrupt1, RISING);
    attachInterrupt(EncoderHandler::ENCA_2, EncoderHandler::handleInterrupt2, RISING);
}

void EncoderHandler::handleInterrupt1() {
    if (digitalRead(EncoderHandler::ENCA_2)) {
        EncoderHandler::encCount1--;
    }
    else EncoderHandler::encCount1++;
}

void EncoderHandler::handleInterrupt2() {
    if (digitalRead(EncoderHandler::ENCB_2)) {
        EncoderHandler::encCount2--;
    }
    else EncoderHandler::encCount2++;
}

void EncoderHandler::resetCounters() {
    EncoderHandler::encCount1 = 0;
    EncoderHandler::encCount2 = 0;
}