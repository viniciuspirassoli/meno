#pragma once
#include <Arduino.h>
#include <SimplyAtomic.h>

class EncoderHandler {
    public:
        EncoderHandler(uint8_t encoderPin1, uint8_t encoderPin2); 
        void setup();
        static void handleInterrupt(EncoderHandler* obj);
        void resetCount();
        int getCount();
       
    private:
        volatile int encoderPulseCount;
        int ENC_1, ENC_2;
        
};
