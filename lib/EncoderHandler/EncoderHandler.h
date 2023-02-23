//TODO get this to work
#include <Arduino.h>
#include <SimplyAtomic.h>

class EncoderHandler {
    public:
        EncoderHandler(int encoderPin1, int encoderPin2); 
        void setup();
        static void handleInterrupt(EncoderHandler* obj);
        void resetCount();
        int getCount();
       
    private:
        volatile int encoderPulseCount;
        int ENC_1, ENC_2;
        
};
