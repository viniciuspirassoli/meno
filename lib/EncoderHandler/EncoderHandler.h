//TODO get this to work
#include <Arduino.h>
#include <SimplyAtomic.h>

class EncoderHandler {
    public:
        static void setupEncoder(int ENC_A1, int ENC_A2, int ENC_B1, int ENC_B2);
        static void handleInterrupt1();
        static void handleInterrupt2();
        static void resetCounters();
        static int getCount1();
        static int getCount2();
        static volatile int encCount1, encCount2;

    private:
        static int ENCA_1, ENCB_1, ENCA_2, ENCB_2;
        
};
