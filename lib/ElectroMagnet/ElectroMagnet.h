#include <Arduino.h>

class ElectroMagnet {
    public:
        ElectroMagnet(int PIN);
        ~ElectroMagnet();
        void setup();
        void setPWM(int PWM);
        void turnON();
        void turnOFF();
    private:
        int pin;
};