#include <Arduino.h>

#define MAX_EM_PWM 255

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