#include <Arduino.h>
#include <SimplyAtomic.h>

#define pin uint8_t
#define FORWARD 1
#define BACKWARD 0

//wheel diameter: 67,48mm
//wheel radius: 33,74mm

class Motor
{
private:
    pin PWM_PIN;
    pin DIR1;
    pin DIR2;
    pin ENC1;
    pin ENC2;

    unsigned long prevT = 0;
    float velocity = 0; //in rad/s
    float filtVelocity = 0;
    float targetVelocity = 0;
    int pos = 0;
    int prevPos = 0;

    volatile int pos_i = 0;

    bool debugFlag = false;
    
    void readEncoder();
    
public:
    Motor(pin PWM_PIN, pin DIR1, pin DIR2, pin ENC1, pin ENC2);
    ~Motor();
    void setup();
    void update();
    void setMotor(uint8_t dir, int pwm);
    void debugMode(bool debug);
};

Motor::Motor(pin PWM_PIN, pin DIR1, pin DIR2, pin ENC1, pin ENC2)
{
    this->PWM_PIN = PWM_PIN;
    this->DIR1 = DIR1;
    this->DIR2 = DIR2;
    this->ENC1 = ENC1;
    this->ENC2 = ENC2;
}

Motor::~Motor()
{
}
