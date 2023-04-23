#include <Arduino.h>
#include <MotorDriver.h>
#include <MotorController.h>
#include <SimplyAtomic.h>
#include <PID_v1.h>
#include <EncoderHandler.h>
#include <ElectroMagnet.h>

//https://arduino-pico.readthedocs.io/en/latest/

//pins on pico use their GPIO numbers
#define ENA 11 //15
#define IN1 12 //16 //MOTOR 1 
#define IN2 13 //17

#define IN3 18 //24
#define IN4 19 //25 // MOTOR 2
#define ENB 20 //26

#define ENCA_MOT1 14 //19 //MOTOR 1
#define ENCB_MOT1 15 //20

#define ENCA_MOT2 17 //21 //MOTOR2
#define ENCB_MOT2 16 //22

#define EM 22 //ELECTROMAGNET

#define FILTER_SIZE 5

#define mm_2_m(x) 1000*x

//TODO: calibrate PIDs!!!
#define KP 1
#define KI 1
#define KD 0

MotorController motorController(ENA, IN1, IN2, IN3, IN4, ENB,
                                ENCA_MOT1, ENCB_MOT1, ENCA_MOT2, ENCB_MOT2,
                                KP, KI, KD, KP, KI, KD);
                                
ElectroMagnet electroMagnet(EM);
// MotorDriver motorDriver(ENA, IN1, IN2, IN3, IN4, ENB);
// EncoderHandler leftEncoder(ENCA_MOT1, ENCB_MOT1);
// EncoderHandler rightEncoder(ENCA_MOT2, ENCB_MOT2);

unsigned long previousMillis = 0;

void setup() {
  motorController.setup();
  electroMagnet.setup();
  // motorDriver.begin();
  // leftEncoder.setup();
  // rightEncoder.setup();
  Serial.begin(115200);
  previousMillis = millis();
} 


void loop() {
  motorController.setRobotV(0.25*MAX_ROBOT_V);
  motorController.setRobotW(0);

  motorController.loop(); // do not remove unless you wish to bypass motorController

  //TODO remove this later
  delay(10);
  }