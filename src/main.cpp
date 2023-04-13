#include <Arduino.h>
#include <MotorDriver.h>
#include <MotorController.h>
#include <SimplyAtomic.h>
#include <PID_v1.h>
#include <EncoderHandler.h>

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

#define ENCA_MOT2 16 //21 //MOTOR2
#define ENCB_MOT2 17 //22

#define FILTER_SIZE 5

//TODO: calibrate PIDs!!!
#define KP 1
#define KI 1
#define KD 0

MotorController motorController(ENA, IN1, IN2, IN3, IN4, ENB,
                                ENCA_MOT1, ENCB_MOT1, ENCA_MOT2, ENCB_MOT2,
                                KP, KI, KD, KP, KI, KD);
                                
double currentTime = 0;
double LastTime = 0;
int set = 0;

String incoming;
//TODO: change the name of these vars. 
//The pico is supposed to receive other things
float xIncoming = 0, yIncoming = 0, thetaIncoming = 0;

void setup() {
  motorController.setup();
  Serial.begin(9600);
  LastTime = millis();
} 


void loop() {
  double left_vel = 0;
  double right_vel = 0;

  if(Serial.available()){
    incoming = Serial.readString();
    left_vel = incoming.substring(0, 7).toDouble();
    right_vel = incoming.substring(8, 14).toDouble();
    // xIncoming = incoming.substring(0, 7).toDouble();
    // yIncoming = incoming.substring(8, 14).toDouble();
    // thetaIncoming = incoming.substring(15).toDouble();

  }

  // Serial.print("xIncoming: ");
  // Serial.print(xIncoming);
  // Serial.print(" yIncoming: ");
  // Serial.print(yIncoming);
  // Serial.print(" thetaIncoming: ");
  // Serial.println(thetaIncoming);

  motorController.setTargetVelocities(0.01*left_vel*MAX_VELOCITY, 0.01*right_vel*MAX_VELOCITY);
  motorController.loop();

  
  
  //TODO remove this later
  delay(10);
  }