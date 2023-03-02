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

#define MAX_VELOCITY 1.79 //in degrees per milisecond

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

void setup() {
  motorController.setup();
  Serial.begin(9600);
  LastTime = millis();
} 


void loop() {
  currentTime = millis();
  if(currentTime-LastTime >= 5000) {
    set = abs(set-1);
    LastTime = currentTime;
  }

  motorController.setTargetVelocities(set*0.5*MAX_VELOCITY, set*0.5*MAX_VELOCITY);
  motorController.loop();

  Serial.print("Left Average Velocity: ");
  Serial.print(motorController.getAvgVelocity(LEFT));
  Serial.print("      ");
  Serial.print("Right Average Velocity: ");
  Serial.print(motorController.getAvgVelocity(RIGHT));
  Serial.println("");
  
  //TODO remove this later
  delay(10);
  }