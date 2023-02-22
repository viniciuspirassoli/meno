#include <Arduino.h>
#include <MotorController.h>
#include <SimplyAtomic.h>
#include <PID_v1.h>
#include "EncoderHandler.h"

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

//Global vars
volatile int encCount1 = 0;
volatile int encCount2 = 0;

int currCount1;
int currCount2;

int last_encCount1 = 0;
int last_encCount2 = 0;

long currentTime_us = 0;
long lastTime_us = 0;

double velocities1[FILTER_SIZE] = {0};
double velocities2[FILTER_SIZE] = {0};
int velIndex = 0;
double sum1 = 0;
double sum2 = 0;
double avgVelocity1 = 0;
double avgVelocity2 = 0;

double targetVelocity1 = 0;
double targetVelocity2 = 0;
double PIDout1 = 0;
double PIDout2 = 0;

MotorController Motors(ENA, IN1, IN2, IN3, IN4, ENB);
PID Motor1(&avgVelocity1, &PIDout1, &targetVelocity1, 58.3, 50, 0, DIRECT);
PID Motor2(&avgVelocity2, &PIDout2, &targetVelocity2, 58.3, 50, 0, DIRECT);

void readEncoder1() {
    if (digitalRead(ENCB_MOT1)) {
        encCount1--;
    }
    else encCount1++;
}

void readEncoder2() {
    if (digitalRead(ENCB_MOT2)){
        encCount2--;
    }
    else encCount2++;
}

void setupEncoders() {
  pinMode(ENCA_MOT1, INPUT);
  pinMode(ENCB_MOT1, INPUT);
  pinMode(ENCA_MOT2, INPUT);
  pinMode(ENCB_MOT2, INPUT);
}

void setup() {
  setupEncoders();
  Motor1.SetMode(AUTOMATIC);
  Motor2.SetMode(AUTOMATIC);
  Motor1.SetOutputLimits(-100, 100);
  Motor2.SetOutputLimits(-100, 100);
  
  Motors.begin();
  Serial.begin(9600);

  attachInterrupt(ENCA_MOT1, readEncoder1, RISING);
  attachInterrupt(ENCA_MOT2, readEncoder2, RISING);
} 


void loop() {
  currentTime_us = micros();

  ATOMIC() {
    currCount1 = encCount1;
    currCount2 = encCount2;
  }
  
  targetVelocity1 = 0.25*MAX_VELOCITY; //percentage of max speed
  targetVelocity2 = 0.25*MAX_VELOCITY; //percentage of max speed

  double currVelocity1 = (double)(1000*(currCount1 - last_encCount1)) / (double)(currentTime_us - lastTime_us); //degrees per milisecond
  double currVelocity2 = (double)(1000*(currCount2 - last_encCount2)) / (double)(currentTime_us - lastTime_us); //degrees per milisecond
  
  if (velIndex >= FILTER_SIZE) velIndex = 0;

  //subtract old readings from sum
  sum1 -= velocities1[velIndex];
  sum2 -= velocities2[velIndex];

  //update buffer with new readings
  velocities1[velIndex] = currVelocity1;
  velocities2[velIndex] = currVelocity2;

  //add new readings to sum
  sum1 += velocities1[velIndex];
  sum2 += velocities2[velIndex];
  
  //calculate average
  avgVelocity1 = sum1 / FILTER_SIZE;
  avgVelocity2 = sum2 / FILTER_SIZE;

  //update index
  velIndex++;

  Motor1.Compute();
  Motor2.Compute();
  
  Motors.setMotorSpeed(PIDout1, 1);
  Motors.setMotorSpeed(PIDout2, 2);

  Serial.print("Average velocity 1: ");
  Serial.print(avgVelocity1);
  Serial.print("  Average velocity 2: ");
  Serial.print(avgVelocity2);
  Serial.print("      ");
  Serial.print("PIDout1: ");
  Serial.print(PIDout1);
  Serial.print("  ");
  Serial.print("PIDout2: ");
  Serial.print(PIDout2);
  Serial.println("");

  last_encCount1 = currCount1;
  last_encCount2 = currCount2;
  lastTime_us = currentTime_us;

  //TODO remove this later
  delay(10);
  }