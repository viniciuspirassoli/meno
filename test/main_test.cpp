#include <Arduino.h>
#include <MotorController.h>
#include <SimplyAtomic.h>

//https://arduino-pico.readthedocs.io/en/latest/
//TODO configure test environment properly
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

//Global vars
volatile int encCount1 = 0;
volatile int encCount2 = 0;

int currCount1;
int currCount2;

int last_encCount1 = 0;
int last_encCount2 = 0;

long currentTime_us = 0;
long lastTime_us = 0;

int rollAvgCount = 0;
double velocities1[FILTER_SIZE] = { 0 };
double velocities2[FILTER_SIZE] = { 0 };

MotorController Motors(ENA, IN1, IN2, IN3, IN4, ENB);
//TODO Make reading the encoders functional

void readEncoder1() {
    if (digitalRead(ENCB_MOT1)) {
        encCount1--;
    }
    else encCount1++;
}

void readEncoder2(){
    if (digitalRead(ENCB_MOT2)){
        encCount2--;
    }
    else encCount2++;
}

void setup() {
  pinMode(ENCA_MOT1, INPUT);
  pinMode(ENCB_MOT1, INPUT);
  pinMode(ENCA_MOT2, INPUT);
  pinMode(ENCB_MOT2, INPUT);
  
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

  float currVelocity1 = (float)(1000*(currCount1 - last_encCount1)) / (float)(currentTime_us - lastTime_us); //ticks per milisecond
  float currVelocity2 = (float)(1000*(currCount2 - last_encCount2)) / (float)(currentTime_us - lastTime_us); //ticks per milisecond

  velocities1[rollAvgCount] = currVelocity1;
  velocities2[rollAvgCount] = currVelocity2;

  if (rollAvgCount >= FILTER_SIZE) rollAvgCount = 0;
  else rollAvgCount++;

  double velocity1 = 0, velocity2 = 0;
  //TODO do this better
  for (int i = 0; i < FILTER_SIZE; i++) {
    velocity1 += velocities1[i];
    velocity2 += velocities2[i];
  }
  
  velocity1 = velocity1/FILTER_SIZE;
  velocity2 = velocity2/FILTER_SIZE;

  Motors.setMotors(80*sin(PI*millis()/2000));

  Serial.print("currCount1: ");
  Serial.print(currCount1);
  Serial.print("__DELTA TICKS 1: ");
  Serial.print((currCount1 - last_encCount1));

  Serial.print("____");
  Serial.print("DELTA T us: ");
  Serial.print(currentTime_us - lastTime_us);
  Serial.print("  Velocity 1: ");
  Serial.print(velocity1);
  Serial.print("speed: ");
  Serial.print(Motors.getSpeed(1));

  
  //Serial.print("currCount2: ");
  //Serial.print(currCount2);
  Serial.println("");

  last_encCount1 = currCount1;
  last_encCount2 = currCount2;
  lastTime_us = currentTime_us;

  //TODO remove this later (after applying rolling average)
  delay(10);
  }