#include <Arduino.h>
#include <MotorController.h>
#include <SimplyAtomic.h>

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

//Global vars
volatile int encCount1 = 0;
volatile int encCount2 = 0;

int currCount1;
int currCount2;

int last_encCount1 = 0;
int last_encCount2 = 0;

long currentTime_us = 0;
long lastTime_us = 0;

float velocities1[FILTER_SIZE] = {0};
float velocities2[FILTER_SIZE] = {0};
int velIndex = 0;
float sum1 = 0;
float sum2 = 0;
float avgVelocity1 = 0;
float avgVelocity2 = 0;

MotorController Motors(ENA, IN1, IN2, IN3, IN4, ENB);

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
  
  velIndex = (velIndex >= FILTER_SIZE) ? 0 : velIndex; //isso e' um if compacto muito maneiro, decorar

  //remove old readings @ velIndex
  sum1 -= velocities1[velIndex];
  sum2 -= velocities2[velIndex];

  //update buffer
  velocities1[velIndex] = currVelocity1;
  velocities2[velIndex] = currVelocity2;

  //add new readings
  sum1 += velocities1[velIndex];
  sum2 += velocities2[velIndex];
  
  //calculate average
  avgVelocity1 = sum1 / FILTER_SIZE;
  avgVelocity2 = sum2 / FILTER_SIZE;

  //update index
  velIndex++;

  Motors.setMotors(80*sin(PI*millis()/2000));

  Serial.print("Average velocity 1: ");
  Serial.print(avgVelocity1);
  Serial.print("   Velocity 1: ");
  Serial.print(currVelocity1);
  
  //Serial.print("currCount2: ");
  //Serial.print(currCount2);
  Serial.println("");

  last_encCount1 = currCount1;
  last_encCount2 = currCount2;
  lastTime_us = currentTime_us;

  //TODO remove this later
  delay(10);
  }