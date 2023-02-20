#include <Arduino.h>
#include <MotorController.h>
#include <SimplyAtomic.h>

//https://arduino-pico.readthedocs.io/en/latest/

//pins on pico use its GPIO numbers
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

//Global vars
volatile int encCount1 = 0;
volatile int encCount2 = 0;

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
  
  //TODO make attachinterrupts
  attachInterrupt(ENCA_MOT1, readEncoder1, RISING);
  attachInterrupt(ENCA_MOT2, readEncoder2, RISING);
} 


void loop() {

  Motors.coastMotors();
  Serial.print("encCount1: ");
  Serial.print(encCount1);
  Serial.print("____");
  Serial.print("encCount2: ");
  Serial.println(encCount2);

}