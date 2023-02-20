#include <Arduino.h>
#include <MotorController.h>
#include <SimplyAtomic.h>

//https://arduino-pico.readthedocs.io/en/latest/

//TODO set correct pins to control motors on pico! maybe create a define file
#define ENA 15
#define IN1 16
#define IN2 17

#define IN3 18
#define IN4 19
#define ENB 20

#define ENCA_MOT1 19
#define ENCB_MOT1 20

#define ENCA_MOT2 22
#define ENCB_MOT2 21

//Global vars
long prevT = 0;
float velocity = 0;
float filtVelocity = 0;
float targetVelocity = 0;
float target = 0;
int prevPos = 0;
int nRotation = 0;


MotorController Motors(ENA, IN1, IN2, IN3, IN4, ENB, ENCA_MOT1, ENCB_MOT1, ENCA_MOT2, ENCB_MOT2);

//Volatile variables (for interrupts)
volatile unsigned long prevT_i = 0; //previous time (volatile)
volatile float velocity_i = 0; //velocity (volatile)
volatile int pos_i = 0; //position (volatile)
volatile int recordedPos = 0; //position recorded by single rotation
volatile int prevRecordedPos = 0; //position recorded in previous rotation

void readEncoder();
/* void setMotor (int dir, int pwmVal, int pwmPin, int in1, int in2); */

void setup() {
  Motors.begin();

  attachInterrupt(digitalPinToInterrupt(ENCA_MOT1), readEncoder, RISING);
}


void loop() {
  // put your main code here, to run repeatedly:
  int pos = 0;

  Motors.setMotorSpeed(100, 1);

  unsigned long currT = micros();
 
  ATOMIC() {
    pos = pos_i;
  }
  
  float deltaT = ((float) (currT - prevT)/1000000); //time elapsed in seconds

  velocity = (pos - prevPos)/(deltaT); //encoder ticks per second


  Serial.println("---------");
  Serial.println("recordedPos: ");
  Serial.print(recordedPos);
  Serial.println("");
  
  Serial.println("pos: ");
  Serial.print(pos);
  Serial.println("");

  Serial.println("deltaT: ");
  Serial.print(deltaT);
  Serial.println("");

  Serial.println("deltaPos: ");
  Serial.print(pos-prevPos);
  Serial.println("");
  
  Serial.println("velocity: ");
  Serial.print(velocity);
  Serial.println("");

  Serial.println("---------");

  //update for next loop
  prevT = currT;
  prevPos = pos;
  delay(10);
}

void readEncoder() {
  //TODO: find encoder tick per rotation. Our best guess was 360 ticks/rotation.
  //However, if we put a trigger here to save pos_i into another variable when LINE is LOW, we may get a better estimate.

  if (digitalRead(ENCB_MOT1)) {
    pos_i += 1;
  }
  else {
    pos_i -= 1;
  }

}