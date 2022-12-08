#include <Arduino.h>
#include <SimplyAtomic.h>

#define PWM_PIN D0
#define DIR1 D14
#define DIR2 D13
#define ENC1 D10
#define ENC2 D9
#define LINE D6

#define FORWARD 1
#define BACKWARD 0


//Global vars
long prevT = 0;
float velocity = 0;
float filtVelocity = 0;
float targetVelocity = 0;
float target = 0;
int prevPos = 0;
int nRotation = 0;

//Volatile variables (for interrupts)
volatile unsigned long prevT_i = 0; //previous time (volatile)
volatile float velocity_i = 0; //velocity (volatile)
volatile int pos_i = 0; //position (volatile)
volatile int recordedPos = 0; //position recorded by single rotation
volatile int prevRecordedPos = 0; //position recorded in previous rotation

void readEncoder();
void setMotor (int dir, int pwmVal, int pwmPin, int in1, int in2);

void setup() {
  // put your setup code here, to run once: 
  pinMode(PWM_PIN, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(ENC1, INPUT);
  pinMode(ENC2, INPUT);
  pinMode(LINE, INPUT);

  digitalWrite(PWM_PIN, LOW);
  digitalWrite(DIR1, LOW);
  digitalWrite(DIR2, LOW);
  attachInterrupt(digitalPinToInterrupt(ENC1), readEncoder, RISING);
}

void loop() {
  // put your main code here, to run repeatedly:
  int pos = 0;

  setMotor(FORWARD, 50, PWM_PIN, DIR1, DIR2);

  unsigned long currT = micros();

  ATOMIC() {
    pos = pos_i;
  }

  setMotor(FORWARD, 50, PWM_PIN, DIR1, DIR2);
  
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

void setMotor (int dir, int pwmVal, int pwmPin, int in1, int in2) {
  if (pwmVal < 0) {
    pwmVal = 0;
  }
  else if (pwmVal > 255) {
    pwmVal = 255;
  }

  analogWrite(pwmPin, pwmVal);

  if (dir == FORWARD) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (dir == BACKWARD) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  //TODO: find encoder tick per rotation. Our best guess was 360 ticks/rotation.
  //However, if we put a trigger here to save pos_i into another variable when LINE is LOW, we may get a better estimate.

  if (digitalRead(ENC2)) {
    pos_i += 1;
  }
  else {
    pos_i -= 1;
  }

}