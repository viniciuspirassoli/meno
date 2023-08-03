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
#define MESSAGE_LENGTH 5 //in bytes
#define MAX_READ_TIME 300 //microseconds

//TODO: calibrate PIDs!!!
#define KP 0.5
#define KI 1.5
#define KD 0.001

#define INITIAL_X 0
#define INITIAL_Y 0
#define INITIAL_THETA 0

bool readyForMovement = true;

MotorController motorController(ENA, IN1, IN2, IN3, IN4, ENB,
                                ENCA_MOT1, ENCB_MOT1, ENCA_MOT2, ENCB_MOT2,
                                KP, KI, KD, KP, KI, KD);
                                
ElectroMagnet electroMagnet(EM);

uint8_t buf[MESSAGE_LENGTH];

unsigned long currentMillis;
unsigned long lastMillis;
unsigned long deltaMillis;

unsigned long currentMicros;
unsigned long lastMicrosSincePosUpdate = 0;

unsigned int posUpdatePeriod = 1000000; //in micros

// last since x y theta message was sent
float lastEstX;
float lastEstY;
float lastEstTheta;

// current
float currEstX;
float currEstY;
float currEstTheta;

// communication functions:
void readMessage(byte* buf);
void sendMessage(uint8_t messageType);
void sendMessage(uint8_t messageType, float param);
void parseMessage(byte* buf);

void setup() {
  motorController.setup();
  electroMagnet.setup();
  Serial.begin(115200);
  currentMicros = micros();
  
  for (int i = 0; i < MESSAGE_LENGTH; i++) {
    buf[i] = 0;
  }

  motorController.setEstimatedX(INITIAL_X);
  motorController.setEstimatedY(INITIAL_Y);
  motorController.setEstimatedTheta(INITIAL_THETA);

  lastEstX = motorController.getEstimatedX();
  lastEstY = motorController.getEstimatedY();
  lastEstTheta = motorController.getEstimatedTheta();

  motorController.turnMovementOn();
} 

// messages that can be received
typedef enum {
NO_MESSAGE = 0,
HEARTBEAT,
FULL_STOP,
V_COM,
W_COM,
SET_EM,
SEND_ROUTINE,
STOP_MOTORS,
READY,
SET_X,
SET_Y,      
SET_THETA,
ACK_FS,
DX,
DY,
DT,
ROUTINE_DONE,
SET_P_L,
SET_I_L,
SET_D_L,
SET_P_R,
SET_I_R,
SET_D_R
} message_t;

void loop() {

  currentMicros = micros();

  if (currentMicros <= 5e6) {
    motorController.setRobotV(0.2);
  }
  else motorController.stop();

  // sends x, y, theta estimated by encoders to serial port every posUpdatePeriod
  // if (currentMicros - lastMicrosSincePosUpdate >= posUpdatePeriod) {
  //   sendMessage(DX, motorController.getEstimatedX());
  //   sendMessage(DY, motorController.getEstimatedY());
  //   sendMessage(DT, motorController.getEstimatedTheta());
  //   lastMicrosSincePosUpdate = micros();
  // }

  // readMessage(buf);
  // parseMessage(buf); // already does actions

  motorController.loop(); // do not remove unless you wish to bypass motorController

  //TODO: remove this
  delay(10);
}

// received messages are always 1 byte + 1 float
void readMessage(uint8_t* buf) {
  if (Serial.available() >= MESSAGE_LENGTH) {
    Serial.readBytes(buf, MESSAGE_LENGTH);
  }
}

void parseMessage(uint8_t* buf) {
  float param = 0;
  uint8_t messageType = buf[0];
  memcpy(&param, &buf[1], sizeof(float));

  switch(messageType) {
    case HEARTBEAT:
      sendMessage(HEARTBEAT);
    break;
    case FULL_STOP:
      motorController.stop();
      electroMagnet.turnOFF();
      sendMessage(ACK_FS);
    break;
    case V_COM:
      motorController.turnMovementOn();
      motorController.setRobotV(param);
    break;
    case W_COM:
      motorController.turnMovementOn();
      motorController.setRobotW(param);
    break;
    case SET_EM:
      if (param <= 0) electroMagnet.turnOFF();
      else if (param >= MAX_EM_PWM) electroMagnet.turnON();
      else electroMagnet.setPWM((int)param);
    break;
    case SEND_ROUTINE:
      //TODO: routines
      delayMicroseconds(100);
      sendMessage(ROUTINE_DONE, param);
    break;
    case STOP_MOTORS:
      motorController.coast();
    break;
    case READY:
      motorController.setEstimatedByStarting();
      if(param == 1) readyForMovement = true;
      else readyForMovement = false;
    break;
    case SET_X:
      if (readyForMovement) motorController.setEstimatedX(param);
      else motorController.setStartingX(param);
    break;
    case SET_Y:
      if (readyForMovement) motorController.setEstimatedY(param);
      else motorController.setStartingY(param);
    break;
    case SET_THETA:
      if (readyForMovement) motorController.setEstimatedTheta(param);
      else motorController.setStartingTheta(param);
    break;
    case SET_P_L:
      motorController.setPIDTuning(LEFT, param, motorController.getLeftKI(), motorController.getLeftKD());
    break;
    case SET_I_L:
      motorController.setPIDTuning(LEFT, motorController.getLeftKP(), param, motorController.getLeftKD());
    break;
    case SET_D_L:
      motorController.setPIDTuning(LEFT, motorController.getLeftKD(), motorController.getLeftKI(), param);
    break;
    case SET_P_R:
      motorController.setPIDTuning(RIGHT, param, motorController.getRightKI(), motorController.getRightKD());
    break;
    case SET_I_R:
      motorController.setPIDTuning(RIGHT, motorController.getRightKP(), param, motorController.getRightKD());
    break;
    case SET_D_R:
      motorController.setPIDTuning(RIGHT, motorController.getRightKP(), motorController.getRightKI(), param);
    break;
  }

  //clear buffer
  for(int i = 0; i < MESSAGE_LENGTH; i++) {
    buf[i] = NO_MESSAGE;
  }
}

// top 10 overload foda
void sendMessage(uint8_t messageType) {
  sendMessage(messageType, 0.0);
}

void sendMessage(uint8_t messageType, float realParam) {
    Serial.write(messageType);
    byte* floatBytes = (byte*)&realParam;

    for (int i = 0; i < 4; i++) {
      Serial.write(floatBytes[i]);
    }
}