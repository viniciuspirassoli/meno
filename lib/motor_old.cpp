/*
#include <MotorController.h>

MotorController::MotorController(MotorDriver m_d, EncoderHandler left_EH, EncoderHandler right_EH) :
md(&m_d), leftEH(&left_EH), rightEH(&right_EH),
leftPID(&leftAvgVelocity, &leftPIDout, &leftTargetVelocity, leftKP, leftKI, leftKD, DIRECT),            //Initialize left PID
rightPID(&rightAvgVelocity, &rightPIDout, &rightTargetVelocity, rightKP, rightKI, rightKD, DIRECT)      //Initialize right PID
{
    this->filterSize = 5;
    this->currEncCountLeft = 0;
    this->currEncCountRight = 0;
    this->prevEncCountLeft = 0;
    this->prevEncCountRight = 0;
    this->currTime_us = 0;
    this->prevTime_us = 0;
    this->velIndex = 0;

    this->leftVelocities = (double*)malloc(this->filterSize*sizeof(double));
    this->rightVelocities = (double*)malloc(this->filterSize*sizeof(double));
}

//TODO: Finish destructor
MotorController::~MotorController() {
    free(this->leftVelocities);
    free(this->rightVelocities);
}

void MotorController::setup() {
    md->begin();


    leftPID.SetMode(AUTOMATIC);
    rightPID.SetMode(AUTOMATIC);
    leftPID.SetOutputLimits(-100, 100);
    rightPID.SetOutputLimits(-100, 100);

    leftEH->setup();
    rightEH->setup();

    //TODO review this Serial.begin()
    Serial.begin(9600);
}

void MotorController::loop() {
    this->currTime_us = micros();

    ATOMIC() {
        this->currEncCountLeft = leftEH->getCount();
        this->currEncCountRight = rightEH->getCount();
    }

    this->prevTime_us = this->currTime_us;
}

void MotorController::setFilterSize(int newFilterSize) {
    if (newFilterSize < 1 || newFilterSize > 100) {return;}
    this->filterSize = newFilterSize;
    realloc(this->leftVelocities, newFilterSize*sizeof(double));
    realloc(this->rightVelocities, newFilterSize*sizeof(double));
}
*/