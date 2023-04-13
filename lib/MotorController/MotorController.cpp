#include <MotorController.h>

MotorController::MotorController(uint8_t EN_A, uint8_t IN_1, uint8_t IN_2, uint8_t IN_3, uint8_t IN_4, uint8_t EN_B,
                                uint8_t leftMotorEnc1, uint8_t leftMotorEnc2, uint8_t rightMotorEnc1, uint8_t rightMotorEnc2,
                                double left_KP, double left_KI, double left_KD, double right_KP, double right_KI, double right_KD)
{
    enA = EN_A; in1 = IN_1; in3 = IN_3; in4 = IN_4; enB = EN_B;
    leftKP = left_KP; leftKI = left_KI; leftKD = left_KD;
    rightKP = right_KP; rightKI = right_KI; rightKD = right_KD;
    leftPID = new PID(&leftAvgVelocity, &leftPIDout, &leftTargetVelocity, leftKP, leftKI, leftKD, DIRECT);
    rightPID = new PID(&rightAvgVelocity, &rightPIDout, &rightTargetVelocity, rightKP, rightKI, rightKD, DIRECT);
    leftEH = new EncoderHandler(leftMotorEnc1, leftMotorEnc2);
    rightEH = new EncoderHandler(rightMotorEnc1, rightMotorEnc2);
    motorDriver = new MotorDriver(enA, in1, in2, in3, in4, enB);

    this->filterSize = 5;

    this->currEncCountLeft = 0;
    this->currEncCountRight = 0;
    this->prevEncCountLeft = 0;
    this->prevEncCountRight = 0;

    this->currTime_us = 0;
    this->prevTime_us = 0;
    this->velIndex = 0;

    this->deltaT = 0;

    this->rightCurrVelocity = 0;
    this->leftCurrVelocity = 0;

    this->leftVelocities = (double*)malloc(this->filterSize*sizeof(double));
    this->rightVelocities = (double*)malloc(this->filterSize*sizeof(double));

    for (int i = 0; i < filterSize; i++) {
        leftVelocities[i] = 0;
        rightVelocities[i] = 0;
    }
}

//TODO: Finish destructor
MotorController::~MotorController() {
    free(this->leftVelocities);
    free(this->rightVelocities);
    free(this->rightPID);
    free(this->leftPID);
    free(this->motorDriver);
    free(this->leftEH);
    free(this->rightEH);
}

void MotorController::setup() {
    motorDriver->begin();

    currTime_us = micros();

    deltaT = (double) (currTime_us - prevTime_us);

    leftPID->SetMode(AUTOMATIC);
    rightPID->SetMode(AUTOMATIC);
    leftPID->SetOutputLimits(-1.79, 1.79);
    rightPID->SetOutputLimits(-1.79, 1.79);

    leftEH->setup();
    rightEH->setup();
}

void MotorController::loop() {
    this->currTime_us = micros();
    this->deltaT = (double) (currTime_us - prevTime_us);

    ATOMIC() {
        this->currEncCountLeft = leftEH->getCount();
        this->currEncCountRight = rightEH->getCount();
    }

    this->leftCurrVelocity = (double)(1000*(currEncCountLeft - prevEncCountLeft)) / deltaT;
    this->rightCurrVelocity = (double)(1000*(currEncCountRight - prevEncCountRight)) / deltaT;

    //Filter stuff
    if (this->velIndex >= this->filterSize) this->velIndex = 0;

    this->leftSum -= leftVelocities[velIndex];
    this->rightSum -= rightVelocities[velIndex];

    leftVelocities[velIndex] = leftCurrVelocity;
    rightVelocities[velIndex] = rightCurrVelocity;
     
    leftSum += leftVelocities[velIndex];
    rightSum += rightVelocities[velIndex];
    
    this->leftAvgVelocity = leftSum / (double)filterSize;
    this->rightAvgVelocity = rightSum / (double)filterSize;
    velIndex++;

    //TODO: test these new values
    // odometry stuff - SI
    this->wLeft = 1000 * leftAvgVelocity * PI/(180);
    this->wRight = 1000 * rightAvgVelocity * PI/(180);
    this->vLeft = wLeft * WHEEL_RADIUS;
    this->vRight = wRight * WHEEL_RADIUS;
    this->vRobot = (vLeft + vRight)/2;
    this->wRobot = (vLeft - vRight)/WHEELS_DISTANCE;

    this->dSpace = vRobot * deltaT;
    this->dTheta = wRobot * deltaT;

    leftPID->Compute();
    rightPID->Compute();
    Serial.print("Left PID Out: ");
    Serial.println(leftPIDout);
    motorDriver->setMotorSpeed(leftPIDout*PID_MULTIPLIER, LEFT);
    motorDriver->setMotorSpeed(rightPIDout*PID_MULTIPLIER, RIGHT);
    Serial.print("Left motor set to: ");
    Serial.println(leftPIDout*PID_MULTIPLIER);
    //Save last loop cycle Values
    this->prevEncCountLeft = currEncCountLeft;
    this->prevEncCountRight = currEncCountRight;
    this->prevTime_us = this->currTime_us;
}

void MotorController::setFilterSize(int newFilterSize) {
    if (newFilterSize < 1 || newFilterSize > 100) {return;}
    this->filterSize = newFilterSize;
    if (!realloc(this->leftVelocities, newFilterSize*sizeof(double)) && Serial.available()) {Serial.print("fuck");}
    if (!realloc(this->rightVelocities, newFilterSize*sizeof(double)) && Serial.available()) {Serial.print("fuck");}
}

void MotorController::setTargetVelocities(double targetLeft, double targetRight) {
    this->setTargetVelocity(LEFT, targetLeft);
    this->setTargetVelocity(RIGHT, targetRight);
}

void MotorController::setTargetVelocity(int motor, double target) {
    if (motor != RIGHT && motor != LEFT) {return;}

    if (target > MAX_VELOCITY) {
        target = MAX_VELOCITY;
    }
    else if (target < -MAX_VELOCITY) {
        target = -MAX_VELOCITY;
    }

    if (motor == RIGHT) {
        this->rightTargetVelocity = target;
    }
    else if (motor == LEFT) {
        this->leftTargetVelocity = target;
    }
}

void MotorController::setPIDTunings(double new_KP, double new_KI, double new_KD) {
    leftPID->SetTunings(new_KP, new_KI, new_KD);
    rightPID->SetTunings(new_KP, new_KI, new_KD);
}

void MotorController::setPIDTuning(int motor, double new_KP, double new_KI, double new_KD) {
    if (motor == LEFT)
        leftPID->SetTunings(new_KP, new_KI, new_KD);
    
    else if (motor == RIGHT)
        rightPID->SetTunings(new_KP, new_KI, new_KD);
}

void MotorController::stop() {
    motorDriver->stopMotors();
    this->leftTargetVelocity = 0;    
    this->rightTargetVelocity = 0;
}

void MotorController::coast() {
    motorDriver->coastMotors();
    this->leftTargetVelocity = 0;
    this->rightTargetVelocity = 0;
}

double MotorController::getAvgVelocity(int motor) {
    if (motor != LEFT && motor != RIGHT) {return 0;}
    if (motor == LEFT) {return this->leftAvgVelocity;}
    else return this->rightAvgVelocity;
}

void MotorController::printOdometry() {
    Serial.print("left/right: (");
    Serial.print(this->vLeft); Serial.print(", "); Serial.print(this->vRight); Serial.print(")");
    Serial.print("   V robot: ");
    Serial.print(this->vRobot);
    Serial.print("   W robot: ");
    Serial.println(this->wRobot);
}