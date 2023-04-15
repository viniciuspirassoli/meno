#include <MotorController.h>

MotorController::MotorController(uint8_t EN_A, uint8_t IN_1, uint8_t IN_2, uint8_t IN_3, uint8_t IN_4, uint8_t EN_B,
                                uint8_t leftMotorEnc1, uint8_t leftMotorEnc2, uint8_t rightMotorEnc1, uint8_t rightMotorEnc2,
                                double left_KP, double left_KI, double left_KD, double right_KP, double right_KI, double right_KD)
{
    enA = EN_A; in1 = IN_1; in3 = IN_3; in4 = IN_4; enB = EN_B;
    leftKP = left_KP; leftKI = left_KI; leftKD = left_KD;
    rightKP = right_KP; rightKI = right_KI; rightKD = right_KD;
    leftPID = new PID(&leftAvgW, &leftPIDout, &leftTargetW, leftKP, leftKI, leftKD, DIRECT);
    rightPID = new PID(&rightAvgW, &rightPIDout, &rightTargetW, rightKP, rightKI, rightKD, DIRECT);
    leftEH = new EncoderHandler(leftMotorEnc1, leftMotorEnc2);
    rightEH = new EncoderHandler(rightMotorEnc1, rightMotorEnc2);
    motorDriver = new MotorDriver(enA, in1, in2, in3, in4, enB);

    this->filterSize = 5;

    this->targetV = 0;
    this->targetW = 0;

    this->currEncCountLeft = 0;
    this->currEncCountRight = 0;
    this->prevEncCountLeft = 0;
    this->prevEncCountRight = 0;

    this->currTime_us = 0;
    this->prevTime_us = 0;
    this->velIndex = 0;

    this->deltaT = 0;

    this->rightCurrW = 0;
    this->leftCurrW = 0;

    this->leftWs = (double*)malloc(this->filterSize*sizeof(double));
    this->rightWs = (double*)malloc(this->filterSize*sizeof(double));

    for (int i = 0; i < filterSize; i++) {
        leftWs[i] = 0;
        rightWs[i] = 0;
    }
}

//TODO: Finish destructor
MotorController::~MotorController() {
    free(this->leftWs);
    free(this->rightWs);
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

    this->leftCurrW = (double)(1000*(currEncCountLeft - prevEncCountLeft)) / deltaT;
    this->rightCurrW = (double)(1000*(currEncCountRight - prevEncCountRight)) / deltaT;

    //Filter stuff
    if (this->velIndex >= this->filterSize) this->velIndex = 0;

    this->leftSum -= leftWs[velIndex];
    this->rightSum -= rightWs[velIndex];

    leftWs[velIndex] = leftCurrW;
    rightWs[velIndex] = rightCurrW;
     
    leftSum += leftWs[velIndex];
    rightSum += rightWs[velIndex];
    
    this->leftAvgW = leftSum / (double)filterSize;
    this->rightAvgW = rightSum / (double)filterSize;
    velIndex++;

    //TODO: test these new values
    // odometry stuff - SI
    this->wLeft = 1000 * leftAvgW * PI/(180);
    this->wRight = 1000 * rightAvgW * PI/(180);
    this->vLeft = wLeft * WHEEL_RADIUS;
    this->vRight = wRight * WHEEL_RADIUS;
    this->vRobot = (vLeft + vRight)/2;
    this->wRobot = (vRight - vLeft)/WHEELS_DISTANCE;
 
    this->dSpace = vRobot * deltaT;
    this->dTheta = wRobot * deltaT;

    this->vRightT = (WHEELS_DISTANCE*targetW*0.5 + targetV); //targets for wheels
    this->vLeftT = (-WHEELS_DISTANCE*targetW*0.5 + targetV); //targets for wheels

    setTargetW(vLeftT*18.0/(WHEEL_RADIUS*PI*100.0), vRightT*18.0/(WHEEL_RADIUS*PI*100.0));

    leftPID->Compute();
    rightPID->Compute();
    // Serial.print("Left PID Out: ");
    // Serial.println(leftPIDout);
    if (leftPIDout == 0) motorDriver->stopMotor(LEFT);
    else motorDriver->setMotorSpeed(leftPIDout*PID_MULTIPLIER, LEFT);
    
    if (rightPIDout == 0) motorDriver->stopMotor(RIGHT);
    else motorDriver->setMotorSpeed(rightPIDout*PID_MULTIPLIER, RIGHT);
    // Serial.print("Left motor set to: ");
    // Serial.println(leftPIDout*PID_MULTIPLIER);
    //Save last loop cycle Values
    this->prevEncCountLeft = currEncCountLeft;
    this->prevEncCountRight = currEncCountRight;
    this->prevTime_us = this->currTime_us;
}

void MotorController::setFilterSize(int newFilterSize) {
    if (newFilterSize < 1 || newFilterSize > 100) {return;}
    this->filterSize = newFilterSize;
    if (!realloc(this->leftWs, newFilterSize*sizeof(double)) && Serial.available()) {Serial.print("fuck");}
    if (!realloc(this->rightWs, newFilterSize*sizeof(double)) && Serial.available()) {Serial.print("fuck");}
}

void MotorController::setRobotW(double w) {
    this->targetW = w;
}

void MotorController::setRobotV(double v) {
    this->targetV = v;
}

//THIS IS FOR THE WHEELS. FOR ROBOT W SEE setRobotW
void MotorController::setTargetW(double targetLeft, double targetRight) {
    this->setTargetW(LEFT, targetLeft);
    this->setTargetW(RIGHT, targetRight);
}

void MotorController::setTargetW(int motor, double target) {
    if (motor != RIGHT && motor != LEFT) {return;}

    if (target > MAX_W) {
        target = MAX_W;
    }
    else if (target < -MAX_W) {
        target = -MAX_W;
    }

    if (motor == RIGHT) {
        this->rightTargetW = target;
    }
    else if (motor == LEFT) {
        this->leftTargetW = target;
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
    this->leftTargetW = 0;    
    this->rightTargetW = 0;
}

void MotorController::coast() {
    motorDriver->coastMotors();
    this->leftTargetW = 0;
    this->rightTargetW = 0;
}

double MotorController::getAvgW(int motor) {
    if (motor != LEFT && motor != RIGHT) {return 0;}
    if (motor == LEFT) {return this->leftAvgW;}
    else return this->rightAvgW;
}

void MotorController::printOdometry() {
    Serial.print("vLeft/vRight: (");
    Serial.print(this->vLeft); Serial.print(", "); Serial.print(this->vRight); Serial.print(")");
    Serial.print("   V robot: "); Serial.print(this->vRobot);
    Serial.print("   W robot: "); Serial.println(this->wRobot);
    Serial.println("Targets: ");
    Serial.print("vLeftT/vRightT: ("); Serial.print(this->vLeftT); Serial.print(", "); Serial.print(this->vRightT); Serial.print(")");
    Serial.print("   V target: "); Serial.print(this->targetV);
    Serial.println("   W target: "); Serial.println(this->targetW);
}