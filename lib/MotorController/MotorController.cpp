#include <MotorController.h>

MotorController::MotorController(uint8_t EN_A, uint8_t IN_1, uint8_t IN_2, uint8_t IN_3, uint8_t IN_4, uint8_t EN_B,
                                uint8_t leftMotorEnc1, uint8_t leftMotorEnc2, uint8_t rightMotorEnc1, uint8_t rightMotorEnc2,
                                double left_KP, double left_KI, double left_KD, double right_KP, double right_KI, double right_KD)
{
    enA = EN_A; in1 = IN_1; in2 = IN_2; in3 = IN_3; in4 = IN_4; enB = EN_B;
    leftKP = left_KP; leftKI = left_KI; leftKD = left_KD;
    rightKP = right_KP; rightKI = right_KI; rightKD = right_KD;
    leftPID = new PID(&leftAvgW, &leftPIDout, &leftTargetW, leftKP, leftKI, leftKD, DIRECT);
    rightPID = new PID(&rightAvgW, &rightPIDout, &rightTargetW, rightKP, rightKI, rightKD, DIRECT);
    leftEH = new EncoderHandler(leftMotorEnc1, leftMotorEnc2);
    rightEH = new EncoderHandler(rightMotorEnc1, rightMotorEnc2);

        // MotorDriver::MotorDriver(int ENA, int IN1, int IN2, int IN3, int IN4,
        //                          int ENB) 

    motorDriver = new MotorDriver(enA, in1, in2, in3, in4, enB);

    this->filterSize = 5;
    this->relativeTheta = 0;
    this->targetV = 0;
    this->targetW = 0;

    this->startingTheta = 0;
    this->startingX = 0;
    this->startingY = 0;

    this->currEncCountLeft = 0;
    this->currEncCountRight = 0;
    this->prevEncCountLeft = 0;
    this->prevEncCountRight = 0;

    this->currTime_us = 0;
    this->prevTime_us = 0;
    this->velIndex = 0;

    this->deltaT_us = 0;

    this->rightCurrW = 0;
    this->leftCurrW = 0;

    this->leftWs = (double*)malloc(this->filterSize*sizeof(double));
    this->rightWs = (double*)malloc(this->filterSize*sizeof(double));

    for (int i = 0; i < filterSize; i++) {
        leftWs[i] = 0;
        rightWs[i] = 0;
    }

    this->fullStop = false;
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

    deltaT_us = (double) (currTime_us - prevTime_us);
    deltaT_s = deltaT_us / 1.0e6;

    leftPID->SetMode(AUTOMATIC);
    rightPID->SetMode(AUTOMATIC);
    leftPID->SetOutputLimits(-1.79, 1.79);
    rightPID->SetOutputLimits(-1.79, 1.79);

    leftEH->setup();
    rightEH->setup();
}

void MotorController::loop() {
    this->currTime_us = micros();
    this->deltaT_us = (double) (currTime_us - prevTime_us);
    this->deltaT_s = deltaT_us / 1.0e6;

    ATOMIC() {
        this->currEncCountLeft = leftEH->getCount();
        this->currEncCountRight = rightEH->getCount();
    }

    this->leftCurrW = (double)(1000*(currEncCountLeft - prevEncCountLeft)) / deltaT_us;
    this->rightCurrW = (double)(1000*(currEncCountRight - prevEncCountRight)) / deltaT_us;
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
    // Serial.print("  leftAvgW = ");
    // Serial.print(leftAvgW);
    // Serial.print("  rightAvgW = ");
    // Serial.println(rightAvgW);
    velIndex++;

    //TODO: test these new values
    // odometry stuff - SI
    this->wLeft = 1000 * leftAvgW * PI/(180);
    this->wRight = 1000 * rightAvgW * PI/(180);
    this->vLeft = wLeft * WHEEL_RADIUS;
    this->vRight = wRight * WHEEL_RADIUS;
    this->vRobot = (vLeft + vRight)/2.0;
    this->wRobot = (vRight - vLeft)/WHEELS_DISTANCE;
    
    this->dSpace = vRobot * deltaT_s;
    this->dTheta = wRobot * deltaT_s;

    Serial.print("\nvRobot: ");
    Serial.print(vRobot);
    Serial.print(" | dSpace: ");
    Serial.print(dSpace);

    this->thetaEstimated += dTheta;
    this->xEstimated += dSpace*cos(thetaEstimated);
    this->yEstimated += dSpace*sin(thetaEstimated);

    Serial.print("\nEstT: ");
    Serial.print(thetaEstimated);
    Serial.print(" | EstX: ");
    Serial.print(xEstimated);
    Serial.print(" | EstY: ");
    Serial.print(yEstimated);

    this->relativeTheta += this->dTheta;
    this->relativeSpace += this->dSpace;

    this->vRightT = WHEELS_DISTANCE*targetW*0.5 + targetV; //targets for wheels
    this->vLeftT = -WHEELS_DISTANCE*targetW*0.5 + targetV; //targets for wheels

    if (abs(this->vRightT) > 0.906) {
        if (this->vRightT > 0) this->vRightT = 0.906;
        else this->vRightT = -0.906;
    }

    if (abs(this->vLeftT > 0.906)) {
        if (this->vLeftT > 0) this->vLeftT = 0.906;
        else this->vLeftT = -0.906;
    }

    setTargetW(vLeftT*18.0/(WHEEL_RADIUS*PI*100.0), vRightT*18.0/(WHEEL_RADIUS*PI*100.0)); // set speed targets based on V and W
    
    if (!fullStop) {
        leftPID->Compute();
        rightPID->Compute();
    }

    if (fullStop) motorDriver->coastMotor(LEFT);
    else motorDriver->setMotorSpeed(leftPIDout*PID_MULTIPLIER, LEFT);
    
    if (fullStop) motorDriver->coastMotor(RIGHT);
    else motorDriver->setMotorSpeed(rightPIDout*PID_MULTIPLIER, RIGHT);

    //Save last loop cycle Values
    this->prevEncCountLeft = currEncCountLeft;
    this->prevEncCountRight = currEncCountRight;
    this->prevTime_us = this->currTime_us;
}

MotorDriver* MotorController::getMD() {
    return this->motorDriver;
}

float MotorController::getEstimatedTheta() {
    return this->thetaEstimated;
}

float MotorController::getEstimatedX() {
    return this->xEstimated;
}

float MotorController::getEstimatedY() {
    return this->yEstimated;
}

void MotorController::setFilterSize(int newFilterSize) {
    if (newFilterSize < 1 || newFilterSize > 100) {return;}
    this->filterSize = newFilterSize;
    if(!realloc(this->leftWs, newFilterSize*sizeof(double))) {
        while(true) {
            delay(1000);
        }
    }
    if (!realloc(this->rightWs, newFilterSize*sizeof(double))) {
        while(true) {
            delay(1000);
        }
    }
}

double MotorController::getTargetW(int motor) {
    if (motor == RIGHT) {
        return vRightT*18.0/(WHEEL_RADIUS*PI*100.0);
    }
    else if (motor == LEFT) {
        return vLeftT*18.0/(WHEEL_RADIUS*PI*100.0);
    }
    else return -10;
}

void MotorController::setRobotW(double w) {
    if (w > MAX_ROBOT_W) w = MAX_ROBOT_W;
    else if (w < -MAX_ROBOT_W) w = -MAX_ROBOT_W;
    this->targetW = w;
}

EncoderHandler* MotorController::getEH(int dir) {
    if (dir == RIGHT) return this->rightEH;
    else if (dir == LEFT) return this->leftEH;
    else return NULL;
}

void MotorController::setRobotV(double v) {
    if (v > MAX_ROBOT_V) v = MAX_ROBOT_V;
    else if (v < -MAX_ROBOT_V) v = -MAX_ROBOT_V;
    this->targetV = v;
}

//THIS IS FOR THE WHEELS. FOR ROBOT W SEE setRobotW
void MotorController::setTargetW(double targetLeft, double targetRight) {
    this->setTargetW(LEFT, targetLeft);
    this->setTargetW(RIGHT, targetRight);
}

//THIS IS FOR THE WHEELS. FOR ROBOT W SEE setRobotW
void MotorController::setTargetW(int motor, double target) {

    if (motor != RIGHT && motor != LEFT) {return;}

    if (target > MAX_WHEEL_W) {
        target = MAX_WHEEL_W;
    }
    else if (target < -MAX_WHEEL_W) {
        target = -MAX_WHEEL_W;
    }

    if (motor == RIGHT) {
        this->rightTargetW = target;
    }
    else if (motor == LEFT) {
        this->leftTargetW = target;
    }
}

        // double getLeftKI();
        // double getLeftKP();
        // double getLeftKD();

        // double getRightKP();
        // double getRightKI();
        // double getRightKD();

double MotorController::getRightKP() {
    return rightKP;
}

double MotorController::getRightKI() {
    return rightKI;
}

double MotorController::getRightKD() {
    return rightKD;
}

double MotorController::getLeftKP() {
    return leftKP;
}

double MotorController::getLeftKI() {
    return leftKI;
}

double MotorController::getLeftKD() {
    return leftKD;
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
    this->fullStop = true;
    motorDriver->stopMotors();
    this->leftTargetW = 0;    
    this->rightTargetW = 0;
}

void MotorController::coast() {
    this->fullStop = true;
    motorDriver->coastMotors();
    this->leftTargetW = 0;
    this->rightTargetW = 0;
}

bool MotorController::isStopped() {
    return this->fullStop;
}

void MotorController::setStartingTheta(float sTheta) {
    this->startingTheta = sTheta;
}

void MotorController::setStartingX(float sX) {
    this->startingX = sX;
}

void MotorController::setStartingY(float sY) {
    this->startingY = sY;
}

void MotorController::setEstimatedByStarting(){//TODO verify dis
    this->thetaEstimated = startingTheta;
    this->xEstimated = startingX;
    this->yEstimated = startingY;
}

void MotorController::setEstimatedTheta(float theta) {
    this->thetaEstimated = theta;
}

void MotorController::setEstimatedX(float x) {
    this->xEstimated = x;
}

void MotorController::setEstimatedY(float y) {
    this->yEstimated = y;
}

void MotorController::turnMovementOn() {
    this->fullStop = false;
}

double MotorController::getAvgW(int motor) {
    if (motor != LEFT && motor != RIGHT) {return 0;}
    if (motor == LEFT) {return this->leftAvgW;}
    else return this->rightAvgW;
}

void MotorController::resetRelativeTheta() {
    this->relativeTheta = 0;
}

int MotorController::routine(float routineID){
    switch((int)routineID){
    case 0://vai pega e volta
    break;

    case 1://vai deixa volta
    break;
    
    case 2:// -90 (deslocamento, i.e., horário)
    break;

    case 3:// +90 (deslocamento, i.e., anti-horário)
    break;

    case 4:// -180
    break;
 
    case 5:// +180
    break;
    }

    return 1;
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