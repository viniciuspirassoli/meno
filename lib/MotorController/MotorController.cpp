#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController(int ENA, int IN1, int IN2, int IN3, int IN4,
                                 int ENB, int ENC1, int ENC2, int ENC3, int ENC4) {
    EN_A = ENA;
    IN_1 = IN1;
    IN_2 = IN2;
    IN_3 = IN3;
    IN_4 = IN4;
    EN_B = ENB;
    
    ENC_1 = ENC1;
    ENC_2 = ENC2;
    ENC_3 = ENC3;
    ENC_4 = ENC4;
}

void MotorController::begin() {
    //PWM outputs
    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);

    //Digital outputs
    pinMode(IN_1, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_4, OUTPUT);

    //Encoder inputs
    pinMode(ENC_1, INPUT);
    pinMode(ENC_2, INPUT);
    pinMode(ENC_3, INPUT);
    pinMode(ENC_4, INPUT);
}

void MotorController::setMotorSpeed(float speed, int motor) {
    if ((motor != 1 && motor != 2) || abs(speed) > 100.0F) {
        return;
    }

    if (speed > 0) {
        if (motor == 1) {
            digitalWrite(IN_1, HIGH);
            digitalWrite(IN_2, LOW);
            analogWrite(EN_A, (uint32_t)(speed * 2.55F));
            motor1PWM = speed;
        }

        else {
            digitalWrite(IN_3, HIGH);
            digitalWrite(IN_4, LOW);
            analogWrite(EN_B, (uint32_t)(speed * 2.55F));
            motor2PWM = speed;
        }
    }

    else if (speed < 0) {
        if (motor == 1) {
            digitalWrite(IN_1, LOW);
            digitalWrite(IN_2, HIGH);
            analogWrite(EN_A, (uint32_t)(-speed * 2.55F));
            motor1PWM = speed;
        } else {
            digitalWrite(IN_3, LOW);
            digitalWrite(IN_4, HIGH);
            analogWrite(EN_B, (uint32_t)(-speed * 2.55F));
            motor2PWM = speed;
        }
    }

    else {
        coastMotor(motor);
    }

    return;
}

void MotorController::coastMotor(int motor) {
    if (motor != 1 && motor != 2) {
        return;
    }

    else if (motor == 1) {
        analogWrite(EN_A, 0);
        motor1PWM = 0;
    }

    else {
        analogWrite(EN_B, 0);
        motor1PWM = 0;
    }
}

void MotorController::setMotors(float speed) {
    if (abs(speed) > 100) {
        return;
    }

    if (speed > 0) {
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);

        digitalWrite(IN_3, HIGH);
        digitalWrite(IN_4, LOW);

        analogWrite(EN_A, (uint32_t)(speed * 2.55F));
        analogWrite(EN_B, (uint32_t)(speed * 2.55F));
        motor1PWM = speed;
        motor2PWM = speed;
    }

    else if (speed < 0) {
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, HIGH);

        digitalWrite(IN_3, LOW);
        digitalWrite(IN_4, HIGH);

        analogWrite(EN_A, (uint32_t)(-speed * 2.55F));
        analogWrite(EN_B, (uint32_t)(-speed * 2.55F));
        motor1PWM = speed;
        motor2PWM = speed;
    }

    else coastMotors();
}

void MotorController::stopMotors() {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, LOW);

    analogWrite(EN_A, 255);
    analogWrite(EN_B, 255);

    motor1PWM = 0;
    motor2PWM = 0;
}

void MotorController::coastMotors() {
    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);

    motor1PWM = 0;
    motor2PWM = 0;
}

int MotorController::getSpeed(int Motor) {
    if (Motor != 1 && Motor != 2) {
        return 0;
    }

    if (Motor == 1) {
        return motor1PWM;
    } else {
        return motor2PWM;
    }
}