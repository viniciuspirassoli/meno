#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController(int ENA, int IN1, int IN2, int IN3, int IN4,
                                 int ENB, int PWMCH1, int PWMCH2) {
    EN_A = ENA;
    IN_1 = IN1;
    IN_2 = IN2;
    IN_3 = IN3;
    IN_4 = IN4;
    EN_B = ENB;
    PWM_CH1 = PWMCH1;
    PWM_CH2 = PWMCH2;
}

void MotorController::begin() {
    ledcSetup(PWM_CH1, 490, 8);
    ledcSetup(PWM_CH2, 490, 8);

    ledcAttachPin(EN_A, PWM_CH1);
    ledcAttachPin(EN_B, PWM_CH2);

    pinMode(IN_1, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_4, OUTPUT);
}

void MotorController::setMotorSpeed(float speed, int motor) {
    if ((motor != 1 && motor != 2) || abs(speed) > 100.0F) {
        return;
    }

    if (speed > 0) {
        if (motor == 1) {
            digitalWrite(IN_1, HIGH);
            digitalWrite(IN_2, LOW);
            ledcWrite(PWM_CH1, (uint32_t)(speed * 2.55F));
            motor1Speed = speed;
        }

        else {
            digitalWrite(IN_3, HIGH);
            digitalWrite(IN_4, LOW);
            ledcWrite(PWM_CH2, (uint32_t)(speed * 2.55F));
            motor2Speed = speed;
        }
    }

    else if (speed < 0) {
        if (motor == 1) {
            digitalWrite(IN_1, LOW);
            digitalWrite(IN_2, HIGH);
            ledcWrite(PWM_CH1, (uint32_t)(-speed * 2.55F));
            motor1Speed = speed;
        } else {
            digitalWrite(IN_3, LOW);
            digitalWrite(IN_4, HIGH);
            ledcWrite(PWM_CH2, (uint32_t)(-speed * 2.55F));
            motor2Speed = speed;
        }
    }

    else {
        coastMotors();
    }

    return;
}

void MotorController::stopMotors() {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, LOW);

    ledcWrite(PWM_CH1, 255);
    ledcWrite(PWM_CH2, 255);

    motor1Speed = 0;
    motor2Speed = 0;
}

void MotorController::coastMotors() {
    ledcWrite(PWM_CH1, 0);
    ledcWrite(PWM_CH2, 0);

    motor1Speed = 0;
    motor2Speed = 0;
}

int MotorController::getSpeed(int Motor) {
    if (Motor != 1 && Motor != 2) {
        return 0;
    }

    if (Motor == 1) {
        return motor1Speed;
    } else {
        return motor2Speed;
    }
}