#include "MotorDriver.h"
#include <Arduino.h>

MotorDriver::MotorDriver(int ENA, int IN1, int IN2, int IN3, int IN4,
                                 int ENB) {
    EN_A = ENA;
    IN_1 = IN1;
    IN_2 = IN2;
    IN_3 = IN3;
    IN_4 = IN4;
    EN_B = ENB;

}

void MotorDriver::begin() {
    //PWM outputs
    pinMode(EN_A, OUTPUT);
    pinMode(EN_B, OUTPUT);

    //Digital outputs
    pinMode(IN_1, OUTPUT);
    pinMode(IN_2, OUTPUT);
    pinMode(IN_3, OUTPUT);
    pinMode(IN_4, OUTPUT);
}


void MotorDriver::setMotorSpeed(float speed, int motor) {

    if ((motor != 1 && motor != 2))
        return;

    if (speed > 100.0F)
        speed = 100.0F;

    else if (speed < -100.F) 
        speed = -100.0F;

    if (speed > 0) {
        if (motor == 1) {
            digitalWrite(IN_1, HIGH);
            digitalWrite(IN_2, LOW);
            analogWrite(EN_A, (uint32_t)(speed * 2.55F));
            motor1PWM = speed;
            /*
            Serial.println("Debug: Left Motor set to ");
            Serial.print((uint32_t)(speed * 2.55F));
            Serial.println("");
            */
        }

        else {
            digitalWrite(IN_3, HIGH);
            digitalWrite(IN_4, LOW);
            analogWrite(EN_B, (uint32_t)(speed * 2.55F));
            motor2PWM = speed;
            /*
            Serial.println("Debug: Right Motor set to ");
            Serial.print((uint32_t)(speed * 2.55F));
            Serial.println("");
            */
        }
    }

    else if (speed < 0) {
        if (motor == 1) {
            digitalWrite(IN_1, LOW);
            digitalWrite(IN_2, HIGH);
            analogWrite(EN_A, (uint32_t)(-speed * 2.55F));
            motor1PWM = speed;
            
            // Serial.print("Debug: Left Motor speed: ");
            // Serial.println(speed);
            // Serial.print("Debug: Left Motor set to ");
            // Serial.print((uint32_t)(-speed * 2.55F));
            // Serial.println("");
            
        } else {
            digitalWrite(IN_3, LOW);
            digitalWrite(IN_4, HIGH);
            analogWrite(EN_B, (uint32_t)(-speed * 2.55F));
            motor2PWM = speed;
            /*
            Serial.println("Debug: Right Motor set to ");
            Serial.print((uint32_t)(speed * 2.55F));
            Serial.println("");
            */
        }
    }

    else {
        coastMotor(motor);
        /*
        if (motor == 1) Serial.println("Debug: Left motor stopped.");
        else Serial.println("Debug: Right motor stopped.");
        */
    }

    return;
}

void MotorDriver::coastMotor(int motor) {
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

void MotorDriver::setMotors(float speed) {
    if (speed > 100) {
        speed = 100.0F;
    }
    else if (speed < -100) {
        speed = -100.0F;
    }

    if (speed > 0) {
        digitalWrite(IN_1, HIGH);
        digitalWrite(IN_2, LOW);

        digitalWrite(IN_3, HIGH);
        digitalWrite(IN_4, LOW);

        analogWrite(EN_A, (uint32_t)(speed * 2.55F));
        analogWrite(EN_B, (uint32_t)(speed * 2.55F));

        /*
        Serial.println("Debug: Motors set to ");
        Serial.print((uint32_t)(-speed * 2.55F));
        Serial.println("");
        */

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

        /*
        Serial.println("Debug: Motors set to ");
        Serial.print((uint32_t)(-speed * 2.55F));
        Serial.println("");
        */

        motor1PWM = speed;
        motor2PWM = speed;
    }

    else coastMotors();
}

void MotorDriver::stopMotor(int motor) {
    if (motor == 1) {
        digitalWrite(IN_1, LOW);
        digitalWrite(IN_2, LOW);
        digitalWrite(EN_A, LOW);
    }

    else if (motor == 2) {
        digitalWrite(IN_3, LOW);
        digitalWrite(IN_4, LOW);
        digitalWrite(EN_B, LOW);
    }

    else stopMotors();
}

void MotorDriver::stopMotors() {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, LOW);

    analogWrite(EN_A, 255);
    analogWrite(EN_B, 255);

    motor1PWM = 255;
    motor2PWM = 255;
}

void MotorDriver::coastMotors() {
    digitalWrite(IN_1, LOW);
    digitalWrite(IN_2, LOW);
    digitalWrite(IN_3, LOW);
    digitalWrite(IN_4, LOW);

    analogWrite(EN_A, 0);
    analogWrite(EN_B, 0);

    motor1PWM = 0;
    motor2PWM = 0;
}

int MotorDriver::getSpeed(int Motor) {
    if (Motor != 1 && Motor != 2) {
        return 0;
    }

    if (Motor == 1) {
        return motor1PWM;
    } else {
        return motor2PWM;
    }
}