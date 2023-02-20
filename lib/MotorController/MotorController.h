
class MotorController {
  public:
    MotorController(int ENA, int IN1, int IN2, int IN3, int IN4, int ENB, int ENC1, int ENC2, int ENC3, int ENC4);
    void begin();

    void setMotorSpeed(float speed, int motor);
    void stopMotors();
    void coastMotors();
    void setMotors(float speed);
    void coastMotor(int motor);
    int getSpeed(int Motor);

  private:
    // pin nums
    int EN_A;
    int IN_1;
    int IN_2;
    int IN_3;
    int IN_4;
    int EN_B;
    int ENC_1;
    int ENC_2;
    int ENC_3;
    int ENC_4;

    // PWM - from 0 to 100
    float motor1PWM;
    float motor2PWM;
};