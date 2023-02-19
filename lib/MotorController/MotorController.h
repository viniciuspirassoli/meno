
class MotorController {
  public:
    MotorController(int ENA, int IN1, int IN2, int IN3, int IN4, int ENB,
                    int PWMCH1, int PWMCH2);
    void begin();

    void setMotorSpeed(float speed, int motor);
    void stopMotors();
    void coastMotors();
    int getSpeed(int Motor);

  private:
    // pin nums
    int EN_A;
    int IN_1;
    int IN_2;
    int IN_3;
    int IN_4;
    int EN_B;

    // PWM Channels
    int PWM_CH1;
    int PWM_CH2;

    // Speed Values
    float motor1Speed;
    float motor2Speed;
};