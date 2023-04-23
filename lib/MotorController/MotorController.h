#include <PID_v1.h>
#include <MotorDriver.h>
#include <EncoderHandler.h>
#include <SimplyAtomic.h>

class MotorController{
    public:
        #define LEFT 2
        #define RIGHT 1
        #define PID_MULTIPLIER 55.866 //scales the PID output
        #define MAX_WHEEL_W 1.79 //in degrees per milisecond
        #define MAX_ROBOT_V 0.906 // in m/s
        #define MAX_ROBOT_W 9.06 // in rad/s
        #define WHEEL_RADIUS 0.029
        #define WHEELS_DISTANCE 0.2 

        /**
        *@brief Constructor of Motor Controller
        *
        *@param EN_A Pin to ENA i.e. RIGHT motor enable pin that receives PWM signal
        *@param IN_1 First pin to set RIGHT motor direction
        *@param IN_2 Second pin to set RIGHT motor direction
        *@param IN_3 First pin to set LEFT motor direction
        *@param IN_4 Second pin to set LEFT motor direction
        *@param EN_B Pin to ENB i.e. LEFT motor enable pin that receives PWM
        *@param leftMotorEnc1 LEFT motor enconder first pin
        *@param leftMotorEnc2 LEFT motor encoder second pin
        *@param rightMotorEnc1 RIGHT motor encoder first pin
        *@param rightMotorEnc2 RIGHT motor encoder second pin
        *@param left_KP LEFT motor KP constant to set PID
        *@param left_KI LEFT motor KI constant to set PID
        *@param left_KD LEFT motor KD constant to set PID
        *@param right_KP RIGHT motor KP constant to set PID
        *@param right_KI RIGHT motor KI constant to set PID
        *@param right_KD RIGHT motor KD constant to set PID
        */
        MotorController(uint8_t EN_A, uint8_t IN_1, uint8_t IN_2, uint8_t IN_3, uint8_t IN_4, uint8_t EN_B,
                        uint8_t leftMotorEnc1, uint8_t leftMotorEnc2, uint8_t rightMotorEnc1, uint8_t rightMotorEnc2,
                        double left_KP, double left_KI, double left_KD, double right_KP, double right_KI, double right_KD);
        ~MotorController();

        void setup();
        void loop();

        void setFilterSize(int newFilterSize);
        void setTargetW(int motor, double target);
        void setTargetW(double targetLeft, double targetRight);
        //TODO: create setRobotW and setRobotV

        double getAvgW(int motor); //degrees per milisecond
        void setRobotW(double w);
        void setRobotV(double v);

        EncoderHandler* getEH(int dir);

        void setPIDTunings(double new_KP, double new_KI, double new_KD);

        void setPIDTuning(int motor, double new_KP, double new_KI, double new_KD);

        void stop();
        void coast();
        void printOdometry();

    private:
        MotorDriver* motorDriver;
        PID *leftPID, *rightPID;
        EncoderHandler *leftEH, *rightEH;
        
        //vector to filter, target to PID, Input to PID for the left motor
        double* leftWs;
        double leftTargetW;
        double leftAvgW;
        double leftCurrW;

        //vector to filter, target to PID, Input to PID for the right motor
        double* rightWs; 
        double rightTargetW;
        double rightAvgW;
        double rightCurrW;

        //Rolling Filter variables
        int filterSize;
        int velIndex;
        double leftSum;
        double rightSum;

        //Constants for PIDs controllers and outputs
        double leftKP, leftKI, leftKD;
        double rightKP, rightKI, rightKD;
        double leftPIDout, rightPIDout;
        
        //Enconder Counters
        int currEncCountLeft;
        int currEncCountRight;
        int prevEncCountLeft;
        int prevEncCountRight;

        long currTime_us;
        long prevTime_us;

        //Pins
        uint8_t enA, in1, in2, in3, in4, enB;

        //Estimation and odometry
        float wLeft, wRight;
        float vLeft, vRight;
        float vRobot, wRobot; //to be adjusted to meet targets
        float dSpace, dTheta;
        float xEstimated, yEstimated, thetaEstimated; //absolute
        float relativeSpace, relativeTheta; //relative

        double targetV, targetW; //targets received from serial port
        double vRightT, vLeftT;

        double deltaT;
        

        
};