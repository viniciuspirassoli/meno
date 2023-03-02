#include <PID_v1.h>
#include <MotorDriver.h>
#include <EncoderHandler.h>
#include <SimplyAtomic.h>

class MotorController{
    //TODO: see if we need more methods, and make them!
    public:
        #define LEFT 1
        #define RIGHT 2
        #define PID_MULTIPLIER 55.866 //scales the PID output

        /**
        *@brief Constructor of Motor Controller
        *
        *@param uint8_t Pin to ENA i.e. RIGHT motor enable pin that receives PWM signal
        *@param uint8_t First pin to set RIGHT motor direction
        *@param uint8_t Second pin to set RIGHT motor direction
        *@param uint8_t First pin to set LEFT motor direction
        *@param uint8_t Second pin to set LEFT motor direction
        *@param uint8_t Pin to ENB i.e. LEFT motor enable pin that receives PWM
        *@param uint8_t LEFT motor enconder first pin
        *@param uint8_t LEFT motor encoder second pin
        *@param uint8_t RIGHT motor encoder first pin
        *@param uint8_t RIGHT motor encoder second pin
        *@param double LEFT motor KP constant to set PID
        *@param double LEFT motor KI constant to set PID
        *@param double LEFT motor KD constant to set PID
        *@param double RIGHT motor KP constant to set PID
        *@param double RIGHT motor KI constant to set PID
        *@param double RIGHT motor KD constant to set PID
        */
        MotorController(uint8_t EN_A, uint8_t IN_1, uint8_t IN_2, uint8_t IN_3, uint8_t IN_4, uint8_t EN_B,
                        uint8_t leftMotorEnc1, uint8_t leftMotorEnc2, uint8_t rightMotorEnc1, uint8_t rightMotorEnc2,
                        double left_KP, double left_KI, double left_KD, double right_KP, double right_KI, double right_KD);
        ~MotorController();

        void setup();
        void loop();

        void setFilterSize(int newFilterSize);
        void setTargetVelocities(double targetLeft, double targetRight);

        double getAvgVelocity(int motor);

        void stop();
        void coast();

    private:
        MotorDriver* motorDriver;
        PID *leftPID, *rightPID;
        EncoderHandler *leftEH, *rightEH;
        
        //vector to filter, target to PID, Input to PID for the left motor
        double* leftVelocities;
        double leftTargetVelocity;
        double leftAvgVelocity;
        double leftCurrVelocity;

        //vector to filter, target to PID, Input to PID for the right motor
        double* rightVelocities; 
        double rightTargetVelocity;
        double rightAvgVelocity;
        double rightCurrVelocity;

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
        
};