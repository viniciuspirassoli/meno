/*
#include <PID_v1.h>
#include <MotorDriver.h>
#include <EncoderHandler.h>
#include <SimplyAtomic.h>

class MotorController{
    public:
        MotorController(MotorDriver m_d, EncoderHandler left_EH, EncoderHandler right_EH);
        ~MotorController();

        void setup();
        void loop();

        void setFilterSize(int newFilterSize);

    private:
        MotorDriver* md;
        PID leftPID, rightPID;
        EncoderHandler *leftEH, *rightEH;

        int filterSize;
        
        //vector to filter, target to PID, Input to PID for the left motor
        double* leftVelocities;
        double leftTargetVelocity;
        double leftAvgVelocity;

        //vector to filter, target to PID, Input to PID for the right motor
        double* rightVelocities; 
        double rightTargetVelocity;
        double rightAvgVelocity;

        int velIndex;

        //Constants for PIDs controllers
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


        
};
*/