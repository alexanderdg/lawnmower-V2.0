#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include<Arduino.h>
#include <PID_v1.h>

#define DIAMETER_WHEEL 120
#define ENCODER_COUNTS_PER_REVOLUTION 3591.84

class MotorDriver
{
  public:
    MotorDriver();
    bool Forward(float speed);
    bool Backward(float speed);
    bool turnRight(float speed);
    bool turnLeft(float speed);
    bool coastBrake(void);
    bool dynamicBrake(void);
    bool takeRandomTurnRight(void);
    bool takeRandomTurnLeft(void);

    bool rawRight(bool direction, int pwm);
    bool rawLeft(bool direction, int pwm);

    bool enableVBlade(void);
    bool disableVBlade(void);
    bool startTurning(void);
    bool bladeStop(void);
    bool bladeEmergencyStop(void);
    
    float getLeftSpeed(void);
    float getRightSpeed(void);
    float getLeftCurrent(void);
    float getRightCurrent(void);
    float getBladeCurrent(void);

    bool selfTest(void);

  private:
    static void EncoderInteruptL();
    static void EncoderInteruptR();
    static void resetCounter();
    static void currentSampler();
    static MotorDriver* MotorDriverS;
    IntervalTimer resetCounterTimer;
    IntervalTimer sampleCurrentTimer;

    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);

    int MStoCPMSconverter(float speed);
    float CPMStoMSconverter(int speed);    
    float radiusWheel = DIAMETER_WHEEL / 2000.0;
    float perimeterWheel = 2 * PI * radiusWheel;
    

    //variables used to measure the motor speed
    int temp_counter_l = 0;
    int temp_counter_r = 0;
    int counter_l = 0;
    long total_counter_1 = 0;
    int counter_r = 0;
    long total_counter_r = 0;
    bool en_pid = true;
    bool en_distance = false; 

    //variables for the current measurment
    int leftCurrent = 0;
    int rightCurrent = 0;
    int bladeCurrent = 0;

    //PID variables
    double SetpointL, InputL, OutputL;
    PID LPID;
    double SetpointR, InputR, OutputR;
    PID RPID;
    
    //pin definitons of the lawnmower control board V2.0
    int RL_EN = 24;
    int RL_SLEEP = 12;
    int R_DIR = 25;
    int R_PWM = 30;
    int L_PWM = 29;
    int L_DIR = 0;
    int L_CURRENT = 21;
    int R_CURRENT = 32;
    int L_ENCA = 20;
    int L_ENCB = 17;
    int R_ENCA = 28;
    int R_ENCB = 27;

    int EN_VBLADE = 13;
    int B_EN = 5;
    int B_SLEEP = 2;
    int B_DIR = 6;
    int B_PWM = 10;
    int B_CURRENT = 31;
};


#endif
