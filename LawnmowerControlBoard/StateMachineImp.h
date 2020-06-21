#ifndef STATEMACHINEIMP_H
#define STATEMACHINEIMP_H

#include<Arduino.h>
#include "MotorDriver.h"
#include "Battery.h"
#include "Speaker.h"
#include "CANbus.h"

#define NUM_STATES 18

enum StateType
{
  INIT,
  SELF_TEST,
  RUN,
  RUN_SLOW,
  AVOID_PERI,
  TRY_LEFT,
  TRY_RIGHT,
  TRY_INSTEAD_LEFT,
  TRY_INSTEAD_RIGHT,
  TRY_LAST_TIME_LEFT,
  TRY_LAST_TIME_RIGHT,
  TRY_BACKWARD,
  STUCK,
  FIND_PERIMETER,
  RANDOM_TURN,
  RETURN_HOME,
  CHARGING,
  SERROR,
};

struct StateMachineType
{
  StateType State;       // Define the command
  void(*func)(void);     // Defines the function to run
};

class StateMachineImp
{

  public:

    StateMachineImp();
    static void RunStatemachine(void);
    static void initStatemachine(void);
    static void changeState(StateType newState);
    static void printDiagnostics(void);
    static void printPIDValues(void);
    static void changePValue(float value);
    static void changeIValue(float value);
    static void changeDValue(float value);

  private:

    static void SM_INIT(void);
    static void SM_SELF_TEST(void);
    static void SM_RUN(void);
    static void SM_RUN_SLOW(void);
    static void SM_AVOID_PERI(void);      
    static void SM_TRY_LEFT(void);      
    static void SM_TRY_RIGHT(void);    
    static void SM_TRY_INSTEAD_LEFT(void);
    static void SM_TRY_INSTEAD_RIGHT(void);
    static void SM_TRY_LAST_TIME_LEFT(void);
    static void SM_TRY_LAST_TIME_RIGHT(void);
    static void SM_TRY_BACKWARD(void);
    static void SM_STUCK(void);
    static void SM_FIND_PERIMETER(void);
    static void SM_RANDOM_TURN(void);
    static void SM_RETURN_HOME(void);
    static void SM_CHARGING(void);
    static void SM_SERROR(void);
    
    static void checkForCharger(void);

    

    static StateMachineType stateMachine[];
    static StateType SM_STATE;
    static MotorDriver *motordriver;
    static Battery *batterydriver;
    static Speaker *speaker;
    static CANbus *canbus;
    
    static bool enter_state;
    static long savedTimestamp;

    //PID variables
    static  PID * perimeterPID;
    static  double  Setpoint, Input, Output;
    static float pvalue, ivalue, dvalue;

};


#endif
