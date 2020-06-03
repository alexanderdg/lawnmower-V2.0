#ifndef STATEMACHINEIMP_H
#define STATEMACHINEIMP_H

#include<Arduino.h>
#include "MotorDriver.h"
#include "Battery.h"
#include "Speaker.h"
#include "CANbus.h"

#define NUM_STATES 12

enum StateType
{
  SELF_TEST,
  RUN,
  RUN_SLOW,
  TRY_LEFT,
  TRY_RIGHT,
  TRY_INSTEAD_LEFT,
  TRY_INSTEAD_RIGHT,
  TRY_LAST_TIME_LEFT,
  TRY_LAST_TIME_RIGHT,
  TRY_BACKWARD,
  STUCK,
  CHARGING
};

struct StateMachineType
{
  StateType State;       // Define the command
  void(*func)(void);     // Defines the function to run
};

class StateMachineImp
{

  public:

    static void RunStatemachine(void);
    static void initStatemachine(void);

  private:

    static void SM_SELF_TEST(void);
    static void SM_RUN(void);
    static void SM_RUN_SLOW(void);      
    static void SM_TRY_LEFT(void);      
    static void SM_TRY_RIGHT(void);    
    static void SM_TRY_INSTEAD_LEFT(void);
    static void SM_TRY_INSTEAD_RIGHT(void);
    static void SM_TRY_LAST_TIME_LEFT(void);
    static void SM_TRY_LAST_TIME_RIGHT(void);
    static void SM_TRY_BACKWARD(void);
    static void SM_STUCK(void);
    static void SM_CHARGING(void);
    static void checkForCharger(void);

    static void changeState(StateType newState);


    /*
      StateMachineType stateMachine[]=
      {
      {STATE_ONE, SM_STATE_ONE},
      {STATE_TWO, SM_STATE_TWO},
      {STATE_THREE, SM_STATE_THREE},
      {STATE_FOUR, SM_STATE_FOUR}
      };
    */
    static StateMachineType stateMachine[];
    static StateType SM_STATE;
    static MotorDriver *motordriver;
    static Battery *batterydriver;
    static Speaker *speaker;
    static CANbus *canbus;
    static bool enter_state;

};


#endif
