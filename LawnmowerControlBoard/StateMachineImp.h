#ifndef STATEMACHINEIMP_H
#define STATEMACHINEIMP_H

#include<Arduino.h>
#include "MotorDriver.h"

#define NUM_STATES 4

enum StateType
{
  STATE_ONE,
  STATE_TWO,
  STATE_THREE,
  STATE_FOUR
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


  private:

    static void SM_STATE_ONE(void);      // State One
    static void SM_STATE_TWO(void);      // State Two
    static void SM_STATE_THREE(void);    // State Three
    static void SM_STATE_FOUR(void);     // State Four

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
    static bool enter_state;

};


#endif
