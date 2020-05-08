#ifndef STATEMACHINEIMP_H
#define STATEMACHINEIMP_H

#include<Arduino.h>

#define NUM_STATES 4



class StateMachineImp
{

  public:

    void RunStatemachine(void);


  private:

    void SM_STATE_ONE(void);      // State One
    void SM_STATE_TWO(void);      // State Two
    void SM_STATE_THREE(void);    // State Three
    void SM_STATE_FOUR(void);     // State Four

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

    /*
      StateMachineType stateMachine[]
      {
      {STATE_ONE, SM_STATE_ONE},
      {STATE_TWO, SM_STATE_TWO},
      {STATE_THREE, SM_STATE_THREE},
      {STATE_FOUR, SM_STATE_FOUR}
      };
    */
    StateMachineType stateMachine = {STATE_ONE, &SM_STATE_ONE};
    StateType SM_STATE = STATE_ONE;


};


#endif
