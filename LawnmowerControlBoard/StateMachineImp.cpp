#include<Arduino.h>
#include "StateMachineImp.h"

StateMachineType StateMachineImp::stateMachine[] =       {
  {STATE_ONE, SM_STATE_ONE},
  {STATE_TWO, SM_STATE_TWO},
  {STATE_THREE, SM_STATE_THREE},
  {STATE_FOUR, SM_STATE_FOUR}
};
StateType StateMachineImp::SM_STATE = STATE_ONE;
MotorDriver *StateMachineImp::motordriver = new MotorDriver();
bool StateMachineImp::enter_state = true;

void StateMachineImp::RunStatemachine(void)
{
  // Make Sure States is valid
  if (SM_STATE < NUM_STATES)
  {
    // Call function for state
    //SM_STATE.func()
    (*StateMachineImp::stateMachine[SM_STATE].func)();
  }
  else
  {
    Serial.println("Error in the execution of the statemachine");
  }
}

void StateMachineImp::SM_STATE_ONE(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter one state");
    enter_state = false;
  }
  else
  {
    Serial.println("State one");
    Serial.print("blade current: ");
    Serial.println(motordriver -> getBladeCurrent());
    delay(1000);
    changeState(STATE_TWO);
  }
}

void StateMachineImp::SM_STATE_TWO(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter two state");
    enter_state = false;
  }
  else
  {
    Serial.println("State two");
    delay(1000);
    changeState(STATE_THREE);
  }
}

void StateMachineImp::SM_STATE_THREE(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter three state");
    enter_state = false;
  }
  else
  {
    Serial.println("State three");
    delay(1000);
    changeState(STATE_FOUR);
  }
}

void StateMachineImp::SM_STATE_FOUR(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter four state");
    enter_state = false;
  }
  else
  {
    Serial.println("State four");
    delay(1000);
    changeState(STATE_ONE);
  }
}

void StateMachineImp::changeState(StateType newState)
{
  SM_STATE = newState;
  enter_state = true;
}
