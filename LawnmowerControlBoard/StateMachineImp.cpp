#include<Arduino.h>
#include "StateMachineImp.h"


void StateMachineImp::RunStatemachine(void)
{
    // Make Sure States is valid
    if (SM_STATE < NUM_STATES)
    {
        // Call function for state
        //(*StateMachine[SM_STATE].func)();
    }
}

void StateMachineImp::SM_STATE_ONE(void)
{
  Serial.println("State one");
  delay(1000);
  SM_STATE = STATE_TWO;
}

void StateMachineImp::SM_STATE_TWO(void)
{
  Serial.println("State two");
  delay(1000);
  SM_STATE = STATE_THREE;
}

void StateMachineImp::SM_STATE_THREE(void)
{
  Serial.println("State three");
  delay(1000);
  SM_STATE = STATE_FOUR;
}

void StateMachineImp::SM_STATE_FOUR(void)
{
  Serial.println("State four");
  delay(1000);
  SM_STATE = STATE_ONE;
}
