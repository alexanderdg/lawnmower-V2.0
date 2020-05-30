#include<Arduino.h>
#include "StateMachineImp.h"

StateMachineType StateMachineImp::stateMachine[] =       {
  {RUN, SM_RUN},
  {RUN_SLOW, SM_RUN_SLOW},
  {TRY_LEFT, SM_TRY_LEFT},
  {TRY_RIGHT, SM_TRY_RIGHT},
  {TRY_INSTEAD_LEFT, SM_TRY_INSTEAD_LEFT},
  {TRY_INSTEAD_RIGHT, SM_TRY_INSTEAD_RIGHT},
  {TRY_LAST_TIME_LEFT, SM_TRY_LAST_TIME_LEFT},
  {TRY_LAST_TIME_RIGHT, SM_TRY_LAST_TIME_RIGHT},
  {TRY_BACKWARD, SM_TRY_BACKWARD},
  {STUCK, SM_STUCK},
  {CHARGING, SM_CHARGING}
};

StateType StateMachineImp::SM_STATE = RUN;
MotorDriver *StateMachineImp::motordriver = new MotorDriver();
Battery *StateMachineImp::batterydriver = new Battery();
Speaker *StateMachineImp::speaker = new Speaker();
CANbus *StateMachineImp::canbus = new CANbus();
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

void StateMachineImp::initStatemachine(void)
{
  speaker -> init();
  canbus -> initCAN();
  motordriver -> coastBrake();
}

void StateMachineImp::SM_RUN(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter RUN state");
    enter_state = false;
    speaker -> playStartMowing();
    motordriver -> Forward(0.5);
  }
  else
  {
    checkForCharger();
    if(motordriver -> getLeftCurrent() > 1.6 or motordriver -> getRightCurrent() > 1.6 or canbus -> readPressure1() > 75 or canbus -> readPressure2() > 75)
    {
      changeState(TRY_LEFT);
    }
    if(batterydriver -> isChargerPresent())
    {
      changeState(CHARGING);
    }
    //Serial.println("State RUN");
    //Serial.print("blade current: ");
    //Serial.println(motordriver -> getLeftCurrent());
    delay(10);
    
  }
}

void StateMachineImp::SM_RUN_SLOW(void)
{

}


void StateMachineImp::SM_TRY_LEFT(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter TRY_LEFT state");
    enter_state = false;
    motordriver -> coastBrake();
    speaker -> playStuck();
  }
  else
  {
    Serial.println("State TRY_LEFT");
    delay(1000);
    changeState(TRY_RIGHT);
  }
}

void StateMachineImp::SM_TRY_RIGHT(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter TRY_RIGHT state");
    enter_state = false;
  }
  else
  {
    Serial.println("State TRY_RIGHT");
    delay(1000);
    changeState(STUCK);
  }
}

void StateMachineImp::SM_TRY_INSTEAD_LEFT(void)
{

}

void StateMachineImp::SM_TRY_INSTEAD_RIGHT(void)
{

}

void StateMachineImp::SM_TRY_LAST_TIME_LEFT(void)
{

}

void StateMachineImp::SM_TRY_LAST_TIME_RIGHT(void)
{

}


void StateMachineImp::SM_STUCK(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter STUCK state");
    enter_state = false;
  }
  else
  {
    Serial.println("State STUCK");
    delay(1000);
    changeState(RUN);
  }
}

void StateMachineImp::SM_TRY_BACKWARD(void)
{

  
}

void StateMachineImp::SM_CHARGING(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter CHARGING state");
    enter_state = false;
    speaker -> playStartCharging();
    //batterydriver -> disableCharger();
  }
  else
  {
      ChargeState state = batterydriver -> readChargeState();
      Serial.println(batterydriver -> enumToString(state));
      //Serial.println(state);
  }
}

void StateMachineImp::changeState(StateType newState)
{
  SM_STATE = newState;
  enter_state = true;
}


//---------------------------------------------------------------------
//---         Check methods                                         ---
//---------------------------------------------------------------------

void StateMachineImp::checkForCharger(void)
{
  if(batterydriver -> isChargerPresent())
  {
    changeState(CHARGING);
  }
}
