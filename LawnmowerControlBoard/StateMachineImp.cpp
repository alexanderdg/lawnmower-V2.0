#include<Arduino.h>
#include "StateMachineImp.h"

StateMachineType StateMachineImp::stateMachine[] =       {
  {SELF_TEST, SM_SELF_TEST},
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

StateType StateMachineImp::SM_STATE = SELF_TEST;
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
  delay(100);
}

void StateMachineImp::SM_SELF_TEST(void)
{
  /*
  Serial.println("Starting selftest of the motordriver");
  if(motordriver -> selfTest() == true)
  {
    Serial.println("Selftest of the motordriver is ok");
    Serial.println("Starting selftest of the sensorboard");
    if(canbus -> selfTest() == true)
    {
       Serial.println("Selftest of the sensorboard is ok");
       Serial.println("Starting selftest of the battery driver");
       if(batterydriver -> selfTest() == true)
       {
            Serial.println("Selftest of the batterydrive is ok");
            Serial.println("Selftest of al the components is finished");
       }
       else
       {
            Serial.println("Selftest of the batterydriver is failed");
       }
    }
    else
    {
      Serial.println("Selftest of the sensorboard is failed");
    }
  }
  else
  {
     Serial.println("Selftest of the motordriver is failed");
  }
  delay(1000);
  changeState(STUCK);
  */
  Serial.println(canbus -> readDistanceSensor(1));
  Serial.println(canbus -> readDistanceSensor(2));
  Serial.println(canbus -> readDistanceSensor(3));
  Serial.println(canbus -> readDistanceSensor(4));
  Serial.println(canbus -> readDistanceSensor(5));
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
    //Serial.println("State RUN");
    //Serial.print("blade current: ");
    //Serial.println(motordriver -> getLeftCurrent());
    
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
    motordriver -> Backward(0.3);
    delay(1000);
    motordriver -> takeRandomTurnRight();
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
