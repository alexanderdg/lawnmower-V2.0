#include<Arduino.h>
#include "StateMachineImp.h"

StateMachineType StateMachineImp::stateMachine[] =       {
  {INIT, SM_INIT},
  {SELF_TEST, SM_SELF_TEST},
  {RUN, SM_RUN},
  {RUN_SLOW, SM_RUN_SLOW},
  {AVOID_PERI, SM_AVOID_PERI},
  {TRY_LEFT, SM_TRY_LEFT},
  {TRY_RIGHT, SM_TRY_RIGHT},
  {TRY_INSTEAD_LEFT, SM_TRY_INSTEAD_LEFT},
  {TRY_INSTEAD_RIGHT, SM_TRY_INSTEAD_RIGHT},
  {TRY_LAST_TIME_LEFT, SM_TRY_LAST_TIME_LEFT},
  {TRY_LAST_TIME_RIGHT, SM_TRY_LAST_TIME_RIGHT},
  {TRY_BACKWARD, SM_TRY_BACKWARD},
  {STUCK, SM_STUCK},
  {FIND_PERIMETER, SM_FIND_PERIMETER},
  {RANDOM_TURN, SM_RANDOM_TURN},
  {RETURN_HOME, SM_RETURN_HOME},
  {CHARGING, SM_CHARGING},
  {SERROR, SM_SERROR}
};

StateType StateMachineImp::SM_STATE = RUN;
MotorDriver *StateMachineImp::motordriver = new MotorDriver();
Battery *StateMachineImp::batterydriver = new Battery();
Speaker *StateMachineImp::speaker = new Speaker();
CANbus *StateMachineImp::canbus = new CANbus();
//PID *StateMachineImp::perimeterPID = new PID(&Input, &Output, &Setpoint, 5, 40, 0.5, DIRECT);
bool StateMachineImp::enter_state = true;
long StateMachineImp::savedTimestamp = 0;

StateMachineImp::StateMachineImp(void)
{
  
}

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
    Serial3.println("Error in the execution of the statemachine");
  }
}

void StateMachineImp::initStatemachine(void)
{
  speaker -> init();
  canbus -> initCAN();
  motordriver -> coastBrake();
  perimeterPID -> SetOutputLimits(0, 1000);
  delay(100);
}

void StateMachineImp::SM_INIT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter INIT state");
    motordriver -> coastBrake();
    motordriver -> bladeStop();
    motordriver -> disableVBlade();
    enter_state = false;
  }
  else
  {

  }
}

void StateMachineImp::SM_SELF_TEST(void)
{
  Serial3.println("Starting selftest of the motordriver");
  if (motordriver -> selfTest() == true)
  {
    Serial3.println("Selftest of the motordriver is ok");
    Serial3.println("Starting selftest of the sensorboard");
    if (canbus -> selfTest() == true)
    {
      Serial3.println("Selftest of the sensorboard is ok");
      Serial3.println("Starting selftest of the battery driver");
      if (batterydriver -> selfTest() == true)
      {
        Serial3.println("Selftest of the batterydrive is ok");
        Serial3.println("Selftest of al the components is finished");
      }
      else
      {
        Serial3.println("Selftest of the batterydriver is failed");
      }
    }
    else
    {
      Serial3.println("Selftest of the sensorboard is failed");
    }
  }
  else
  {
    Serial3.println("Selftest of the motordriver is failed");
  }
  delay(1000);
  speaker -> playStartMowing();
  changeState(RUN);
}

void StateMachineImp::SM_RUN(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter RUN state");
    enter_state = false;
    //motordriver -> Forward(0.5);
    motordriver -> startTurning();
  }
  else
  {
    //Serial3.println(canbus -> readDistanceSensor(LL));
    //Serial3.println(canbus -> readDistanceSensor(LM));
    //Serial3.println(canbus -> readDistanceSensor(RM));
    //Serial3.println(canbus -> readDistanceSensor(RR));
    //Serial3.println(batterydriver -> readVoltage());
    //Serial3.println(batterydriver -> readCurrent());
    int value, sign;
    canbus -> readPerimeter(&value, &sign);
    //Serial3.println(value);
    //Serial3.println(sign);
    //Serial3.println("---------");
    //Serial3.println(canbus -> readPressure1());
    //Serial3.println(canbus -> readPressure2());
    if (motordriver -> getRightCurrent() > 2 or canbus -> readPressure2() > 75)
    {
      changeState(TRY_LEFT);
    }
    if (motordriver -> getLeftCurrent() > 2 or canbus -> readPressure1() > 75)
    {
      changeState(TRY_RIGHT);
    }
    if (canbus -> readDistanceSensor(LL) < 100 or canbus -> readDistanceSensor(LM) < 100 or canbus -> readDistanceSensor(RM) < 100 or canbus -> readDistanceSensor(RR) < 100)
    {
      changeState(RUN_SLOW);
    }
    checkForCharger();
  }
}

void StateMachineImp::SM_RUN_SLOW(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter RUN_SLOW state");
    motordriver -> Forward(0.3);
    enter_state = false;
    savedTimestamp = millis();
    //motordriver -> enableVBlade();
  }
  else
  {
    long currentTimestamp = millis();
    if (motordriver -> getRightCurrent() > 2 or canbus -> readPressure2() > 75)
    {
      changeState(TRY_LEFT);
    }
    if (motordriver -> getLeftCurrent() > 2 or canbus -> readPressure1() > 75)
    {
      changeState(TRY_RIGHT);
    }
    if (canbus -> readDistanceSensor(LL) > 100 and canbus -> readDistanceSensor(LM) > 100 and canbus -> readDistanceSensor(RM) > 100 and canbus -> readDistanceSensor(RR) > 100 and (currentTimestamp - savedTimestamp) > 1000)
    {
      changeState(RUN);
    }
    checkForCharger();
  }
}

void StateMachineImp::SM_AVOID_PERI(void)
{

}


void StateMachineImp::SM_TRY_LEFT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_LEFT state");
    //Serial3.println("test");
    motordriver -> coastBrake();
    savedTimestamp = millis();
    enter_state = false;
  }
  else
  {
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    //Serial3.begin(currentTimestamp);
    //Serial3.begin(savedTimestamp);
    if ( (currentTimestamp - savedTimestamp) < 800 )
    {

    }
    else if ( (currentTimestamp - savedTimestamp) < 2000 )
    {
      motordriver-> Backward(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < 3000)
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < 4000 )
    {
      motordriver -> turnLeft(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < (5000 + randomtime))
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < (6000 + randomtime))
    {
      changeState(RUN);
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp) > 1000)
    {
      changeState(TRY_INSTEAD_LEFT);
    }
  }
}

void StateMachineImp::SM_TRY_RIGHT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_RIGHT state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
  }
  else
  {
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    if ( (currentTimestamp - savedTimestamp) < 800 )
    {

    }
    else if ( (currentTimestamp - savedTimestamp) < 2000 )
    {
      motordriver-> Backward(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < 3000)
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < 4000 )
    {
      motordriver -> turnRight(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < (5000 + randomtime))
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < (6000 + randomtime))
    {
      changeState(RUN);
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp) > 1500)
    {
      changeState(TRY_INSTEAD_LEFT);
    }
  }
}

void StateMachineImp::SM_TRY_INSTEAD_LEFT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_INSTEAD_LEFT state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
  }
  else
  {
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    if ( (currentTimestamp - savedTimestamp) < 800 )
    {

    }
    else if ( (currentTimestamp - savedTimestamp) < 2000 )
    {
      motordriver-> Backward(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < 3000)
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < 4000 )
    {
      motordriver -> turnRight(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < (5000 + randomtime))
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < (6000 + randomtime))
    {
      changeState(RUN);
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp) > 1500)
    {
      changeState(TRY_BACKWARD);
    }
  }
}

void StateMachineImp::SM_TRY_INSTEAD_RIGHT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_INSTEAD_RIGHT state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
  }
  else
  {
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    if ( (currentTimestamp - savedTimestamp) < 800 )
    {

    }
    else if ( (currentTimestamp - savedTimestamp) < 2000 )
    {
      motordriver-> Backward(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < 3000)
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < 4000 )
    {
      motordriver -> turnLeft(0.3);
    }
    else if ( (currentTimestamp - savedTimestamp) < (5000 + randomtime))
    {
      motordriver -> coastBrake();
    }
    else if ( (currentTimestamp - savedTimestamp) < (6000 + randomtime))
    {
      changeState(RUN);
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp) > 1500)
    {
      changeState(TRY_BACKWARD);
    }
  }
}

void StateMachineImp::SM_TRY_LAST_TIME_LEFT(void)
{

}

void StateMachineImp::SM_TRY_LAST_TIME_RIGHT(void)
{

}

void StateMachineImp::SM_TRY_BACKWARD(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter TRY_BACKWARD state");
    enter_state = false;
    motordriver -> coastBrake();
    motordriver -> bladeStop();
  }
  else
  {

  }

}

void StateMachineImp::SM_STUCK(void)
{
  if (enter_state == true)
  {
    Serial.println("Enter STUCK state");
    enter_state = false;
    motordriver -> bladeStop();
  }
  else
  {
    Serial.println("State STUCK");
    delay(1000);
    changeState(RUN);
  }
}

void StateMachineImp::SM_FIND_PERIMETER(void)
{
  if(enter_state == true)
  {
    Serial3.println("Enter FIND_PERIMETER state");
    enter_state = false;
  }
  else
  {
    int value, sign;
    canbus -> readPerimeter(&value, &sign);
    Serial3.print(sign);
    Serial3.print(" : ");
    Serial3.println(value);
    motordriver -> rawLeft(0, 2000);
    motordriver -> rawRight(0, 1000);
  }
}

void StateMachineImp::SM_RANDOM_TURN(void)
{

}

void StateMachineImp::SM_RETURN_HOME(void)
{

}

void StateMachineImp::SM_SERROR(void)
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
    motordriver -> bladeStop();
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
  if (batterydriver -> isChargerPresent())
  {
    changeState(CHARGING);
  }
}
