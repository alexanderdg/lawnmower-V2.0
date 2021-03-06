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

double StateMachineImp::Input = 0;
double StateMachineImp::Output = 0;
double StateMachineImp::Setpoint = 0;
float StateMachineImp::pvalue = 0;
float StateMachineImp::ivalue = 0;
float StateMachineImp::dvalue = 0;
StateType StateMachineImp::SM_STATE = INIT;
MotorDriver *StateMachineImp::motordriver = new MotorDriver();
Battery *StateMachineImp::batterydriver = new Battery();
Speaker *StateMachineImp::speaker = new Speaker();
CANbus *StateMachineImp::canbus = new CANbus();
Settings *StateMachineImp::setting = new Settings();
PID *StateMachineImp::perimeterPID = new PID(&Input, &Output, &Setpoint, 5, 40, 0.5, DIRECT);
bool StateMachineImp::enter_state = true;
long StateMachineImp::savedTimestamp = 0;
long StateMachineImp::savedTimestamp2 = 0;
bool StateMachineImp::collision = false;
int StateMachineImp::internal_state = 0;
bool StateMachineImp::result_selftest = false;
int StateMachineImp::mowEnable = 0;

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
    checkGlobalParameters();
  }
  else
  {
    Serial3.println("Error in the execution of the statemachine (unknown state)");
  }
}

void StateMachineImp::initStatemachine(void)
{
  speaker -> init();
  canbus -> initCAN();
  motordriver -> coastBrake();
  setting -> readPIDValues(&pvalue, &ivalue, &dvalue);
  setting -> readPIDSetpoint(&Setpoint);
  setting -> readMowEnable(&mowEnable);
  perimeterPID -> SetTunings(pvalue, ivalue, dvalue);
  perimeterPID -> SetOutputLimits(0, 6000);
  perimeterPID -> SetSampleTime(15);
  perimeterPID -> SetMode(AUTOMATIC);
  delay(100);
  canbus -> setState(0);
}

void StateMachineImp::SM_INIT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter INIT state");
    motordriver -> coastBrake();
    motordriver -> bladeStop();
    enter_state = false;
    savedTimestamp = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    if((currentTimestamp - savedTimestamp) > 1000)
    {
      motordriver -> disableVBlade();
    }
    
    if (canbus -> readStatus() == 2)
    {
      changeState(SELF_TEST);
    }
    else if (canbus -> readStatus() == 5)
    {
      speaker -> playStartMowing();
      changeState(RUN);
    }
  }
}

void StateMachineImp::SM_SELF_TEST(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter SELF_TEST state");
    motordriver -> coastBrake();
    motordriver -> bladeStop();
    motordriver -> disableVBlade();
    enter_state = false;
    result_selftest = false;
  }
  else
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
          result_selftest = true;
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
    if (result_selftest) canbus -> setStatus(2);
    else canbus -> setStatus(1);
    changeState(INIT);
  }
}

void StateMachineImp::SM_RUN(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter RUN state");
    enter_state = false;
    motordriver -> Forward(0.5);
    if(mowEnable > 0)
    {
      motordriver -> enableVBlade();
    }
    savedTimestamp = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    if((currentTimestamp - savedTimestamp) > 3000)
    {
      motordriver -> startTurning();
    }
    if(batterydriver -> readBatteryLevel() < LOW_BATTERY) 
    {
      speaker -> playEmptyBattery();
      changeState(FIND_PERIMETER);
    }
    int value, sign;
    canbus -> readPerimeter(&value, &sign);
    if (motordriver -> getRightCurrent() > 2 or canbus -> readPressure2() > 75 or value > 45)
    {
      changeState(TRY_LEFT);
    }
    if (motordriver -> getLeftCurrent() > 2 or canbus -> readPressure1() > 75)
    {
      changeState(TRY_RIGHT);
    }
    int distanceLL = canbus -> readDistanceSensor(LL);
    int distanceLM = canbus -> readDistanceSensor(LM);
    int distanceRM = canbus -> readDistanceSensor(RM);
    int distanceRR = canbus -> readDistanceSensor(RR);
    if (distanceLL < 100 or distanceLM < 100 or distanceRM < 100 or distanceRR < 100 or value > 30)
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
    checkForCharger();
    int value, sign;
    canbus -> readPerimeter(&value, &sign);
    long currentTimestamp = millis();

    if (motordriver -> getRightCurrent() > 1.6 or canbus -> readPressure2() > 75 or value > 45)
    {
      changeState(TRY_LEFT);
    }
    if (motordriver -> getLeftCurrent() > 1.6 or canbus -> readPressure1() > 75)
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
    savedTimestamp2 = millis();
    enter_state = false;
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnLeft(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }
    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(TRY_INSTEAD_RIGHT);
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
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnRight(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
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
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnLeft(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }
    Serial3.println(currentTimestamp - savedTimestamp2);
    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
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
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    int randomtime = random(500, 2000);
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnRight(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }

    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(TRY_BACKWARD);
    }
  }
}

void StateMachineImp::SM_TRY_BACKWARD(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_BACKWARD state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    int value, sign;
    canbus -> readPerimeter(&value, &sign);
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnRight(0.4, 180);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }
    if ((motordriver -> getRightCurrent() > 1.6 or canbus -> readPressure2() > 75 or value > 45) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(TRY_LAST_TIME_LEFT);
    }
    if ((motordriver -> getLeftCurrent() > 1.6 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(TRY_LAST_TIME_RIGHT);
    }
  }

}

void StateMachineImp::SM_TRY_LAST_TIME_LEFT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_LAST_TIME_LEFT state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnLeft(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }
    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(STUCK);
    }
  }
}

void StateMachineImp::SM_TRY_LAST_TIME_RIGHT(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter TRY_LAST_TIME_RIGHT state");
    enter_state = false;
    motordriver -> coastBrake();
    savedTimestamp = millis();
    savedTimestamp2 = millis();
  }
  else
  {
    checkForCharger();
    long currentTimestamp = millis();
    switch (internal_state)
    {
      case 0:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> driveBackward(0.4, 0.5);
        }
        break;
      case 1:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 2:
        if ( (currentTimestamp - savedTimestamp) > 800 ) {
          internal_state ++;
          motordriver -> takeTurnRight(0.4, 90);
        }
        break;
      case 3:
        if (motordriver -> distanceTargetReached())  internal_state ++;
        savedTimestamp = millis();
        break;
      case 4:
        if ( (currentTimestamp - savedTimestamp) > 800 ) internal_state ++;
        break;
      case 5:
        changeState(RUN);
        break;
    }
    if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 100 or canbus -> readPressure1() > 100) and (currentTimestamp - savedTimestamp2) > 1500)
    {
      changeState(STUCK);
    }
  }
}

void StateMachineImp::SM_STUCK(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter STUCK state");
    enter_state = false;
    motordriver -> bladeStop();
    motordriver -> coastBrake();
    speaker -> playStuck();
    savedTimestamp = millis();
  }
  else
  {
    long currentTimestamp = millis();
    if((currentTimestamp - savedTimestamp) > 1000)
    {
      motordriver -> disableVBlade();
    }
    checkForCharger();
  }
}

void StateMachineImp::SM_FIND_PERIMETER(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter FIND_PERIMETER state");
    enter_state = false;
    motordriver -> Forward(0.4);
  }
  else
  {
    if (!collision)
    {
      motordriver -> Forward(0.3);
      savedTimestamp = millis();
      if (motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75)
      {
        collision = true;
      }
      int value, sign;
      canbus -> readPerimeter(&value, &sign);
      if (value > (Setpoint - 10))
      {
        changeState(RETURN_HOME);
      }
    }
    else
    {
      long currentTimestamp = millis();
      int randomtime = random(500, 2000);
      checkForCharger();
      if ( (currentTimestamp - savedTimestamp) < 800 )
      {
        motordriver -> coastBrake();
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
        collision = false;
      }
      if ((motordriver -> getRightCurrent() > 2 or motordriver -> getLeftCurrent() > 2 or canbus -> readPressure2() > 75 or canbus -> readPressure1() > 75) and (currentTimestamp - savedTimestamp))
      {
        savedTimestamp = currentTimestamp;
      }

    }
  }
}

void StateMachineImp::SM_RANDOM_TURN(void)
{

}



void StateMachineImp::SM_RETURN_HOME(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter RETURN_HOME state");
    enter_state = false;
    savedTimestamp = millis();
  }
  else
  {
    int value, sign, PIDvalue;
    canbus -> readPerimeterPID(&value, &sign, &PIDvalue);
    long currentTimestamp = millis();
    Serial3.print(PIDvalue);
    Serial3.print(" : ");
    Serial3.print(Output);
    Serial3.print(" : "); 
    Serial3.println(Setpoint);
    Input = PIDvalue;
    perimeterPID -> Compute();
    motordriver -> rawLeft(3000 - Output);
    motordriver -> rawRight(1, Output);
    if ((motordriver -> getRightCurrent() > 2.0 or motordriver -> getLeftCurrent() > 2.0 ) and (currentTimestamp - savedTimestamp) > 1000)
    {
      canbus -> setState(8);
      changeState(STUCK);
    }
  }
}

void StateMachineImp::SM_SERROR(void)
{

}

void StateMachineImp::SM_CHARGING(void)
{
  if (enter_state == true)
  {
    Serial3.println("Enter CHARGING state");
    enter_state = false;
    motordriver -> bladeStop();
    motordriver -> dynamicBrake();
    canbus -> setChargingState(batterydriver -> readChargeState());
    canbus -> setChargingVoltage(batterydriver -> readVoltage());
    canbus -> setState(6);
    speaker -> playStartCharging();
    //batterydriver -> disableCharger();
    savedTimestamp = millis();
    savedTimestamp2 = millis();
  }
  else
  {
    long currentTimestamp = millis();
    if((currentTimestamp - savedTimestamp2) > 1000)
    {
      motordriver -> disableVBlade();
    }
    
    if ((currentTimestamp - savedTimestamp) > 500 )
    {
      canbus -> setChargingState(batterydriver -> readChargeState());
      canbus -> setChargingVoltage(batterydriver -> readVoltage());
      savedTimestamp = currentTimestamp;
    }
    if (!batterydriver -> isChargerPresent())
    {
      canbus -> setState(0);
      changeState(INIT);
    }
  }
}

void StateMachineImp::changeState(StateType newState)
{
  //printDiagnostics();
  //delay(500);
  SM_STATE = newState;
  enter_state = true;
  internal_state = 0;
}

void StateMachineImp::updateState(StateType newState)
{
  changeState(newState);
  switch (newState)
  {
    case RUN:
      canbus -> setState(5);
      break;
    case RETURN_HOME:
      canbus -> setState(7);
      break;
    case INIT:
      canbus -> setState(0);
      break;
  }
}

void StateMachineImp::printDiagnostics(void)
{
  int value, sign, PIDvalue;
  canbus -> readPerimeterPID(&value, &sign, &PIDvalue);
  Serial3.println("---------------------------------------------------------");
  Serial3.println("---                   Diagnostics                     ---");
  Serial3.println("---------------------------------------------------------");
  Serial3.println("Distancesensor LL: " + String(canbus -> readDistanceSensor(LL)));
  delay(10);
  Serial3.println("Distancesensor LM: " + String(canbus -> readDistanceSensor(LM)));
  delay(10);
  Serial3.println("Distancesensor B: " + String(canbus -> readDistanceSensor(B)));
  delay(10);
  Serial3.println("Distancesensor RM: " + String(canbus -> readDistanceSensor(RM)));
  delay(10);
  Serial3.println("Distancesensor RR: " + String(canbus -> readDistanceSensor(RR)));
  Serial3.println("Perimeter Magnitude: " + String(value));
  Serial3.println("Perimeter sign: " + String(sign));
  Serial3.println("Perimeter PID value: " + String(PIDvalue));
  Serial3.println("Pressure1: " + String(canbus -> readPressure1()));
  Serial3.println("Pressure2: " + String(canbus -> readPressure2()));
  Serial3.println("Status: " + String(canbus -> readStatus()));
  Serial3.println("Battery voltage: " + String(batterydriver -> readVoltage()));
  Serial3.println("Battery current: " + String(batterydriver -> readCurrent()));
  Serial3.println("Battery level: " + String(batterydriver -> readBatteryLevel()));
  Serial3.println("Left motor current: " + String(motordriver -> getLeftCurrent()));
  Serial3.println("Right motor current: " + String(motordriver -> getRightCurrent()));
  Serial3.println("Blade motor current: " + String(motordriver -> getBladeCurrent()));
  Serial3.println("Left motor speed: " + String(motordriver -> getLeftSpeed()));
  Serial3.println("Right motor speed: " + String(motordriver -> getRightSpeed()));
  Serial3.println("Current state: " + SM_STATE);
  Serial3.println("---------------------------------------------------------");
}

void StateMachineImp::printPIDValues(void)
{
  Serial3.print(pvalue);
  Serial3.print(":");
  Serial3.print(ivalue);
  Serial3.print(":");
  Serial3.print(dvalue);
  Serial3.print(":");
  Serial3.println(Setpoint);
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

void StateMachineImp::checkGlobalParameters(void)
{
  canbus -> setMasterState(SM_STATE);
  int controlPanelStatus  = canbus -> readStatus();
  if (controlPanelStatus == 0 and SM_STATE != INIT and SM_STATE != CHARGING) changeState(INIT);
  else if (controlPanelStatus == 7 and (SM_STATE != FIND_PERIMETER and SM_STATE != RETURN_HOME)) changeState(FIND_PERIMETER);
}

void StateMachineImp::changePValue(float value)
{
  pvalue = value;
  perimeterPID -> SetTunings(pvalue, ivalue, dvalue);
  setting -> writePIDValues(pvalue, ivalue, dvalue);
}

void StateMachineImp::changeIValue(float value)
{
  ivalue = value;
  perimeterPID -> SetTunings(pvalue, ivalue, dvalue);
  setting -> writePIDValues(pvalue, ivalue, dvalue);
}

void StateMachineImp::changeDValue(float value)
{
  dvalue = value;
  perimeterPID -> SetTunings(pvalue, ivalue, dvalue);
  setting -> writePIDValues(pvalue, ivalue, dvalue);
}

void StateMachineImp::changePIDSetpoint(float value)
{
  Setpoint = value;
  setting -> writePIDSetpoint(value);
}

void StateMachineImp::changeMowEnable(int value)
{
  mowEnable = value;
  setting -> writeMowEnable(value);
}
