#include "Settings.h"
#include <EEPROM.h>

#define PID_P_VALUE_A 0
#define PID_I_VALUE_A 1
#define PID_D_VALUE_A 2

Settings::Settings(void) {

}


bool Settings::writePIDValues(float PValue, float IValue, float DValue)
{
  float temp = 0.0;
  EEPROM.get(0, temp);
  if(temp != PValue) EEPROM.put(0, PValue);
  EEPROM.get(4, temp);
  if(temp != IValue) EEPROM.put(4, IValue);
  EEPROM.get(8, temp);
  if(temp != DValue) EEPROM.put(8, DValue);
  return true;
}

bool Settings::writePIDSetpoint(double value)
{
  float temp = 0.0;
  EEPROM.get(12, temp);
  if(temp != value) EEPROM.put(12, value);
}

bool Settings::writeMowEnable(int enable)
{
  int temp = 0;
  EEPROM.get(16, temp);
  if(temp != enable) EEPROM.put(16, enable);
}

bool Settings::readPIDValues(float * PValue, float * IValue, float * DValue)
{
  EEPROM.get(0, * PValue);
  EEPROM.get(4, * IValue);
  EEPROM.get(8, * DValue);
  return true;
}

bool Settings::readPIDSetpoint(double * value)
{
  EEPROM.get(12, * value);
  return true;
}

bool Settings::readMowEnable(int * enable)
{
  EEPROM.get(16, * enable);
  return true;
}
