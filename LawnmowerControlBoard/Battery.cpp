#include "Battery.h"
#include<Arduino.h>
#include <Wire.h>


//define the impedance of the battery in milliOhms
//to simplify the calculations, a constant impedance is used
#define impedance 65

Battery::Battery(void) {
  pinMode(CHRG_EN, OUTPUT);
  pinMode(CHRG_PRESENT, INPUT);
  pinMode(CHARGE, INPUT);
  pinMode(FAULT, INPUT);
  pinMode(READY, INPUT);
  pinMode(TOC, INPUT);
  Wire.begin();
  initBattery();
}

bool Battery::selfTest(void) {
  bool temp = false;
  if(digitalRead(FAULT) == 1) 
  {
    if(readVoltage() > 9) temp = true;
  }
  return temp; 
}

bool Battery::initBattery(void) {
  //RST = 1: Activate reset
  //BRNG = 1: FSR of busvoltage at 32V
  //PGA = 0b00: set gain to max
  //BADC 4-2 = 0b111: set Averaging samples to max 128 for the Bus voltage
  int MSBdata = 0xA7;
  //BADC 1 = 1: set Averaging samples to max 128
  //SADC 4-1 = 0b1111: set Averaging samples to max 128 for the shunt voltage
  // Mode 3-1 = 0b111: Set ADC in continous mode for the Shunt and bus voltage
  int LSBdata = 0xFF;
  writeI2CByte(0x00, MSBdata, LSBdata);
  //Program the calibration register with 6826
  MSBdata = 0x6A;
  LSBdata = 0xAA;
  writeI2CByte(0x05, MSBdata, LSBdata);
  return true;
}

void Battery::enableCharger(void) {
  digitalWrite(CHRG_EN, LOW);
}

void Battery::disableCharger(void) {
  digitalWrite(CHRG_EN, HIGH);
}

bool Battery::isChargerPresent(void) {
  return not(digitalRead(CHRG_PRESENT));
}

ChargeState Battery::readChargeState(void)
{
  ChargeState status = UNDEFINED;
  if (digitalRead(CHRG_PRESENT) == 1)
  {
    status = NO_CHARGER_PRESENT;
  }
  else
  {
    if (digitalRead(CHRG_EN) == 0)
    {
      if (digitalRead(FAULT) == 0)
      {
        status = FAULT;
      }
      else if (digitalRead(CHARGE) == 0 && digitalRead(READY) == 0 && digitalRead(TOC) == 1)
      {
        status = FAST_CHARGE;
      }
      else if (digitalRead(CHARGE) == 0 && digitalRead(READY) == 0 && digitalRead(TOC) == 0)
      {
        status = TOP_OFF_CHARGE;
      }
      else if (digitalRead(CHARGE) == 1 && digitalRead(READY) == 0 && digitalRead(TOC) == 1)
      {
        status = CHARGE_COMPLETE;
      }
    }
    else
    {
      status = CHARGER_DISABLED;
    }
  }
  return status;
}

String Battery::enumToString(ChargeState state)
{
  String temp = "";
  ChargeState status = state;
  switch (state)
  {
    case ChargeState::NO_CHARGER_PRESENT:
      temp = "No charger detected at the main board";
      break;
    case ChargeState::FAST_CHARGE:
      temp = "Battery is fast charged by the charger";
      break;
    case ChargeState::TOP_OFF_CHARGE:
      temp = "Battery is top off charged by the charger";
      break;
    case ChargeState::CHARGE_COMPLETE:
      temp = "Battery is fully charged by the charger";
      break;
    case ChargeState::CHARGER_DISABLED:
      temp = "Charger is disabled by the main board";
      break;
    case ChargeState::FAULT:
      temp = "Fault is detected with the charger";
      break;
    case ChargeState::UNDEFINED:
      temp = "Charger is in an undefined state";
      break;
  }
  return temp;
}

float Battery::readPower(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x03, & MSBdata, & LSBdata);
  int temp = (MSBdata << 8) + LSBdata;
  float normalized = temp * 0.03;
  return normalized;
}

float Battery::readCurrent(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x04, & MSBdata, & LSBdata);
  int temp = (MSBdata << 8) + LSBdata;
  float normalized = temp * 0.0015;
  return normalized;
}

float Battery::readVoltage(void) {
  int MSBdata, LSBdata;
  readI2CByte(0x02, & MSBdata, & LSBdata);
  int temp = (MSBdata << 5) + ((LSBdata & 0b11111000) >> 3);
  float normalized = temp * 0.004;
  return normalized;
}

float Battery::readOpenVoltage(void) {
  float voltage = readVoltage();
  float current = readCurrent();
  float openVoltage = voltage + (current * (impedance / 1000.0));
  return openVoltage;
}

int Battery::readBatteryLevel(void) {
  float voltage = readVoltage();
  float current = readCurrent();
  float openVoltage = voltage + (current * (impedance / 1000.0));
  //Serial.println(current * (impedance/1000.0));
  int foundIndex = -1;
  for (int i = 0; i < 15; i ++)
  {
    if (openVoltage > dischargeGraph[i])
    {
      foundIndex = i;
      break;
    }
  }
  return int(100 - (foundIndex * 6.66));
}

bool Battery::writeI2CByte(int reg, int MSBvalue, int LSBvalue) {
  Wire.beginTransmission(adress);
  Wire.write(reg);
  Wire.write(MSBvalue);
  Wire.write(LSBvalue);
  Wire.endTransmission();
  return true;
}

bool Battery::readI2CByte(int reg, int * MSBvalue, int * LSBvalue) {
  Wire.beginTransmission(adress);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(adress, 2);
  byte buff[2];
  Wire.readBytes(buff, 2);
  * MSBvalue = buff[0];
  * LSBvalue = buff[1];
  return true;
}
