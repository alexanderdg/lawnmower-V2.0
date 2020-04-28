#include "Battery.h"
#include<Arduino.h>
#include <Wire.h>

Battery::Battery(void) {
  pinMode(CHRG_EN, OUTPUT);
  Wire.begin();
  initBattery();
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
}

void Battery::enableCharger(void) {
  digitalWrite(CHRG_EN, LOW);
}

void Battery::disableCharger(void) {
  digitalWrite(CHRG_EN, HIGH);
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
