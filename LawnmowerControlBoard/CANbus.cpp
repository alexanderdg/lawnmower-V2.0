#include "CANbus.h"
#include<Arduino.h>
#include <FlexCAN.h>



CANbus::CANbus() {
  //pinMode(CANTX, OUTPUT);
  //pinMode(CANRX, INPUT);
  //pinMode(3, OUTPUT);
  //digitalWrite(3, LOW);
  //digitalWrite(CANTX, HIGH);

  msg.ext = 0;
  msg.id = 0x01;
  msg.len = 1;
  msg.buf[0] = 0;
}

bool CANbus::selfTest(void) {
  bool temp = false;
  int value, sign;
  readPerimeter(&value, &sign);
  if (readPressure1() > 0 && readPressure2() > 0
      && readDistanceSensor(1) > 0 && readDistanceSensor(2) > 0
      && readDistanceSensor(3) > 0 && readDistanceSensor(4) > 0
      && readDistanceSensor(5) > 0 && value > 0) temp = true;
  return temp;
}

bool CANbus::initCAN(void) {
  Can0.begin(250000);
  offsetPressure1 = readPressure1();
  offsetPressure2 = readPressure2();
  return true;
}

int CANbus::readPressure1(void) {
  int data = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(1, 1, & inMsg);
  if (temp)
  {
    data = (inMsg.buf[0] << 4 | ((inMsg.buf[1] >> 4) & 0xFF));
  }
  return data - offsetPressure1;
}

int CANbus::readPressure2(void) {
  int data = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(1, 2, & inMsg);
  if (temp)
  {
    data = (inMsg.buf[0] << 4 | ((inMsg.buf[1] >> 4) & 0xFF));
  }
  return data - offsetPressure2;
}

void CANbus::readPerimeter(int * value, int * sign) {
  * value = -1;
  * sign = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(1, 8, & inMsg);
  if (temp)
  {
    if (inMsg.buf[2] != 1)
    {
      * value = -2;
      * sign = -2;
    }
    else
    {
      * value = inMsg.buf[0];
      * sign = inMsg.buf[1];
    }
  }
}

void CANbus::readPerimeterPID(int *value, int *sign, int *PIDvalue) {
  int returnvalue = 0;
  int lvalue, lsign;
  readPerimeter(&lvalue, &lsign);
  if (lsign == 0)
  {
    returnvalue = 500;
  }
  else if (lsign == 1)
  {
    returnvalue = lvalue;
  }
  * PIDvalue = returnvalue;
  * value = lvalue;
  * sign = lsign;
}

int CANbus::readDistanceSensor(DistanceSensor sensor) {
  int data = -1;
  CAN_message_t inMsg;
  if (sensor >= 0 && sensor < 5)
  {
    bool temp = readCANReg(1, sensor + 3, & inMsg);
    if (temp)
    {
      if (inMsg.buf[2] != 1)
      {
        data = -2;
      }
      else
      {
        data = (inMsg.buf[0] << 8 | inMsg.buf[1]) / 11.7;
      }
    }
  }
  return data;
}

int CANbus::readStatus(void) {
  int data = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(2, 0, & inMsg);
  if (temp) data = inMsg.buf[0];
  return data;
}

bool CANbus::readCANReg(int device, int reg, CAN_message_t * inMsg) {

  bool ok = false;
  msg.id = device;
  msg.buf[0] = reg;
  Can0.write(msg);
  delayMicroseconds(800);
  while (Can0.available())
  {
    Can0.read( * inMsg);
    ok = true;
  }
  delayMicroseconds(100);
  return ok;
}
