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

bool CANbus::initCAN(void) {
  Can0.begin(250000);
  offsetPressure1 = readPressure1();
  offsetPressure2 = readPressure2();
  return true;
}

int CANbus::readPressure1(void) {
  int data = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(1, & inMsg);
  if (temp)
  {
    data = (inMsg.buf[0] << 4 | ((inMsg.buf[1] >> 4) & 0xFF));
  }
  return data - offsetPressure1;
}

int CANbus::readPressure2(void) {
  int data = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(2, & inMsg);
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
  bool temp = readCANReg(8, & inMsg);
  if (temp)
  {
    if (inMsg.buf[2] != 1)
    {
      *value = -2;
    }
    else
    {
      * value = inMsg.buf[0];
    }
  }
}

int CANbus::readDistanceSensor(int sensor) {
  int data = -1;
  CAN_message_t inMsg;
  if (sensor > 0 && sensor < 6)
  {
    bool temp = readCANReg(sensor + 2, & inMsg);
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

bool CANbus::readCANReg(int reg, CAN_message_t * inMsg) {

  bool ok = false;
  msg.buf[0] = reg;
  Can0.write(msg);
  delayMicroseconds(800);
  while (Can0.available())
  {
    Can0.read( * inMsg);
    ok = true;
  }
  delayMicroseconds(800);
  return ok;
}
