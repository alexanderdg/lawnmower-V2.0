#include "CANbus.h"
#include<Arduino.h>
#include <FlexCAN.h>

static uint8_t hex[17] = "0123456789abcdef";

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}

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
  return true;
}

void CANbus::readPerimeter(int * value, int * sign) {
  * value = -1;
  * sign = -1;
  CAN_message_t inMsg;
  bool temp = readCANReg(8, & inMsg);
  if(temp)
  {
    if(inMsg.buf[2] != 1)
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
      data = (inMsg.buf[0] << 8 | inMsg.buf[1]) / 11.7;
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
