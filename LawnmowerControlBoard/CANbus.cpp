#include "CANbus.h"
#include<Arduino.h>
#include <FlexCAN.h>

static uint8_t hex[17] = "0123456789abcdef";

static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working>>4 ] );
    Serial.write( hex[ working&15 ] );
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
  msg.len = 8;
  msg.buf[0] = 10;
  msg.buf[1] = 20;
  msg.buf[2] = 0;
  msg.buf[3] = 100;
  msg.buf[4] = 128;
  msg.buf[5] = 64;
  msg.buf[6] = 32;
  msg.buf[7] = 16;
}

bool CANbus::initCAN(void) {
  Can0.begin(250000);
  return true;
}

int CANbus::readPot(void) {
  int result = Can0.write(msg);
  CAN_message_t inMsg;
  while (Can0.available()) 
  {
    Can0.read(inMsg);
    Serial.print("CAN bus 0: "); hexDump(8, inMsg.buf);
  }
  return result;
}
