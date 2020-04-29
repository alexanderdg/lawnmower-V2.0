#include "MotorDriver.h"
#include "CANbus.h"
#include "Battery.h"


int L_CURRENT = 21;
int R_CURRENT = 32; 

MotorDriver motioncontrol;
CANbus canbus;
Battery battery;
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

void setup() {
  canbus.initCAN();
  battery.enableCharger();
  delay(5000);
  
  //motioncontrol.enableVBlade();
  motioncontrol.startTurning();
  //delay(5000);
  //motioncontrol.bladeStop();
  //motioncontrol.Forward(0.5);
  /*
  delay(5000);
  motioncontrol.coastBrake();
  delay(5000);
  motioncontrol.Backward(0.5);
  delay(5000);
  motioncontrol.dynamicBrake();
  delay(5000);
  motioncontrol.turnLeft(0.5);
  */
}

void loop() {
  canbus.readPot();
  /*
  Can0.write(msg);
  CAN_message_t inMsg;
  while (Can0.available()) 
  {
    Can0.read(inMsg);
    Serial.print("CAN bus 0: "); hexDump(8, inMsg.buf);
  }
  */
  Serial.print("l current: ");
  Serial.println(motioncontrol.getLeftCurrent());
  Serial.print("r current: ");
  Serial.println(motioncontrol.getRightCurrent());
  Serial.print("Speed L: ");
  Serial.println(motioncontrol.getLeftSpeed());
  Serial.print("Speed R: ");
  Serial.println(motioncontrol.getRightSpeed());
  Serial.print("blade current: ");
  Serial.println(motioncontrol.getBladeCurrent());
  //battery.readVoltage();
  Serial.print("Battery voltage: ");
  Serial.println(battery.readVoltage());
  Serial.print("Battery current: ");
  Serial.println(battery.readCurrent());
  Serial.print("Battery status: ");
  Serial.println(battery.readBatteryLevel());
  delay(100);
}
