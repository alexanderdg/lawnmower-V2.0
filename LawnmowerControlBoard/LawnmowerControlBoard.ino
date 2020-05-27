#include "MotorDriver.h"
#include "CANbus.h"
#include "Battery.h"
#include "StateMachineImp.h"



CANbus canbus;
StateMachineImp SM;
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
  Serial3.begin(9600);
  canbus.initCAN();
  SM.initStatemachine();
  //battery.enableCharger();
  delay(500);
  
  //motioncontrol.enableVBlade();
  //motioncontrol.startTurning();
  //delay(5000);
  //motioncontrol.bladeStop();
 // motioncontrol.Forward(0.5);
  /*
  delay(500000);
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
  int value, sign;
  Serial.println("---------------------------------------------");
  Serial.println(canbus.readDistanceSensor(1));
  Serial.println(canbus.readDistanceSensor(2));
  Serial.println(canbus.readDistanceSensor(3));
  Serial.println(canbus.readDistanceSensor(4));
  Serial.println(canbus.readDistanceSensor(5));
  canbus.readPerimeter(& value, & sign);
  Serial.println(value);
  Serial.println("---------------------------------------------");
  delay(10);
  /*
  Can0.write(msg);
  CAN_message_t inMsg;
  while (Can0.available()) 
  {
    Can0.read(inMsg);
    Serial.print("CAN bus 0: "); hexDump(8, inMsg.buf);
  }
  */
  //Serial3.print("l current: ");
  //Serial3.println(motioncontrol.getLeftCurrent());
  //Serial3.print("r current: ");
  //Serial3.println(motioncontrol.getRightCurrent());
  //Serial3.print("Speed L: ");
  //Serial3.println(motioncontrol.getLeftSpeed());
  //Serial3.print("Speed R: ");
  //Serial3.println(motioncontrol.getRightSpeed());
  //Serial3.print("blade current: ");
  //Serial3.println(motioncontrol.getBladeCurrent());
  //battery.readVoltage();
  //Serial3.print("Battery voltage: ");
  //Serial3.println(battery.readVoltage());
  //Serial3.print("Battery current: ");
  //Serial3.println(battery.readCurrent());
  //Serial3.print("Battery status: ");
  //Serial3.println(battery.readBatteryLevel());
  //SM.RunStatemachine();
  //delay(100);
}

void serialEvent3() {
  while (Serial3.available()) {
    Serial3.println(Serial3.read());
  }
}
