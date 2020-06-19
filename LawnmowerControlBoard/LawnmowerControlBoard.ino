#include "Battery.h"
#include "StateMachineImp.h"


StateMachineImp SM;
Battery battery;



void setup() {
  //Serial3.begin(9600);
  //Serial3.print("AT+BAUD8");
  SM.initStatemachine();
  //battery.enableCharger();
  Serial3.begin(9600);
  
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
  /*
  int value, sign;
  Serial.println("---------------------------------------------");
  Serial.println(canbus.readDistanceSensor(1));
  Serial.println(canbus.readDistanceSensor(2));
  Serial.println(canbus.readDistanceSensor(3));
  Serial.println(canbus.readDistanceSensor(4));
  Serial.println(canbus.readDistanceSensor(5));
  canbus.readPerimeter(& value, & sign);
  Serial.println(value);
  Serial.println(canbus.readPressure1());
  Serial.println(canbus.readPressure2());
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
  //Serial3.println("Run statemachine :) ");
  SM.RunStatemachine();
  delay(10);
}


void serialEvent3() {
  if (Serial3.available() > 0) {
    byte temp = Serial3.read();
    //Serial3.println(temp);
    if(temp == 's')
    {
      SM.changeState(RUN);
    }
    else if(temp == 'h')
    {
      SM.changeState(INIT);
    }
    else if(temp == 't')
    {
      SM.changeState(FIND_PERIMETER);
    }
  }
}
