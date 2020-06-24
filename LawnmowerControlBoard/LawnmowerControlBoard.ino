#include "Battery.h"
#include "StateMachineImp.h"


StateMachineImp SM;
Battery battery;
String receivedText = "";


void setup() {
  //Serial3.begin(9600);
  //Serial3.print("AT+BAUD8");
  SM.initStatemachine();
  //battery.enableCharger();M
  Serial3.begin(115200);
  
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
    char temp = Serial3.read();
    Serial3.print(temp);
    //Serial3.println(receivedText);
    if (temp == 13)
    {
      Serial3.println();
      String commandText = receivedText.substring(0, 5);
      String dataText = receivedText.substring(6);
      if (commandText == "s")
      {
        SM.changeState(RUN);
      }
      else if (commandText == "h")
      {
        SM.changeState(INIT);
      }
      else if (commandText == "t")
      {
        SM.changeState(FIND_PERIMETER);
      }
      else if (commandText == "d")
      {
        SM.printDiagnostics();
      }
      else if (commandText == "PID")
      {
        SM.printPIDValues();
      }
      else if (commandText == "PID_P")
      {
        Serial3.print("Change PID P setting to ");
        Serial3.println(dataText);
        SM.changePValue(dataText.toFloat());
      }
      else if (commandText == "PID_I")
      {
        Serial3.print("Change PID I setting to ");
        Serial3.println(dataText);
        SM.changeIValue(dataText.toFloat());
      }
      else if (commandText == "PID_D")
      {
        Serial3.print("Change PID D setting to ");
        Serial3.println(dataText);
        SM.changeDValue(dataText.toFloat());
      }
      else if (commandText == "PID_T")
      {
        Serial3.print("Change PID Setpoint to ");
        Serial3.println(dataText);
        SM.changePIDSetpoint(dataText.toFloat());
      }
      receivedText = "";
    }
    else
    {
      receivedText += temp;
    }
  }
}
