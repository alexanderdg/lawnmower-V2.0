#ifndef CANBUS_H
#define CANBUS_H

#include<Arduino.h>
#include <FlexCAN.h>

static CAN_message_t msg;
static CAN_message_t inMsg;

class CANbus
{
  public:
    CANbus();
    bool initCAN(void);
    
    void readPerimeter(int * value, int * sign);
    int readDistanceSensor(int sensor);

  private:
    //pin definitions used for the CAN receiver/transceiver
    int CANTX = 3;
    int CANRX = 4;

    bool readCANReg(int reg, CAN_message_t * inMsg);

};


#endif
