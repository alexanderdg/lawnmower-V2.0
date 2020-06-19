#ifndef CANBUS_H
#define CANBUS_H

#include<Arduino.h>
#include <FlexCAN.h>

static CAN_message_t msg;

enum DistanceSensor {
  LL,
  LM,
  B,
  RM,
  RR,
};

class CANbus
{
  public:
    CANbus();
    bool initCAN(void);

    int readPressure1(void);
    int readPressure2(void);
    void readPerimeter(int * value, int * sign);
    int readDistanceSensor(DistanceSensor sensor);

    bool selfTest(void);

  private:
    //pin definitions used for the CAN receiver/transceiver
    int CANTX = 3;
    int CANRX = 4;
    int offsetPressure1 = 0;
    int offsetPressure2 = 0;
    bool readCANReg(int reg, CAN_message_t * inMsg);

};


#endif
