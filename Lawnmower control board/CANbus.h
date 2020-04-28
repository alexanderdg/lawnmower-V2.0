#ifndef CANBUS_H
#define CANBUS_H

#include<Arduino.h>
#include <FlexCAN.h>

static CAN_message_t msg;

class CANbus
{
  public:
    CANbus();
    bool initCAN(void);
    int readPot(void);

  private:
    //pin definitions used for the CAN receiver/transceiver
    int CANTX = 3;
    int CANRX = 4;

};


#endif
