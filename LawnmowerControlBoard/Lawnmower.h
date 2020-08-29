#ifndef LAWNMOWER_H
#define LAWNMOWER_H


#include<Arduino.h>
#include "StateMachineImp.h"

class Lawnmower
{
  public:
    Lawnmower();
    void checkBluetooth(void);
    void run(void);
    void init(void);
  private:
    StateMachineImp SM;
    String receivedText = "";
    void printHelpMenu(void);
};

#endif
