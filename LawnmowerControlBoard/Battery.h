#ifndef BATTERY_H
#define BATTERY_H

#include<Arduino.h>
class Battery {
  public:
    Battery();
    bool initBattery(void);
    float readVoltage(void);
    float readCurrent(void);
    float readPower(void);
    int readBatteryLevel(void);

    void enableCharger(void);
    void disableCharger(void);


  private:
    bool writeI2CByte(int reg, int MSBvalue, int LSBvalue);
    bool readI2CByte(int reg, int * MSBvalue, int * LSBvalue);

    int adress = 0x45;
    
    int CHRG_EN = 33;

};

#endif
