#ifndef BATTERY_H
#define BATTERY_H

#include<Arduino.h>

enum ChargeState
{
  NO_CHARGER_PRESENT,
  FAST_CHARGE,
  TOP_OFF_CHARGE,
  CHARGE_COMPLETE,
  CHARGER_DISABLED,
  FAULT,
  UNDEFINED
};

class Battery {
  public:
    Battery();
    bool initBattery(void);
    float readVoltage(void);
    float readOpenVoltage(void);
    float readCurrent(void);
    float readPower(void);
    int readBatteryLevel(void);
    bool isChargerPresent(void);

    void enableCharger(void);
    void disableCharger(void);
    ChargeState readChargeState(void);
    String enumToString(ChargeState state);


  private:
    bool writeI2CByte(int reg, int MSBvalue, int LSBvalue);
    bool readI2CByte(int reg, int * MSBvalue, int * LSBvalue);

    int adress = 0x45;
    
    int CHRG_EN = 33;
    int CHRG_PRESENT = 38;
    int CHARGE = 34; 
    int FAULT = 35;
    int READY = 36;
    int TOC = 37;

    float dischargeGraph[15] = {14.55, 13.25, 13.05, 12.9, 12.8, 12.75, 12.7, 12.6, 12.55, 12.45, 12.4, 12.3, 12.2, 11.95, 11.55};

};

#endif
