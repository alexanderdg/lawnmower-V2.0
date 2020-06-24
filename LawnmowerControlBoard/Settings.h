#ifndef SETTINGS_H
#define SETTINGS_H

#include <Arduino.h>

class Settings 
{

public:
    Settings(void);
    
    bool writePIDValues(float PValue, float IValue, float DValue);
    bool writePIDSetpoint(double value);
    bool readPIDValues(float * PValue, float * IValue, float * DValue);
    bool readPIDSetpoint(double * value);

private:


};

#endif
