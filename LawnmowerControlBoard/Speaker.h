#ifndef SPEAKER_H
#define SPEAKER_H

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include<Arduino.h>

class Speaker
{
  public:
    Speaker();
    void init(void);
    void playFile(const char *filename);
    void playTemp(void);
    void playStartMowing(void);
    void playPerimeterWireBroken(void);
    void playNoConnectionWithPerimeter(void);
    void playLostConnectionWithPerimeter(void);
    void playStuck(void);
    void playStartCharging(void);
    void playChargingComplete(void);

  private:
    AudioPlaySdWav playWav1;
    AudioOutputI2S audioOutput;
    AudioConnection patchCord1;
    AudioConnection patchCord2;
    AudioControlSGTL5000 sgtl5000_1;

};





#endif
