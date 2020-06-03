#include<Arduino.h>
#include "Speaker.h"

#define SDCARD_CS_PIN    BUILTIN_SDCARD
#define SDCARD_MOSI_PIN  100  
#define SDCARD_SCK_PIN   110 

Speaker::Speaker(void) : patchCord1(playWav1, 0, audioOutput, 0), patchCord2(playWav1, 1, audioOutput, 1)
{
  
}

void Speaker::init(void) 
{
  AudioMemory(8);
  sgtl5000_1.enable();
  sgtl5000_1.volume(0.5);
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  SD.begin(SDCARD_CS_PIN);
}

bool Speaker::selfTest(void)
{
  return true;
}

void Speaker::playFile(const char *filename)
{
  Serial.print("Playing file: ");
  Serial.println(filename);
  playWav1.play(filename);
  delay(5);
  while (playWav1.isPlaying()) {
    
  }
}

void Speaker::playTemp(void)
{
  playFile("STARTMOW.WAV"); 
}

void Speaker::playStartMowing(void)
{
  playFile("STARTMOW.WAV"); 
}

void Speaker::playPerimeterWireBroken(void)
{
  playFile("BROKPERI.WAV");
}

void Speaker::playNoConnectionWithPerimeter(void)
{
  playFile("NOCONPER.WAV");
}

void Speaker::playLostConnectionWithPerimeter(void)
{
  playFile("CONPERIL.WAV");
}

void Speaker::playStuck(void)
{
  playFile("STUCK.WAV");
}

void Speaker::playStartCharging(void)
{
  playFile("SCHRG.WAV");
}

void Speaker::playChargingComplete(void)
{
  playFile("CHRGC.WAV");
}
