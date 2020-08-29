#include<Arduino.h>
#include "Lawnmower.h"

Lawnmower mower;

void setup() {
  mower.init();
}

void loop() {
  mower.run();
  delay(10);
}

void serialEvent3() {
  mower.checkBluetooth();
}
