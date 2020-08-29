#include <WiFi.h>
#include <Wire.h>
#include "SW_MCP4017.h"

// Replace with your network credentials
const char* ssid = "telenet-2594037";
const char* password = "j76kdjjpkCdx";

// Set web server port number to 80
WiFiServer server(80);

uint8_t dpMaxSteps = 128; //remember even thought the the digital pot has 128 steps it looses one on either end (usually cant go all the way to last tick)
int maxRangeOhms = 10000; //this is a 5K potentiometer
MCP4017 i2cDP(0x2F, dpMaxSteps, maxRangeOhms);

int led = 2;
int ps = 32;
int we = 12;
int ce = 23;
// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";



// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0;
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  // initialize the digital pin as an output.
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(ps, OUTPUT);
  pinMode(we, OUTPUT);
  pinMode(ce, OUTPUT);
  digitalWrite(ps, HIGH);
  digitalWrite(we, HIGH);
  digitalWrite(ce, LOW);
  Serial.begin(115200);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  i2cDP.setSteps(23);
}

void loop() {
  if (Serial2.available() > 0) {
    char temp = Serial2.read();
    Serial.print(temp);
  }
}



/*
void serialEvent2() {
  
}
*/
