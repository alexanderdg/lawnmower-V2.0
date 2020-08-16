/*
#include <WiFi.h>
#include <Wire.h>
#include "SW_MCP4017.h"

// Replace with your network credentials
const char* ssid = "telenet-2594037";
const char* password = "";

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
  pinMode(led, OUTPUT);
  pinMode(ps, OUTPUT);
  pinMode(we, OUTPUT);
  pinMode(ce, OUTPUT);
  digitalWrite(ps, HIGH);
  digitalWrite(we, HIGH);
  digitalWrite(ce, HIGH);
  Serial.begin(115200);
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  WiFiClient client = server.available();   // Listen for incoming clients
  if (client) {                             // If a new client connects,
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected() && currentTime - previousTime <= timeoutTime) {  // loop while the client's connected
      currentTime = millis();
      //Serial.println("test");
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(led, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(led, LOW);
            } else if (header.indexOf("GET /27/on") >= 0) {
              Serial.println("GPIO 27 on");
              output27State = "on";
              digitalWrite(we, HIGH);
            } else if (header.indexOf("GET /27/off") >= 0) {
              Serial.println("GPIO 27 off");
              output27State = "off";
              digitalWrite(we, LOW);
            }

            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");

            // Web Page Heading
            client.println("<body><h1>Robot lawnmower</h1>");

            // Display current state, and ON/OFF buttons for GPIO 26
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button
            if (output26State == "off") {
              client.print("<p><a href=\"/26/on\"><button class=\"button\">ON");
              client.print(millis());
              client.println("</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
              client.println("<INPUT TYPE=SUBMIT NAME='submit' VALUE='Change Pin 5!'>");
            }

            // Display current state, and ON/OFF buttons for GPIO 27
            client.println("<p>GPIO 27 - State " + output27State + "</p>");
            // If the output27State is off, it displays the ON button
            if (output27State == "off") {
              client.println("<p><a href=\"/27/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/27/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");

            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}
*/






















#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <DHT.h>

#define ssid      "xxx"       // WiFi SSID
#define password  "xxxxxxxx"  // WiFi password
#define DHTTYPE   DHT22       // DHT type (DHT11, DHT22)
#define DHTPIN    D4          // Broche du DHT / DHT Pin
const uint8_t GPIOPIN[4] = {D5,D6,D7,D8};  // Led
float   t = 0 ;
float   h = 0 ;
float   p = 0;
String  etatGpio[4] = {"OFF","OFF","OFF","OFF"};

// Création des objets / create Objects
DHT dht(DHTPIN, DHTTYPE);
Adafruit_BMP085 bmp;
ESP8266WebServer server ( 80 );

String getPage(){
  String page = "<html lang='fr'><head><meta http-equiv='refresh' content='60' name='viewport' content='width=device-width, initial-scale=1'/>";
  page += "<link rel='stylesheet' href='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/css/bootstrap.min.css'><script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.1/jquery.min.js'></script><script src='https://maxcdn.bootstrapcdn.com/bootstrap/3.3.7/js/bootstrap.min.js'></script>";
  page += "<title>ESP8266 Demo - www.projetsdiy.fr</title></head><body>";
  page += "<div class='container-fluid'>";
  page +=   "<div class='row'>";
  page +=     "<div class='col-md-12'>";
  page +=       "<h1>Demo Webserver ESP8266 + Bootstrap</h1>";
  page +=       "<h3>Mini station m&eacute;t&eacute;o</h3>";
  page +=       "<ul class='nav nav-pills'>";
  page +=         "<li class='active'>";
  page +=           "<a href='#'> <span class='badge pull-right'>";
  page +=           t;
  page +=           "</span> Temp&eacute;rature</a>";
  page +=         "</li><li>";
  page +=           "<a href='#'> <span class='badge pull-right'>";
  page +=           h;
  page +=           "</span> Humidit&eacute;</a>";
  page +=         "</li><li>";
  page +=           "<a href='#'> <span class='badge pull-right'>";
  page +=           p;
  page +=           "</span> Pression atmosph&eacute;rique</a></li>";
  page +=       "</ul>";
  page +=       "<table class='table'>";  // Tableau des relevés
  page +=         "<thead><tr><th>Capteur</th><th>Mesure</th><th>Valeur</th><th>Valeur pr&eacute;c&eacute;dente</th></tr></thead>"; //Entête
  page +=         "<tbody>";  // Contenu du tableau
  page +=           "<tr><td>DHT22</td><td>Temp&eacute;rature</td><td>"; // Première ligne : température
  page +=             t;
  page +=             "&deg;C</td><td>";
  page +=             "-</td></tr>";
  page +=           "<tr class='active'><td>DHT22</td><td>Humidit&eacute;</td><td>"; // 2nd ligne : Humidité
  page +=             h;
  page +=             "%</td><td>";
  page +=             "-</td></tr>";
  page +=           "<tr><td>BMP180</td><td>Pression atmosph&eacute;rique</td><td>"; // 3ème ligne : PA (BMP180)
  page +=             p;
  page +=             "mbar</td><td>";
  page +=             "-</td></tr>";
  page +=       "</tbody></table>";
  page +=       "<h3>GPIO</h3>";
  page +=       "<div class='row'>";
  page +=         "<div class='col-md-4'><h4 class ='text-left'>D5 ";
  page +=           "<span class='badge'>";
  page +=           etatGpio[0];
  page +=         "</span></h4></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D5' value='1' class='btn btn-success btn-lg'>ON</button></form></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D5' value='0' class='btn btn-danger btn-lg'>OFF</button></form></div>";
  page +=         "<div class='col-md-4'><h4 class ='text-left'>D6 ";
  page +=           "<span class='badge'>";
  page +=           etatGpio[1];
  page +=         "</span></h4></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D6' value='1' class='btn btn-success btn-lg'>ON</button></form></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D6' value='0' class='btn btn-danger btn-lg'>OFF</button></form></div>";
  page +=         "<div class='col-md-4'><h4 class ='text-left'>D7 ";
  page +=           "<span class='badge'>";
  page +=           etatGpio[2];
  page +=         "</span></h4></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D7' value='1' class='btn btn-success btn-lg'>ON</button></form></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D7' value='0' class='btn btn-danger btn-lg'>OFF</button></form></div>";
  page +=         "<div class='col-md-4'><h4 class ='text-left'>D8 ";
  page +=           "<span class='badge'>";
  page +=           etatGpio[3];
  page +=         "</span></h4></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D8' value='1' class='btn btn-success btn-lg'>ON</button></form></div>";
  page +=         "<div class='col-md-4'><form action='/' method='POST'><button type='button submit' name='D8' value='0' class='btn btn-danger btn-lg'>OFF</button></form></div>";
  page +=       "</div>";
  page +=     "<br><p><a href='http://www.projetsdiy.fr'>www.projetsdiy.fr</p>
  page += "</div></div></div>";
  page += "</body></html>";
  return page;
}
void handleRoot(){ 
  if ( server.hasArg("D5") ) {
    handleD5();
  } else if ( server.hasArg("D6") ) {
    handleD6();
  } else if ( server.hasArg("D7") ) {
    handleD7();
  } else if ( server.hasArg("D8") ) {
    handleD8();
  } else {
    server.send ( 200, "text/html", getPage() );
  }  
}

void handleD5() {
  String D5Value; 
  updateGPIO(0,server.arg("D5")); 
}

void handleD6() {
  String D6Value; 
  updateGPIO(1,server.arg("D6")); 
}

void handleD7() {
  String D7Value; 
  updateGPIO(2,server.arg("D7")); 
}

void handleD8() {
  String D8Value; 
  updateGPIO(3,server.arg("D8")); 
}

void updateGPIO(int gpio, String DxValue) {
  Serial.println("");
  Serial.println("Update GPIO "); Serial.print(GPIOPIN[gpio]); Serial.print(" -> "); Serial.println(DxValue);
  
  if ( DxValue == "1" ) {
    digitalWrite(GPIOPIN[gpio], HIGH);
    etatGpio[gpio] = "On";
    server.send ( 200, "text/html", getPage() );
  } else if ( DxValue == "0" ) {
    digitalWrite(GPIOPIN[gpio], LOW);
    etatGpio[gpio] = "Off";
    server.send ( 200, "text/html", getPage() );
  } else {
    Serial.println("Err Led Value");
  }  
}

void setup() {
  for ( int x = 0 ; x < 5 ; x++ ) { 
    pinMode(GPIOPIN[x],OUTPUT);
  }  
  Serial.begin ( 115200 );
  // Initialisation du BMP180 / Init BMP180
  
  WiFi.begin ( ssid, password );
  // Attente de la connexion au réseau WiFi / Wait for connection
  while ( WiFi.status() != WL_CONNECTED ) {
    delay ( 500 ); Serial.print ( "." );
  }
  // Connexion WiFi établie / WiFi connexion is OK
  Serial.println ( "" ); 
  Serial.print ( "Connected to " ); Serial.println ( ssid );
  Serial.print ( "IP address: " ); Serial.println ( WiFi.localIP() );

  // On branche la fonction qui gère la premiere page / link to the function that manage launch page 
  server.on ( "/", handleRoot );

  server.begin();
  Serial.println ( "HTTP server started" );
  
}

void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
  t = 25
  h = 100
  p = 101;

  delay(1000);
}
