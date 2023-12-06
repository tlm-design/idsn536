/*
  CHIMEWebSocket
  Created for CHIME prototype
  Control electromagnet with ultrasonic sensor (distance sensor)
  This code reads websocket data and when a message is received, electromagnet is turned on & off and reversed direction
  by Tara Manuel tmanuel@usc.edu
*/

#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"

char ssid[] = SECRET_SSID;                     // network ID stored in arduino_secrets.h
char pass[] = SECRET_PASS;                     // network password
char serverAddress[] = SECRET_SERVER_ADDRESS;  // server address
int port = 80;                                 //3000;

String socketdata[3];
int counter = 0;
int lastIndex = 0;

WiFiClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;



//electromagnet
#define ENABLE 6  //greenwire 6
#define DIRA 3    //orangewire
#define DIRB 7    //yellowwire 7
int i;

void setup() {
//  Serial.begin(9600);
  stopChime();
  //sensor setup
 //electromagnet setup
  pinMode(ENABLE, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(DIRB, OUTPUT);

  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
  }

  IPAddress ip = WiFi.localIP();
}

void loop() {
  client.begin();
  while (client.connected()) {
    int messageSize = client.parseMessage();  // check if a message is available
    if (messageSize > 0) {
      ringChime();
    }
  }
  delay(100);
}

void ringChime() {
  //turn on/off electromagnet
  //update to set intensity based on threshold
  digitalWrite(ENABLE, HIGH);  // enable on
  for (i = 0; i < 5; i++) {
    digitalWrite(DIRA, HIGH);  //one way
    digitalWrite(DIRB, LOW);
    delay(80);
    digitalWrite(DIRA, LOW);  //reverse
    digitalWrite(DIRB, HIGH);
    delay(80);
  }
    digitalWrite(ENABLE,LOW); // disable
  delay(10);
}
void stopChime() {
  //turn on/off electromagnet
  digitalWrite(ENABLE, LOW);  // disable
}
