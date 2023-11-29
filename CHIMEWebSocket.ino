/*
  CHIMEWebSocket
  Created for CHIME prototype
  Control electromagnet with capacitive sensor
  by Tara Manuel tmanuel@usc.edu
*/

#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
#include <CapacitiveSensor.h>

char ssid[] = SECRET_SSID;    // network ID stored in arduino_secrets.h
char pass[] = SECRET_PASS;    // network password
char serverAddress[] = SECRET_SERVER_ADDRESS;   // server address
int port = 80;//3000;

String socketdata[3];
int counter = 0;
int lastIndex = 0;

WiFiClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;

//sensing
CapacitiveSensor capSensor = CapacitiveSensor(4, 2); // pin 4 sends electrical energy, pin 2 senses senses a change
int threshold = 1000;

//electromagnet 
#define ENABLE 6 //greenwire 6
#define DIRA 3 //orangewire
#define DIRB 7 //yellowwire 7
int i;

void setup() {
  Serial.begin(9600);
  
  //electromagnet set up
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);
  
  while ( status != WL_CONNECTED) {
    Serial.print("=> Attempting to connect to Network named: ");
    Serial.println(ssid);            // print the network name (SSID)
    status = WiFi.begin(ssid, pass);
  }

  Serial.print("=> SSID: ");         // print the SSID of the network
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("=> IP Address: ");   // print your Arduino's IP address
  Serial.println(ip);
}

void loop() {
  Serial.println("=> Starting WebSocket client");
  client.begin();
 
  while (client.connected()) {
     getSensorData();
    int messageSize = client.parseMessage();  // check if a message is available
    if (messageSize > 0) {
      String input = client.readString();
      for (int i = 0; i < input.length(); i++) {
        if (input.substring(i, i+1) == ",") {
          socketdata[counter] = input.substring(lastIndex, i);
          lastIndex = i + 1;
          counter++;
        }
        if (i == input.length() - 1) {
          socketdata[counter] = input.substring(lastIndex, i);
        }
      }
      Serial.print("Rcvd: ");    // confirm receipt of data with the computer
      input.trim();
      Serial.println(input);
      input = "";
      counter = 0;
      lastIndex = 0;
      if(socketdata[0]=="ON"){ ringChime();}
        else{stopChime();}
    }
  }

  Serial.println("=> Disconnected");
  delay(1000);
}
void getSensorData(){
  // store the value reported by the sensor in a variable
  long sensorValue = capSensor.capacitiveSensor(30);
  // if the value is greater than the threshold
  if (sensorValue > threshold) {
    client.beginMessage(TYPE_TEXT);
      client.print("ON");      // send data
      client.print(",");
      client.print("0");
      client.print(",");
      client.println(sensorValue);
      client.endMessage();
  }
  // if it's lower than the threshold
  else {
     client.beginMessage(TYPE_TEXT);
      client.print("OFF");      // send data
      client.print(",");
      client.print("0");
      client.print(",");
      client.println("0");
      client.endMessage();
  }
  delay(10);
}
void ringChime(){
  //turn on/off electromagnet
  //update to set intensity based on threshold
   digitalWrite(ENABLE,HIGH); // enable on
    for (i=0;i<5;i++) {
    digitalWrite(DIRA,HIGH); //one way
    digitalWrite(DIRB,LOW);
    delay(20);
    digitalWrite(DIRA,LOW);  //reverse
    digitalWrite(DIRB,HIGH);
    delay(20);
  }
}
void stopChime(){
  //turn on/off electromagnet
  digitalWrite(ENABLE,LOW); // disable
}
   