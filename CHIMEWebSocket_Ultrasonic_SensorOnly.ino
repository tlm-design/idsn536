/*
  CHIMEWebSocket
  Created for CHIME prototype
  Control electromagnet with ultrasonic sensor (distance sensor)
  This code sends a message via websocket with sensor data
  by Tara Manuel tmanuel@usc.edu
*/

#include <ArduinoHttpClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"
//#include <CapacitiveSensor.h>

char ssid[] = SECRET_SSID;                     // network ID stored in arduino_secrets.h
char pass[] = SECRET_PASS;                     // network password
char serverAddress[] = SECRET_SERVER_ADDRESS;  // server address
int port = 80;                                 //3000;

// ULTRASONIC SENSOR
int TRIG = 4;
int ECHO = 2;
int DURATION;
int DISTANCE;

String socketdata[3];
int counter = 0;
int lastIndex = 0;

WiFiClient wifi;
WebSocketClient client = WebSocketClient(wifi, serverAddress, port);
int status = WL_IDLE_STATUS;


void setup() {
  Serial.begin(9600);
 // ULTRASONIC SENSOR
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  while (status != WL_CONNECTED) {
    Serial.print("=> Attempting to connect to Network named: ");
    Serial.println(ssid);  // print the network name (SSID)
    status = WiFi.begin(ssid, pass);
  }

  Serial.print("=> SSID: ");  // print the SSID of the network
  Serial.println(WiFi.SSID());
  IPAddress ip = WiFi.localIP();
  Serial.print("=> IP Address: ");  // print your Arduino's IP address
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
        if (input.substring(i, i + 1) == ",") {
          socketdata[counter] = input.substring(lastIndex, i);
          lastIndex = i + 1;
          counter++;
        }
        if (i == input.length() - 1) {
          socketdata[counter] = input.substring(lastIndex, i);
        }
      }
      Serial.print("Rcvd: ");  // confirm receipt of data with the computer
      input.trim();
      Serial.println(input);
      input = "";
      counter = 0;
      lastIndex = 0;
    }
  }

  Serial.println("=> Disconnected");
  delay(100);
}
void getSensorData() {
  digitalWrite(TRIG,HIGH);
  delay(1);
  digitalWrite(TRIG,LOW);
  DURATION = pulseIn(ECHO,HIGH);
  DISTANCE = DURATION / 58.2;
  
 if(DISTANCE > 0 && DISTANCE <7 ){
    client.beginMessage(TYPE_TEXT);
    client.print("ON");  // send data
    client.print(",");
    client.print("0");
    client.print(",");
    client.println("0");
    client.endMessage();
  }
   delay(100);
}
