#include <SoftwareSerial.h>
SoftwareSerial Bluetooth(3,2);

void setup() {
  Bluetooth.begin(38400);
  Serial.begin(9600);
  Serial.println("          HC-05 AT Commands");
  Serial.println("SET NAME         |AT+NAME=<name>   |");
  Serial.println("    Enter an AT command to start");
} 

void loop() {
  if(Bluetooth.available()){
      Serial.write(Bluetooth.read());
  }

  if(Serial.available()){
      Bluetooth.write(Serial.read());  
  }
}
