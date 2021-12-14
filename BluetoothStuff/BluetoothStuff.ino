#include <SoftwareSerial.h>

SoftwareSerial Bluetooth(3,2);

void setup() {
  Bluetooth.begin(9600);
  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Bluetooth.print("AT+NAME=Golda1");
  Serial.println("Loaded bluetooth stuff");
}

void loop() {
  if(Bluetooth.available()>0)
  {
    char data = Bluetooth.read();
    String sData = String(data);
    Serial.println(sData);
    if (sData == "e")
    {
      Serial.println("sending test");
      digitalWrite(13,HIGH);
      Bluetooth.print("test message 1234567890");
      digitalWrite(13,LOW);
    }
  }


}
