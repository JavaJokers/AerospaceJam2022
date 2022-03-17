#include <SoftwareSerial.h> // Add SoftwareSerial library

SoftwareSerial SensorArduino(5, 6); // Define SoftwareSerial object for the main sensor board
String command;
void setup() {
  Serial.begin(9600); // Begin debugging Serial port
  Serial.println("test");
  pinMode(13, OUTPUT);
  SensorArduino.begin(9600); // Begin SoftwareSerial port
}

void loop() {
  checkForMessage();
}

void checkForMessage() {
    if(SensorArduino.available()){ // If data can be recieved (If there is data to be read)
        command = SensorArduino.readStringUntil('\n'); // Recieve data until newline character
         
        Serial.println(command); // Print the command recieved for debugging

        if(command == "H"){
          digitalWrite(13, HIGH);  
        } else if (command == "L"){
          digitalWrite(13, LOW);
        }
    }  
}

void delayWithChecking(unsigned int delayAmount) { // Non-blocking version of delay() so that we can use it to wait during light functions without blocking the event loop of checking for message availability

    unsigned long millisToStop = millis() + delayAmount;

    while(millisToStop >= millis()){
      checkForMessage();
    }
}
