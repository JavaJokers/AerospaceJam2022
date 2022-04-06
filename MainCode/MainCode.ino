#include <SoftwareSerial.h>

#include <Adafruit_AS7341.h>

#include <Servo.h>

#define servo_pin 9

// Variables for bluetooth
SoftwareSerial Bluetooth(3, 2);
// Light Contorller
SoftwareSerial LightController(6, 7);
// Configuration and variables for LIDAR
SoftwareSerial Lidar(4, 5); //define software serial port name as Serial1 and define pin2 as RX and pin3 as TX
int lidarDist; //actual distance measurements of LiDAR
int lidarStrength; //signal strength of LiDAR
float lidarTemprature;
int check; //save check value
int i;
int uart[9]; //save data measured by LiDAR
const int HEADER = 0x59; //frame header of data package
String lidarQueryResponse;
String LidarQueryResponse;
String LightQueryResponse;
int moisturePin = A0; 
int moistureValue;  
int moistureLimit = 515; 
int val; // define numeric variables val

int hallPin = 10;

Adafruit_AS7341 as7341;

Servo servo;
int pos = 0;

void setup() {

  // Begin Serial and SoftwareSerial ports
  Serial.begin(9600);
  Lidar.begin(115200);
  Bluetooth.begin(9600);
  LightController.begin(9600);
  pinMode(hallPin, INPUT_PULLUP);

//  servo.attach(9);
//  servo.write(165);
  // Listen on Bluetooth port
  Bluetooth.listen();
  // Begin Light sensor
  if (!as7341.begin()) {
    Serial.println("Could not find AS7341");
    while (1) {
      delay(10);
    }
  }
  // Light sensor config
  as7341.setATIME(100);
  as7341.setASTEP(999);
  as7341.setGain(AS7341_GAIN_256X);
  as7341.setLEDCurrent(30);
  as7341.enableLED(false);
  Serial.println("Initialized all sensors, waiting for connections...");
  Bluetooth.listen();
}

void loop() {
  // Whilst reciving bluetooth messages
  if (Bluetooth.available()) {
    Serial.println("made it");
    char data = Bluetooth.read();
    String sData = String(data);
    Serial.println(sData);
    // If query is for lidar (letter 'A'), return lidar stats
    if (sData == "l") {
      fetchLidar();

      LidarQueryResponse = "Distance: " + String(lidarDist) + ":" + String(lidarStrength);
      Bluetooth.print(LidarQueryResponse);
    }
    // Elif query is for light wavelength sensor (letter 'B'), return light stats
    else if (sData == "c") {

      
      fetchLight();

      LightQueryResponse = String();
      int32_t R = int32_t(convertToRgb(as7341.getChannel(AS7341_CHANNEL_680nm_F8)));
      int32_t G = int32_t(convertToRgb(as7341.getChannel(AS7341_CHANNEL_515nm_F4)));
      int32_t B = int32_t(convertToRgb(as7341.getChannel(AS7341_CHANNEL_445nm_F2)));

      LightQueryResponse = RgbToHex(byte(R), byte(G), byte(B));
      
      LightController.listen();
      LightController.print(String(convertToRgb(as7341.getChannel(AS7341_CHANNEL_680nm_F8)))+","+String(convertToRgb(as7341.getChannel(AS7341_CHANNEL_515nm_F4)))+","+String(convertToRgb(as7341.getChannel(AS7341_CHANNEL_445nm_F2)))+";");
      Bluetooth.listen();
      
      Bluetooth.print(LightQueryResponse);

      
    } else if (sData == "s") {
      Serial.println("dirtgrabber close");
      servo.attach(servo_pin);
      for (pos = 165; pos >= 50; pos -= 1) { // goes from 180 degrees to 0 degrees
          servo.write(pos);              // tell servo to go to position in variable 'pos'
          delay(15);                       // waits 15 ms for the servo to reach the position
        }
      servo.detach();
      Bluetooth.print("Closed DirtGrabber™");
      

    } else if (sData == "h") {
      Serial.println("Hall Sensor");
      val = digitalRead(hallPin); // read sensor line
      if (val == LOW) {
        Bluetooth.print("Near a magnet!");
      } else {
        Bluetooth.print("Not near a magnet.");
      }

    } else if (sData == "r") {
      fetchLight();

      Bluetooth.print(" F1 (415nm, purple):" + String(as7341.getChannel(AS7341_CHANNEL_415nm_F1)) + " \n\rF2 (445nm, navy blue):" + String(as7341.getChannel(AS7341_CHANNEL_445nm_F2)) + " \n\rF3 (480nm, baby blue):" + String(as7341.getChannel(AS7341_CHANNEL_480nm_F3)) + " \n\rF4 (515nm, green):" + String(as7341.getChannel(AS7341_CHANNEL_515nm_F4)) + " \n\rF5 (555nm, yellow):" + String(as7341.getChannel(AS7341_CHANNEL_555nm_F5)) + " \n\rF6 (590nm, orange):" + String(as7341.getChannel(AS7341_CHANNEL_590nm_F6)) + " \n\rF7 (630nm, orangeish red):" + String(as7341.getChannel(AS7341_CHANNEL_630nm_F7)) + " \n\rF8 (680nm, red):" + String(as7341.getChannel(AS7341_CHANNEL_680nm_F8)));
    } else if (sData == "m") {

      // Completely Dry - 627 to 752
      // Barely touching - 456
      // Halfway submerged - 279 - 292
      // Completely Wet - 239 to 243
      moistureValue = analogRead(moisturePin); 
      String onOff = "ERROR!";
      
       if (moistureValue<moistureLimit) {
       onOff = "Completely Wet";
       }
       else if (moistureValue>627) {
       onOff = "Completely Dry";
       }
       else if(moistureValue>456&&moistureValue<627){
       onOff = "Damp"; 
       }
       else if(moistureValue>279&&moistureValue<292){
       onOff = "Halfway Submerged";
       }
       String response = "Moisture: " + onOff + "\r\n  Raw value: " + String(moistureValue) + "\r\n  Percentage: " + String(map(moistureValue, 740, 320, 0, 100)) + "%";
       Bluetooth.print(response);
    }
    else if (sData == "o") {
      servo.attach(servo_pin);
      for (pos = 50; pos <= 165; pos += 1) { // goes from 0 degrees to 180 degrees
        // in steps of 1 degree
        servo.write(pos);              // tell servo to go to position in variable 'pos'
        delay(15);                       // waits 15 ms for the servo to reach the position
      }
      servo.detach();
      Bluetooth.print("Opened DirtGrabber™");

  }

}

}

void fetchLidar() { // TODO: if check fails, retry up to 10 times
  Lidar.listen();
  delay(75);
  Serial.println("Fetching lidar stats");
  if (Lidar.available()) { //check if serial port has data input
    Serial.println("Lidar available");
    if (Lidar.read() == HEADER) { //assess data package frame header 0x59
      uart[0] = HEADER;
      if (Lidar.read() == HEADER) { //assess data package frame header 0x59
        uart[1] = HEADER;
        for (i = 2; i < 9; i++) { //save data in array
          uart[i] = Lidar.read();
        }
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];
        if (uart[8] == (check & 0xff)) { //verify the received data as per protocol
          Serial.println(String(uart[2]));
          Serial.println(String(uart[3]));
          lidarDist = uart[2] + uart[3] * 256; //calculate distance value
          lidarStrength = uart[4] + uart[5] * 256; //calculate signal strength value
          lidarTemprature = uart[6] + uart[7] * 256; //calculate chip temprature
          lidarTemprature = lidarTemprature / 8 - 256;
        } else {
         Bluetooth.listen();
         Bluetooth.print("["+String(millis())+"] ERROR! Failed to confirm checksum of LIDAR data. Retrying...");
         delay(100);
         Lidar.listen();
         fetchLidar();
        }
      }
    }
    Serial.println("Distance: " + String(lidarDist));
    Bluetooth.listen();
  }
}

void fetchLight() {
  as7341.enableLED(true);
  delay(50);
  if (!as7341.readAllChannels()) {
    Serial.println("ERROR reading channels in light sensor!");
    return;
  }
  Serial.print("F1 415nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_415nm_F1));
  Serial.print("F2 445nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_445nm_F2));
  Serial.print("F3 480nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_480nm_F3));
  Serial.print("F4 515nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_515nm_F4));
  Serial.print("F5 555nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_555nm_F5));
  Serial.print("F6 590nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_590nm_F6));
  Serial.print("F7 630nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_630nm_F7));
  Serial.print("F8 680nm : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_680nm_F8));

  Serial.print("Clear    : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_CLEAR));

  Serial.print("Near IR  : ");
  Serial.println(as7341.getChannel(AS7341_CHANNEL_NIR));
  as7341.enableLED(false);
  return;
}

String RgbToHex(byte R, byte G, byte B) {
  char hex[7] = {
    0
  };
  sprintf(hex, "%02X%02X%02X", R, G, B); //convert to an hexadecimal string. Lookup sprintf for what %02X means.
  return "#" + String(hex);
}

int convertToRgb(uint16_t x) {
  Serial.println("Out: " + String(map(x, 0, 65535, 0, 255)) + "in:" + String(x));
  return map(x, 0, 65535, 0, 255);
}
