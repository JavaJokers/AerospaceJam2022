#include <SoftwareSerial.h> // Add SoftwareSerial Library

#include <Adafruit_NeoPixel.h> // Add Neopixel Library

#include <StringSplitter.h> // Add Sting Managment Library

#ifdef __AVR__
  #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LF_LED_PIN 6
#define RF_LED_PIN 5
#define LB_LED_PIN 7
#define RB_LED_PIN 8

String r;
String g;
String b;

int effect = 1;
// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 4

Adafruit_NeoPixel stripLF(LED_COUNT, LF_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripRF(LED_COUNT, RF_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripLB(LED_COUNT, LB_LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripRB(LED_COUNT, RB_LED_PIN, NEO_GRB + NEO_KHZ800);

SoftwareSerial SensorArduino(2, 3); // Define SoftwareSerial object for the main sensor board

// Global Vars
String command;

void setup() {
  Serial.begin(9600); // Begin debugging Serial port
  Serial.println("Serial enabled");
  pinMode(13, OUTPUT);
  SensorArduino.begin(9600); // Begin SoftwareSerial port

  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.

  stripLF.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  stripLF.show(); // Turn OFF all pixels ASAP
  stripLF.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  stripRF.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  stripRF.show(); // Turn OFF all pixels ASAP
  stripRF.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  stripLB.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  stripLB.show(); // Turn OFF all pixels ASAP
  stripLB.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)

  stripRB.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  stripRB.show(); // Turn OFF all pixels ASAP
  stripRB.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
}

void loop() {
  if(effect == 1){
    rainbow(25);  
  }
}

void colorWipe(uint32_t color, int wait, int endWait) {
  for (int i = 0; i < stripLF.numPixels(); i++) { // For each pixel in strip...
    stripRF.setPixelColor(i, color); //  Set pixel's color (in RAM)
    stripLF.setPixelColor(i, color); //  Set pixel's color (in RAM)
    stripRB.setPixelColor(i, color); //  Set pixel's color (in RAM)
    stripLB.setPixelColor(i, color); //  Set pixel's color (in RAM)
    stripRF.show(); //  Update strip to match
    stripLF.show(); //  Update strip to match
    stripRB.show(); //  Update strip to match
    stripLB.show(); //  Update strip to match
    delay(wait); //  Pause for a moment
  }
  delay(endWait);
}

void checkForMessage() {

  if (SensorArduino.available()) { // If data can be recieved (If there is data to be read)
    command = SensorArduino.readStringUntil(';'); // Recieve data until newline character

    Serial.println(command); // Print the command recieved for debugging

      colorConverter(command);
      Serial.print("r: ");
      Serial.println(r.toInt());
      Serial.println(g.toInt());
      Serial.println(b.toInt());
      colorWipe(stripLF.Color(g.toInt(), r.toInt(), b.toInt()), 50, 5000);
  }
}

void colorConverter(String rgbString) {
  StringSplitter * splitter = new StringSplitter(rgbString, ',', 3);
  int itemCount = splitter -> getItemCount();

  for (int i = 0; i < itemCount; i++) {
    String item = splitter -> getItemAtIndex(i);

    if (i == 0) {
      r = item;
    } else if (i == 1) {
      g = item;
    } else if (i == 2) {
      b = item;
    }
  }

}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < stripLF.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / stripLF.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      stripLF.setPixelColor(i, stripLF.gamma32(stripLF.ColorHSV(pixelHue)));
      stripRF.setPixelColor(i, stripRF.gamma32(stripRF.ColorHSV(pixelHue)));
      stripRB.setPixelColor(i, stripRB.gamma32(stripRB.ColorHSV(pixelHue)));
      stripLB.setPixelColor(i, stripLB.gamma32(stripLB.ColorHSV(pixelHue)));
    }
    stripLF.show();
    stripRF.show();
    stripRB.show();
    stripLB.show();// Update strip with new contents
    delayWithChecking(wait); // Pause for a moment
  }
}

void delayWithChecking(unsigned int delayAmount) { // Non-blocking version of delay() so that we can use it to wait during light functions without blocking the event loop of checking for message availability

  unsigned long millisToStop = millis() + delayAmount;

  while (millisToStop >= millis()) {
    checkForMessage();
  }
}
