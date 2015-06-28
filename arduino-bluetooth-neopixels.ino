/*
 * Arduino Bluetooth Neopixels - Control the color of the NeoPixels connected to the arduino with a phone
 *
 * Connect a Bluetooth low energy breakout board to an Arduino and some NeoPixels. Download the iOS app
 * and control the color of the NeoPixels with an iPhone or iPad :) Either use the color picker module to pick
 * a color or stream the Quaternion sensor data which sets the color based on the pitch, yaw and roll 
 * rotation of the device. A potentiometer directly connected to the Arduino is used to control the 
 * brightness of the NeoPixels.
 *
 * (c) 2015 Jurgen Smit. All rights reserved.
 *
 * Circuit: 
 *
 * - nRF8001 Bluefruit LE Breakout connected to the SPI interface of the Arduino + Pin 2 for interupts
 *      https://www.adafruit.com/products/1697
 * - Some NeoPixles connected to pin 6
 * - A potmeter connected to anlog port A0
 *
 * External References:
 *
 * - Adafruit NeoPixel Library: https://github.com/adafruit/Adafruit_NeoPixel
 * - Adafruit nRF8001 Bluetooth Low Energy Breakout Library: https://github.com/adafruit/Adafruit_nRF8001
 * - Adafruit Bluefruit LE Connect iOS App: https://itunes.apple.com/us/app/adafruit-bluefruit-le-connect/id830125974?mt=8
 *
 */

#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <Adafruit_NeoPixel.h>

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

#define NEOPIXEL_DATA_IN 6    // Data inpur pin of the neopixels
#define NEOPIXEL_AMOUNT 16

#define POTMETER_PIN 0        // Analog potmeter pin
#define STATUSLED_PIN 3       // Status led

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
Adafruit_NeoPixel NeoPixels = Adafruit_NeoPixel(NEOPIXEL_AMOUNT, NEOPIXEL_DATA_IN, NEO_GRB + NEO_KHZ800);

// The current color and brightness of the neo pixels
byte lastRed = 255;
byte lastGreen = 255;
byte lastBlue = 255;
byte lastBrightness = 64;

void setup() {
  // Initialize the serial port
  Serial.begin(115200);
  Serial.println(F("Control the color of the neopixels connected to the arduino with a phone"));

  // Initialize the status led
  //pinMode(STATUSLED_PIN, OUTPUT);
  //digitalWrite(STATUSLED_PIN, HIGH);

  // Initialize the neopixels
  lastBrightness = getBrightness();
  NeoPixels.begin();
  setNeoPixelColor(lastRed, lastGreen, lastBlue, lastBrightness);
  
  // Initialize the bluetooth device
  BTLEserial.setDeviceName("BLENEOP"); 
  BTLEserial.begin();
}

/*
 * Get the brightness based on the value of the potmeter
 */
byte getBrightness() {
  int value = analogRead(POTMETER_PIN);
  // Only use 64 steps to decrease jitter
  int brightness = map(value, 0, 1023, 0, 63);
  return brightness * 4;
}

/*
 * Set the color and brightness of the NeoPixels
 */
void setNeoPixelColor(byte red, byte green, byte blue, byte brightness) {

  Serial.println("Color: (R:" + String(red) + ", G:" + String(green) + ", B:" + String(blue) + ", I:" + String(brightness) + ")");
  
  // Calculate the final rgb values based on the given colors and brightness
  uint32_t color = NeoPixels.Color((red * brightness) >> 8, (green * brightness) >> 8, (blue * brightness) >> 8);
  
  // Set the color of the neopixels
  for(int i = 0; i < NEOPIXEL_AMOUNT; i++){
    NeoPixels.setPixelColor(i, color);
  }
  NeoPixels.show();
  
  // Remember the current (raw) values 
  lastRed = red;
  lastGreen = green;
  lastBlue = blue;
  lastBrightness = brightness;
  
  // Set the brightness of the status led inverse to the brightness of the neopixels
  analogWrite(STATUSLED_PIN, 255 - brightness);
}

/*
 * Retrieve the bluetooth device status and write any changes to the serial port
 */
aci_evt_opcode_t getBLEState() {
  // The last status of the bluetooth device
  static aci_evt_opcode_t lastBLEStatus = ACI_EVT_DISCONNECTED;
  
  // Ask what is our current status
  aci_evt_opcode_t bleStatus = BTLEserial.getState();

  // If the status changed....
  if (bleStatus != lastBLEStatus) {
    // print it out!
    if (bleStatus == ACI_EVT_DEVICE_STARTED) {
        Serial.println(F("* Advertising started"));
    }
    if (bleStatus == ACI_EVT_CONNECTED) {
        Serial.println(F("* Connected!"));
    }
    if (bleStatus == ACI_EVT_DISCONNECTED) {
        Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    lastBLEStatus = bleStatus;
  }
  
  return bleStatus;
}

/*
 * Process the color command
 */
void processColorCommand(byte *command) {
  // Get the parameters of the command
  byte red = command[2];
  byte green = command[3];
  byte blue = command[4];
  
  // Set the color of the nexpixels
  setNeoPixelColor(red, green, blue, lastBrightness);
}

/*
 * Process the quaternion command
 */
void processQuaternionCommand(byte *command) {
  // Get the parameters of the command, all parameters are floats ranging -1.0 ... +1.0
  float x = *(float *)&command[2];
  float y = *(float *)&command[6];
  float z = *(float *)&command[10];
  float w = *(float *)&command[14];

  Serial.println("xyzw: (" + String(x) + ", " + String(y) + ", " + String(z) + ", " + String(w) + ")");
  
  // Calculate the color based on the position of the phone
  byte red = abs(x) * 255;
  byte green = abs(y) * 255;
  byte blue = abs(z) * 255;
  
  // Set the color of the nexpixels
  setNeoPixelColor(red, green, blue, lastBrightness);
}

/*
 * Process a single command
 */
void processCommand(byte *command, byte commandLength) {
  if(commandLength == 6) {
    if(command[0] == '!' && command[1] == 'C') {
      processColorCommand(command);
    }
  }
  else if(commandLength == 19) {
    if(command[0] == '!' && command[1] == 'Q') {
      processQuaternionCommand(command);
    }
  }
}

/*
 * Handle any incoming messages from the bluetooth device if available
 */
void handleIncomingBLEMessages() {
  static byte commandBuffer[20];
  static byte bufferPointer = 0;

  if (BTLEserial.available()) {
    Serial.print(F("* ")); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));

    while (BTLEserial.available()) {
      if(bufferPointer < sizeof(commandBuffer)) {
        byte data = BTLEserial.read();
        commandBuffer[bufferPointer++] = data;
      }
      else {
        /* Buffer overrun, start from scratch */
        bufferPointer = 0;
      }
    }

    if(bufferPointer > 0) {
      processCommand(commandBuffer, bufferPointer);
      bufferPointer = 0;
    }
  }
}

void loop() {
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Check if the bluetooth is connected
  aci_evt_opcode_t bleStatus = getBLEState();
  if (bleStatus == ACI_EVT_CONNECTED) {
    // Check and handle any incoming messages
    handleIncomingBLEMessages();
  }    

  // Set the brightness according the value of the potmeter
  byte brightness = getBrightness();
  if(brightness != lastBrightness) {
    setNeoPixelColor(lastRed, lastGreen, lastBlue, brightness);
    lastBrightness = brightness;
  }
}
