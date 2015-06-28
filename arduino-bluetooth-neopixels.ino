/*
 * Arduino Bluetooth Neopixels - Control the color of the NeoPixels connected to the arduino with a phone
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
  uint32_t color = NeoPixels.Color((red * brightness) >> 8, (green * brightness) >> 8, (blue * brightness) >> 8);
  for(int i = 0; i < NEOPIXEL_AMOUNT; i++){
    NeoPixels.setPixelColor(i, color);
  }
  NeoPixels.show();
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
  lastRed = command[2];
  lastGreen = command[3];
  lastBlue = command[4];
  
  Serial.println("Color: (" + String(lastRed) + ", " + String(lastGreen) + ", " + String(lastBlue) + ")");
  setNeoPixelColor(lastRed, lastGreen, lastBlue, lastBrightness);
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
