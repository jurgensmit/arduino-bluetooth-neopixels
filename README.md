# Arduino Bluetooth NeoPixels
Control the color of the NeoPixels connected to the arduino with a phone. 

Connect a Bluetooth low energy breakout board to an Arduino and some NeoPixels. Download the iOS app
and control the color of the NeoPixels with an iPhone or iPad :) Either use the color picker module to pick
a color or stream the Quaternion sensor data which sets the color based on the pitch, yaw and roll 
rotation of the device. A potentiometer directly connected to the Arduino is used to control the 
brightness of the NeoPixels.

![Fritzing Sketch](https://github.com/jurgensmit/arduino-bluetooth-neopixels/blob/master/images/arduino-bluetooth-neopixels.png "Arduino Bluetooth NeoPixels")

## Do it yourself

1. Build the circuit according the Fritzing sketch
2. Compile and upload the Arduino sketch
3. Fire up the iPhone or Ipad app
4. Connect the app to the Arduino
5. Control the color of the NeoPixels with the iPhone or iPad

## What you need

1. Arduino
2. Adafruit nRF8001 Bluefruit LE Breakout Board
3. Some NeoPixels
4. A 470 ohm resistor
5. A 10 kOhm potentiometer

Optionally

5. Led + 1 kOhm Resistor as status led to show the project is active

## Result

![iOS App](https://github.com/jurgensmit/arduino-bluetooth-neopixels/blob/master/images/iOSApp.png "Arduino Bluetooth NeoPixels") ![Project Photo](https://github.com/jurgensmit/arduino-bluetooth-neopixels/blob/master/images/Photo.jpg "Arduino Bluetooth NeoPixels")
