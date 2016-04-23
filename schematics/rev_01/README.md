This hardware version consists of the following components:

* [SparkFun RedBoard](https://www.sparkfun.com/products/12757)
* [SparkFun USB Host Shield](https://www.sparkfun.com/products/9947) -- also order a [stackable header kit](https://www.sparkfun.com/products/11417) because this comes with no headers
* [LCD Button Shield V2](https://www.sparkfun.com/products/13293)
* [SparkFun Barometric Pressure Sensor Breakout - BMP180](https://www.sparkfun.com/products/11824) -- also order a row of [breakaway headers](https://www.sparkfun.com/products/116)
* [Garmin ANT+ Stick](http://www.amazon.com/Garmin-USB-Stick-Fitness-Devices/dp/B00CM381SQ)
* [HK Pilot Analog Air Speed Sensor And Pitot Tube Set](http://www.hobbyking.com/hobbyking/store/__67371__HK_Pilot_Analog_Air_Speed_Sensor_And_Pitot_Tube_Set.html)

The schematics in this directory document how all these components connect to each other but from a practical perspective there are a few things of note:

1. The USB host shield connects digital pin 7 to the IC's reset but the driver we're using just uses the standard reset pin so we need to run a jumper from that pin to RST.
2. The LCD shield uses pins 7-9 which conflicts with the USB host shield so we need to snip off those pins and jumper them to pins 3-1 respectively ([photo](https://goo.gl/photos/3Y3cHiHikDKRrcxn8)).
3. I mounted both the header for the airspeed sensor and the barometer to the prototype area  of the USB host shield and just used wire wrap to connect them to the correct pins on the shield ([top](https://goo.gl/photos/juqtsfn3JbZ5vNQN8) and [bottom](https://goo.gl/photos/Y9SFbCevF5JDuEbC7)).

You do need to solder on the headers.  You could also solder on the jumpers but I used wire wrap because I wanted to be able to quickly and cleanly undo everything once I get rev 2 working.
