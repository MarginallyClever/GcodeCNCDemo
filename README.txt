#GcodeCNCDemo

This is a simple demonstration of a 2-axis CNC machine using an Arduino UNO and the Adafruit Motor Shield v1 and v2.

##Which files should I use?

If you have an Adafruit Motor Shield v1 then use GcodeCNCDemo2AxisV1.
If you have an Adafruit Motor Shield v2 then use GcodeCNCDemo2AxisV2, GcodeCNCDemo4AxisV2, or GcodeCNCDemo6AxisV2.
If you have a RUMBA controller then use GcodeCNCDemo6AxisRumba.

##Note

You can use any of of the *V2 with an Adafruit Motor Shield v2.
With all V2 code you will have to change the address of the board.  Look for

  Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x61);

and change the 0x61 to the address of your shield.

##Prerequisites

AFMotor: https://github.com/adafruit/Adafruit-Motor-Shield-library
Arduino: http://arduino.cc/

##More

For the latest version please visit http://www.github.com/MarginallyClever/GcodeCNCDemo
For more info on Adafruit shields please visit adafruit.com
For more info on RUMBA controllers please visit http://reprap.org/wiki/RUMBA

##Author

http://marginallyclever.com
Dan Royer
2013-08-30

