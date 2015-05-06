#GcodeCNCDemo

This is a simple demonstration CNC machines using an Arduino.

##Which files should I use?

GcodeCNCDemo2AxisV1 uses an UNO R3 and the Adafruit Motor Shield v1.
GcodeCNCDemo*AxisV2 uses an UNO R3 and the Adafruit Motor Shield v2.
GcodeCNCDemo6AxisRumba* uses an RUMBA board.

GcodeCNCDemo6AxisRumba abuses the timer interrupt on the RUMBA to eliminate the 'hiccough' between commands.  It can also 'look ahead' and plan more efficient movement.

##Sending whole files

So you've used a demo to make a CNC machine, and now you want to deliver a whole file full of gcode.

See our sister project, the improved Arduino serial monitor: https://github.com/MarginallyClever/gcodesender

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

