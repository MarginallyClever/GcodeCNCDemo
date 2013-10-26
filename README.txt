#GcodeCNCDemo

This is a simple demonstration of a 2-axis CNC machine using an Arduino UNO and the Adafruit Motor Shield v1 and v2.

##Note

Some of the files have define that must be set.

MOTOR_SHIELD_VERSION (X) - change this to your version number

VERBOSE - define this to see how bresenham's line algo works even without steppers.

BAUD (57600) - How fast is the Arduino talking?  Default for us is 57600.
STEPS_PER_TURN (400) - depends on your stepper motor.  Mine are all 400.  Many are 200.

##Prerequisites

AFMotor: https://github.com/adafruit/Adafruit-Motor-Shield-library
Arduino: http://arduino.cc/

##More

For more info please see http://www.github.com/MarginallyClever/GcodeCNCDemo

##Author

http://marginallyclever.com
Dan Royer
2013-08-30

