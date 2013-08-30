This is a simple demonstration of a 2-axis CNC machine using an Arduino UNO and the Adafruit Motor Shield v1 and v2.

Be sure to adjust the following values before compiling your code:

//#define MOTOR_SHIELD_VERSION (1)  // change to your version number
#define MOTOR_SHIELD_VERSION (2)  // Must choose one!

//#define VERBOSE              (1)  // add to get a lot more serial output.

#define BAUD                 (57600)  // How fast is the Arduino talking?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.

