//------------------------------------------------------------------------------
// 2 Axis CNC Demo v2 - supports Adafruit motor shields v1 and v2
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/2AxisCNCDemo for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define MOTOR_SHIELD_VERSION (1)  // change to your version number
#define MOTOR_SHIELD_VERSION (2)  // Must choose one!

//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (3500)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)

#ifndef MOTOR_SHIELD_VERSION
#error MOTOR_SHIELD_VERSION must be defined!
#endif


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#if MOTOR_SHIELD_VERSION == 1

#include <AFMotorDrawbot.h>

#else  // MOTOR_SHIELD_VERSION == 2

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#endif


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// Initialize Adafruit stepper controller
// Connect stepper motors with 400 steps per revolution (1.8 degree)
#if MOTOR_SHIELD_VERSION == 1

static AF_Stepper m1(STEPS_PER_TURN, 1);  // to motor port #1 (M1 and M2)
static AF_Stepper m2(STEPS_PER_TURN, 2);  // to motor port #2 (M3 and M4)

#else  // MOTOR_SHIELD_VERSION == 2

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *m1 = AFMS.getStepper(STEPS_PER_TURN, 1);  // to motor port #1 (M1 and M2)
Adafruit_StepperMotor *m2 = AFMS.getStepper(STEPS_PER_TURN, 2);  // to motor port #2 (M3 and M4)

#endif

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

float px, py;  // location

// speeds
float fr=0;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */
void position(float npx,float npy) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
}


/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void onestep(int motor,int direction) {
  if(motor==1) {
#ifdef VERBOSE
    Serial.print('X');
#endif
#if MOTOR_SHIELD_VERSION == 1
      m1.onestep(direction);
#else
      m1->onestep(direction>0?FORWARD:BACKWARD,SINGLE);
#endif
  } else {
#ifdef VERBOSE
    Serial.print('Y');
#endif
#if MOTOR_SHIELD_VERSION == 1
      m2.onestep(direction);
#else
      m2->onestep(direction>0?FORWARD:BACKWARD,SINGLE);
#endif
  }
}


void release() {
#if MOTOR_SHIELD_VERSION == 1
  m1.release();
  m2.release();
#else
  m1->release();
  m2->release();
#endif
}

/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy) {
  long dx=newx-px;
  long dy=newy-py;
  int dirx=dx>0?1:-1;
  int diry=dy>0?-1:1;  // because the motors are mounted in opposite directions
  dx=abs(dx);
  dy=abs(dy);

  long i;
  long over=0;

#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  if(dx>dy) {
    for(i=0;i<dx;++i) {
      onestep(1,dirx);
      over+=dy;
      if(over>=dx) {
        over-=dx;
        onestep(2,diry);
      }
      pause(step_delay);
    }
  } else {
    for(i=0;i<dy;++i) {
      onestep(2,diry);
      over+=dx;
      if(over>=dy) {
        over-=dy;
        onestep(1,dirx);
      }
      pause(step_delay);
    }
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  px=newx;
  py=newy;
}


/**
 * Look for character /code/ in the buffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parsenumber(char code,float val) {
  char *ptr=buffer;
  while(ptr && *ptr && ptr<buffer+sofar) {
    if(*ptr==code) {
      return atof(ptr+1);
    }
    ptr=strchr(ptr,' ')+1;
  }
  return val;
} 


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code,float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("MixologyBot "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  int cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0: // move linear
  case  1: // move linear
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    break;
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0) );
    break;
  default:  break;
  }

  cmd = parsenumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    release();
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  
  help();  // say hello
  position(0,0);  // set staring position
  feedrate(200);  // set default speed
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF) buffer[sofar++]=c;  // store it
    if(buffer[sofar-1]==';') break;  // entire message received
  }

  if(sofar>0 && buffer[sofar-1]==';') {
    // we got a message and it ends with a semicolon
    buffer[sofar]=0;  // end the buffer so string functions work right
    Serial.print(F("\r\n"));  // echo a return character for humans
    processCommand();  // do something with the command
    ready();
  }
}


//------------------------------------------------------------------------------
// Copyright (C) 2013 Dan Royer (dan@marginallyclever.com)
// Permission is hereby granted, free of charge, to any person obtaining a 
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation 
// the rights to use, copy, modify, merge, publish, distribute, sublicense, 
// and/or sell copies of the Software, and to permit persons to whom the 
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//------------------------------------------------------------------------------