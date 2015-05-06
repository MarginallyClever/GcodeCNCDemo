//------------------------------------------------------------------------------
// 2 Axis CNC Demo v2 - supports Adafruit motor shields v1 and v2
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define MOTOR_SHIELD_VERSION (1)  // change to your version number
//#define MOTOR_SHIELD_VERSION (2)  // Must choose one!

//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MAX_FEEDRATE         (10000)
#define MIN_FEEDRATE         (1)

#ifndef MOTOR_SHIELD_VERSION
#error MOTOR_SHIELD_VERSION must be defined!
#endif


// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#if MOTOR_SHIELD_VERSION == 1

#include <AFMotorDrawbot.h>

#endif
#if MOTOR_SHIELD_VERSION == 2

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

#endif
#if MOTOR_SHIELD_VERSION == 2

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 
Adafruit_StepperMotor *m1 = AFMS.getStepper(STEPS_PER_TURN, 1);  // to motor port #1 (M1 and M2)
Adafruit_StepperMotor *m2 = AFMS.getStepper(STEPS_PER_TURN, 2);  // to motor port #2 (M3 and M4)

#endif

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

float px, py;  // location

// speeds
float fr=0;  // human version
// machine version
long step_delay_ms;
long step_delay_us;

// settings
char mode_abs=1;  // absolute mode??

long line_number=0;


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


void tick() {
  delay(step_delay_ms);
  delayMicroseconds(step_delay_us);
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
  
  long us_per_min = 60 * 1000 * 1000;
  long x = us_per_min / nfr;
  step_delay_ms = x / 1000;
  step_delay_us = x % 1000;
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
      tick();
    }
  } else {
    for(i=0;i<dy;++i) {
      onestep(2,diry);
      over+=dx;
      if(over>=dy) {
        over-=dy;
        onestep(1,dirx);
      }
      tick();
    }
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  px=newx;
  py=newy;
}


// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0) a=(PI*2.0)+a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
static void arc(float cx,float cy,float x,float y,float dir) {
  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius=sqrt(dx*dx+dy*dy);

  // find angle of arc (sweep)
  float angle1=atan3(dy,dx);
  float angle2=atan3(y-cy,x-cx);
  float theta=angle2-angle1;
  
  if(dir>0 && theta<0) angle2+=2*PI;
  else if(dir<0 && theta>0) angle1+=2*PI;
  
  theta=angle2-angle1;
  
  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );
 
  float nx, ny, angle3, scale;

  for(i=0;i<segments;++i) {
    // interpolate around the arc
    scale = ((float)i)/((float)segments);
    
    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx,ny);
  }
  
  line(x,y);
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
  Serial.print(F("GcodeCNCDemo2AxisV2 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the input buffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  // blank lines
  if(buffer[0]==';') return;
  
  long cmd;
  
  // is there a line number?
  cmd=parsenumber('N',-1);
  if(cmd!=-1 && buffer[0]=='N') {  // line number must appear first on the line
    if( cmd != line_number ) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }
  
    // is there a checksum?
    if(strchr(buffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(buffer[c]!='*') checksum ^= buffer[c++];
      c++; // skip *
      int against = strtod(buffer+c,NULL);
      if( checksum != against ) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      } 
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }
    
    line_number++;
  }

  cmd = parsenumber('G',-1);
  switch(cmd) {
  case  0: 
  case  1: { // line
    feedrate(parsenumber('F',fr));
    line( parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py) );
    break;
    }
  case 2:
  case 3: {  // arc
      feedrate(parsenumber('F',fr));
      arc(parsenumber('I',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('J',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('X',(mode_abs?px:0)) + (mode_abs?0:px),
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          (cmd==2) ? -1 : 1);
      break;
    }
  case  4:  pause(parsenumber('S',0) + parsenumber('P',0)*1000.0f);  break;  // dwell
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
  case 110:  line_number = parsenumber('N',line_number);  break;
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
  
#if MOTOR_SHIELD_VERSION == 2
  AFMS.begin();  // create with the default frequency 1.6KHz
#endif

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
    if(sofar<MAX_BUF-1) buffer[sofar++]=c;  // store it
    if(c=='\n') {
      // entire message received
      // we got a message and it ends with a semicolon
      buffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/
