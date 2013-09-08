//------------------------------------------------------------------------------
// 6 Axis CNC Demo v2 - supports Adafruit motor shield v2
// dan@marginallycelver.com 2013-09-07
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.

#define VERSION              (2)  // firmware version
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (400)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (1500)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)
#define NUM_AXIES            (6)

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;
  long absdelta;
  int dir;
  long over;
  int motor;
} Axis;



//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// Initialize Adafruit stepper controller
Adafruit_MotorShield AFMS0 = Adafruit_MotorShield(0x61);
Adafruit_MotorShield AFMS1 = Adafruit_MotorShield(0x60);
Adafruit_MotorShield AFMS2 = Adafruit_MotorShield(0x63);
// Connect stepper motors with 400 steps per revolution (1.8 degree)
// Create the motor shield object with the default I2C address
Adafruit_StepperMotor *m[NUM_AXIES];


Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr=0;  // human version
long step_delay;  // machine version

float px,py,pz,pu,pv,pw;  // position

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
void position(float npx,float npy,float npz,float npu,float npv,float npw) {
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
  pu=npu;
  pv=npv;
  pw=npw;
}


/**
 * Supports movement with both styles of Motor Shield
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void onestep(int motor,int direction) {
#ifdef VERBOSE
  char *letter="UVWXYZ";
  Serial.print(letter[motor]);
#endif
  m[motor]->onestep(direction>0?FORWARD:BACKWARD,SINGLE);
}


void release() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {
    m[i]->release();
  }
}


/**
 * Uses bresenham's line algorithm to move both motors
 * @input newx the destination x position
 * @input newy the destination y position
 **/
void line(float newx,float newy,float newz,float newu,float newv,float neww) {
  a[0].delta = newx-px;
  a[1].delta = newy-py;
  a[2].delta = newz-pz;
  a[3].delta = newu-pu;
  a[4].delta = newv-pv;
  a[5].delta = neww-pw;
  
  long i,j;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].motor=i;
    a[i].dir=a[i].delta > 0 ? 1:-1;
    if(i%2) a[i].dir=-a[i].dir;  // because the motors are mounted in opposite directions
    a[i].absdelta = abs(a[i].delta);
  }
  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif

  // sort the axies with the fastest mover at the front of the list
  for(i=0;i<NUM_AXIES;++i) {
    for(j=i+1;j<NUM_AXIES;++j) {
      if(a[j].absdelta>a[i].absdelta) {
        memcpy(&atemp,&a[i] ,sizeof(Axis));
        memcpy(&a[i] ,&a[j] ,sizeof(Axis));
        memcpy(&a[j] ,&atemp,sizeof(Axis));
      }
    }
    a[i].over=0;
  }
  
  for(i=0;i<a[0].absdelta;++i) {
    onestep(a[0].motor,a[0].dir);
    
    for(j=1;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= a[0].absdelta) {
        a[j].over -= a[0].absdelta;
        onestep(a[j].motor,a[j].dir);
      }
    }
    pause(step_delay);
  }

#ifdef VERBOSE
  Serial.println(F("< Done."));
#endif

  position(newx,newy,newz,newu,newv,neww);
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
  output("Z",pz);
  output("U",pu);
  output("V",pv);
  output("W",pw);
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
  Serial.println(F("G00/G01 [X/Y/Z/U/V/W(steps)] [F(feedrate)]; - linear move"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X/Y/Z/U/V/W(steps)]; - change logical position"));
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
          parsenumber('Y',(mode_abs?py:0)) + (mode_abs?0:py),
          parsenumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz),
          parsenumber('U',(mode_abs?pu:0)) + (mode_abs?0:pu),
          parsenumber('V',(mode_abs?pv:0)) + (mode_abs?0:pv),
          parsenumber('W',(mode_abs?pw:0)) + (mode_abs?0:pw) );
    break;
  case  4:  pause(parsenumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parsenumber('X',0),
              parsenumber('Y',0),
              parsenumber('Z',0),
              parsenumber('U',0),
              parsenumber('V',0),
              parsenumber('W',0) );
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

  AFMS0.begin(); // Start the shields
  AFMS1.begin();
  AFMS2.begin();
  
  m[0] = AFMS0.getStepper(STEPS_PER_TURN, 1);
  m[1] = AFMS0.getStepper(STEPS_PER_TURN, 2);
  m[2] = AFMS1.getStepper(STEPS_PER_TURN, 1);
  m[3] = AFMS1.getStepper(STEPS_PER_TURN, 2);
  m[4] = AFMS2.getStepper(STEPS_PER_TURN, 1);
  m[5] = AFMS2.getStepper(STEPS_PER_TURN, 2);

  help();  // say hello
  position(0,0,0,0,0,0);  // set staring position
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


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DrawbotGUI is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/
