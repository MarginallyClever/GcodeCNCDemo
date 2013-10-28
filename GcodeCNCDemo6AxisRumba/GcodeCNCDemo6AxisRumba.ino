//------------------------------------------------------------------------------
// 6 Axis CNC Demo Rumba - supports raprapdiscount RUMBA controller
// dan@marginallycelver.com 2013-10-28
// RUMBA should be treated like a MEGA 2560 Arduino.
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
#define MIN_STEP_DELAY       (50)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)
#define NUM_AXIES            (6)


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];

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
void onestep(int motor) {
#ifdef VERBOSE
  char *letter="UVWXYZ";
  Serial.print(letter[motor]);
#endif
  
  digitalWrite(motors[motor].step_pin,HIGH);
  digitalWrite(motors[motor].step_pin,LOW);
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
  
  long i,j,maxsteps=0;

  for(i=0;i<NUM_AXIES;++i) {
    a[i].absdelta = abs(a[i].delta);
    a[i].over=0;
    if( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;
    // set the direction once per movement
    digitalWrite(motors[i].dir_pin,a[i].delta>0?HIGH:LOW);
  }
  
#ifdef VERBOSE
  Serial.println(F("Start >"));
#endif
  
  for( i=0; i<maxsteps; ++i ) {
    for(j=0;j<NUM_AXIES;++j) {
      a[j].over += a[j].absdelta;
      if(a[j].over >= a[0].absdelta) {
        a[j].over -= a[0].absdelta;
        onestep(j);
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
  Serial.print(F("GcodeCNCDemo6AxisV2 "));
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
  case 17:  motor_enable();  break;
  case 18:  motor_disable();  break;
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
 * set up the pins for each motor
 */
void motor_setup() {
  motors[0].step_pin=17;
  motors[0].dir_pin=16;
  motors[0].enable_pin=48;
  motors[0].limit_switch_pin=37;

  motors[1].step_pin=54;
  motors[1].dir_pin=47;
  motors[1].enable_pin=55;
  motors[1].limit_switch_pin=36;

  motors[2].step_pin=57;
  motors[2].dir_pin=56;
  motors[2].enable_pin=62;
  motors[2].limit_switch_pin=35;

  motors[3].step_pin=23;
  motors[3].dir_pin=22;
  motors[3].enable_pin=27;
  motors[3].limit_switch_pin=34;

  motors[4].step_pin=26;
  motors[4].dir_pin=25;
  motors[4].enable_pin=24;
  motors[4].limit_switch_pin=33;

  motors[5].step_pin=29;
  motors[5].dir_pin=28;
  motors[5].enable_pin=39;
  motors[5].limit_switch_pin=32;
  
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    // set the motor pin & scale
    pinMode(motors[i].step_pin,OUTPUT);
    pinMode(motors[i].dir_pin,OUTPUT);
    pinMode(motors[i].enable_pin,OUTPUT);
  }
}


void motor_enable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,LOW);
  }
}


void motor_disable() {
  int i;
  for(i=0;i<NUM_AXIES;++i) {  
    digitalWrite(motors[i].enable_pin,HIGH);
  }
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms

  motor_setup();
  motor_enable();
  
  help();  // say hello
  position(0,0,0,0,0,0);  // set staring position
  feedrate(1000);  // set default speed
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
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/
