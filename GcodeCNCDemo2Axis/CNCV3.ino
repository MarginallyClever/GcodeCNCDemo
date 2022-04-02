//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallyclever.com 2015-12-23
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

#if CONTROLLER == CNCV3

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define M1_STEP 2
#define M1_DIR  5
#define M1_ENA  8

#define M2_STEP 3
#define M2_DIR  6
#define M2_ENA  8

// limit switches
#define SWITCH1 9
#define SWITCH2 10

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
  
//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void m1step(int dir) {
  digitalWrite(M1_ENA,LOW);
  digitalWrite(M1_DIR,dir);
  digitalWrite(M1_STEP,HIGH);
  digitalWrite(M1_STEP,LOW);
}

void m2step(int dir) {
  digitalWrite(M2_ENA,LOW);
  digitalWrite(M2_DIR,dir);
  digitalWrite(M2_STEP,HIGH);
  digitalWrite(M2_STEP,LOW);
}

void disable() {
  digitalWrite(M1_ENA,HIGH);
  digitalWrite(M2_ENA,HIGH);
}


void setup_controller() {
  pinMode(M1_ENA,OUTPUT);
  pinMode(M2_ENA,OUTPUT);
  pinMode(M1_STEP,OUTPUT);
  pinMode(M2_STEP,OUTPUT);
  pinMode(M1_DIR,OUTPUT);
  pinMode(M2_DIR,OUTPUT);
}


#endif  // CONTROLLER == CNCV3
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



