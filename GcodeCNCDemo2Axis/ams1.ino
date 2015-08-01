//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

#if CONTROLLER == AMS1

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include <AFMotorDrawbot.h>


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// Initialize Adafruit stepper controller
AF_Stepper m1((int)STEPS_PER_TURN, 1);
AF_Stepper m2((int)STEPS_PER_TURN, 2);


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void m1step(int dir) {
  m1.onestep(dir);
}

void m2step(int dir) {
  m2.onestep(dir);
}

void disable() {
    m1.release();
    m2.release();
}


void setup_controller() {}


#endif  // CONTROLLER == AMS1
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
