//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

#include "config.h"

#if CONTROLLER == AMS2

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"


#if CONTROLLER = AMS2

// Make sure you set the right address.  If you aren't sure,
// use http://playground.arduino.cc/Main/I2cScanner to find it.
//#define AFMS2_ADDRESS  (0x60)

#ifndef AFMS2_ADDRESS
#error AFMS2_ADDRESS must be defined!
#endif
#endif

//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(AMS2_ADDRESS); 
Adafruit_StepperMotor *m1 = AFMS.getStepper(STEPS_PER_TURN, 1);  // to motor port #1 (M1 and M2)
Adafruit_StepperMotor *m2 = AFMS.getStepper(STEPS_PER_TURN, 2);  // to motor port #2 (M3 and M4)


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void m1step(int dir) {
  m1->onestep(dir>0?FORWARD:BACKWARD,SINGLE);
}

void m2step(int dir) {
  m2->onestep(dir>0?FORWARD:BACKWARD,SINGLE);
}

void disable() {
  m1->release();
  m2->release();
}


void setup_controller() {
  AFMS.begin();  // create with the default frequency 1.6KHz
}


#endif  // CONTROLLER == AMS2

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
