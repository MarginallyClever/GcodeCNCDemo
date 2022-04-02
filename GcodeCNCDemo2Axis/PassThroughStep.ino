//------------------------------------------------------------------------------
// 2 Axis CNC Demo
// dan@marginallycelver.com 2013-08-30
// modified by lsahidin@yahoo.com added HG7881 stepper controller
// modified by drf5na@gmail.com added a simple pass-through Stepper.h driver
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

// Warning.
// * to reduce and avoid overheat on HG7881, use M18 after every G00, G01, G02, G03 on Gcode * \\

#if CONTROLLER == PASS_STEP

//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include <Stepper.h> // https://github.com/arduino-libraries/Stepper/blob/master/src/Stepper.h


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// Initialize Stepper.h stepper controller
// Stepper.h motor(Step, pinA_IA, pinA_IB, pinB_IA, pin_B_IB);
Stepper m1((int)STEPS_PER_TURN, 6,7,8,9);
Stepper m2((int)STEPS_PER_TURN, 5,4,3,2);


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void m1step(int dir) {
    m1.step(dir);
}

void m2step(int dir) {
    m2.step(dir);
}

void disable() {
    //m1.release();
    //m2.release();
}


void setup_controller() {
    ;
    m1.setSpeed(100);
    m2.setSpeed(100);
}


#endif  // CONTROLLER == PASS_STEP
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


