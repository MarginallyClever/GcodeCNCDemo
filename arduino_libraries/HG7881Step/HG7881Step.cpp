/*
  * HG7881 Stepper driver for GCodeCNCDemo
  
  * HG7881Step.h
  * 1/8 step for HG7881
  * by Luqman Sahidin, 2015
  
  The sequence of control signals for 4 control wires is as follows:

  Step C0 C1 C2 C3
     1  1  0  1  0	\
     2	1  1  1  0	|
     3  0  1  1  0	/
     4	0  1  1  1	-
     5  0  1  0  1	\
     6	1  1  0  1	|
     7  1  0  0  1	/
     8	1  0  1  1	-

*/


#include "Arduino.h"
#include "HG7881Step.h"


/*
 *   constructor for 4-pin version
 *   Sets which wires should control the motor.
 */

//--Full Step--//
//int coil_seq[4][4] = {{1,0,1,0},{0,1,1,0},{0,1,0,1},{1,0,0,1}};

//--Half Step--//
int coil_seq[8][4] = {{1,0,1,0},{1,1,1,0},{0,1,1,0},{0,1,1,1},{0,1,0,1},{1,1,0,1},{1,0,0,1},{1,0,1,1}};

int thisStep;
int currentstep;

HG7881Step::HG7881Step(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4)
{
  currentstep = 0;
  
  // Arduino pins for the motor control connection:
  this->pinA_IA = motor_pin_1;
  this->pinA_IB = motor_pin_2;
  this->pinB_IA = motor_pin_3;
  this->pinB_IB = motor_pin_4;

  // setup the pins on the microcontroller:
  pinMode(this->pinA_IA, OUTPUT);
  pinMode(this->pinA_IB, OUTPUT);
  pinMode(this->pinB_IA, OUTPUT);
  pinMode(this->pinB_IB, OUTPUT);
  
}

/* 
 * release all coils
 * HG7881 needs this to not drive the motor unnecessarily
 * to avoid produce more heat on IC's
 */
 void HG7881Step::release(){
	 digitalWrite(this->pinA_IA, LOW);
	 digitalWrite(this->pinA_IB, LOW);
	 digitalWrite(this->pinB_IA, LOW);
	 digitalWrite(this->pinB_IB, LOW);
 }

/*
 * Moves the motor forward or backwards.
 * If the number is negative, 
   the motor moves in the reverse direction.
 */
void HG7881Step::onestep(int dir)
{
	if((currentstep/(MICROSTEPS/2)) % 2) { // we're at an odd step, weird
		if(dir == FORWARD) currentstep += MICROSTEPS/2;
		else               currentstep -= MICROSTEPS/2;
	} else {           // go to the next even step
		if(dir == FORWARD) currentstep += MICROSTEPS;
		else               currentstep -= MICROSTEPS;
	}

	currentstep += MICROSTEPS*4;
	currentstep %= MICROSTEPS*4;

	thisStep = currentstep/(MICROSTEPS/2);

	/* Energize the coil by sequence */

    digitalWrite(pinA_IA, coil_seq[thisStep][0]);
    digitalWrite(pinA_IB, coil_seq[thisStep][1]);
    digitalWrite(pinB_IA, coil_seq[thisStep][2]);
    digitalWrite(pinB_IB, coil_seq[thisStep][3]);

}
