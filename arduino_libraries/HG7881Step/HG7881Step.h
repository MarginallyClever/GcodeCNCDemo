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

// ensure this library description is only included once
#ifndef HG7881Step_h
#define HG7881Step_h

#define MICROSTEPS 16  // 8 or 16
#define FORWARD 1

// library interface description
class HG7881Step {
  public:
    // constructors:
    HG7881Step(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4);

    // mover method:
    void onestep(int dir);

	// release all coils
	void release();

  private:    
    // motor pin numbers:
    int pinA_IA;
    int pinA_IB;
    int pinB_IA;
    int pinB_IB;
};

#endif

