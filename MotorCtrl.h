/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for setting up and controlling the motor.
  -----------------------------------------------------------------------------
*/

// We want to define this only once
#ifndef __MOTOR_H__
#define __MOTOR_H__
  // Function declarations
  void setup_pins();            // Set up pins for motor control
  void stop_motor();            // Park the motor
  int sine_lookup(int angle);   // Calculate sine from lookup table
  int one_step(bool dir, int step); // Move the motor one step in the specified direction
  void output(int theta, int effort); // Move the motor to the set angle

  // Macros for quickly writing to GPIO
  #define IN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
  #define IN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
  #define IN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
  #define IN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
  #define IN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
  #define IN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
  #define IN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
  #define IN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
  #define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
  #define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)

extern const int STEP_PER_REV;
extern const int VALS_PER_REV;
extern const int VAL_PER_STEP;
extern const int QUARTER_TURN;
extern const int MICROSTEPS;          // 1/32 microstepping
extern const int MOTOR_SETTLE;        // ms it takes for the motor to settle
extern const bool CCW;
extern const bool CW;
// Create constant parameters for controlling the motor
extern const int   uMAX; // Set the maximum "effort"; (225 duty/3.3 volts) * (iMax amps * rSense ohms * 10) gives us a way to convert from target current to PWM duty cycle
#endif