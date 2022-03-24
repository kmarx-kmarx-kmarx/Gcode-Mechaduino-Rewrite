/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for setting up and controlling the motor.
  -----------------------------------------------------------------------------
*/
#include <stdint.h>
// We want to define this only once
#ifndef __MOTOR_H__
#define __MOTOR_H__
  // Function declarations
  void setup_pins();            // Set up pins for motor control
  void stop_motor();            // Park the motor
  int32_t sine_lookup(uint32_t angle);   // Calculate sine from lookup table
  int32_t one_step(bool dir, int32_t step); // Move the motor one step in the specified direction
  void output(int32_t theta, int32_t effort); // Move the motor to the set angle

  // Macros for quickly writing to GPIO
  #define PIN_1_HIGH() (REG_PORT_OUTSET0 = PORT_PA06)
  #define PIN_1_LOW() (REG_PORT_OUTCLR0 = PORT_PA06)
  #define PIN_2_HIGH() (REG_PORT_OUTSET0 = PORT_PA21)
  #define PIN_2_LOW() (REG_PORT_OUTCLR0 = PORT_PA21)
  #define PIN_3_HIGH() (REG_PORT_OUTSET0 = PORT_PA15)
  #define PIN_3_LOW() (REG_PORT_OUTCLR0 = PORT_PA15)
  #define PIN_4_HIGH() (REG_PORT_OUTSET0 = PORT_PA20)
  #define PIN_4_LOW() (REG_PORT_OUTCLR0 = PORT_PA20)
  #define ledPin_HIGH() (REG_PORT_OUTSET0 = PORT_PA17)
  #define ledPin_LOW() (REG_PORT_OUTCLR0 = PORT_PA17)

  // Define global constants
  #define MOTOR_SETTLE     10                 // ms it takes for the motor to settle
  #define STEP_PER_REV     200                // Motor parameter: number of steps
  #define MICROSTEPS       32                 // 1/32 microstepping; we can set the motor to STEPS_PER_REV * QUARTER_TURN possible positions
  #define VALS_PER_REV     (4 * STEP_PER_REV * MICROSTEPS) // Number of "degrees" per full revolution. We want it to be a large number so we can do math as integers without losing precision.
                                                    // We want to make sure we can hit all of the possible values and still have enough resolution for error correction
                                                    // We have 4 bytes (32 bits); 1 revolution takes 15 bits, 1 bit indicates sign, and 8 bits are reserved for fixed point operations. This gives us
                                                    // 256 full revolutions before overflow; for poseidon, we can rotate 100 times before hitting an end stop so this is OK; please tune as needed to meet your needs

  #define FIXED_PT_SCALE   (1<<8) // Increasing this value increases precision but brings us closer to overflowing
  #define VAL_PER_STEP     (int)(VALS_PER_REV/STEP_PER_REV) // Number of "degrees" per step; we have chosen VALS_PER_REV to make this a whole number
  #define QUARTER_TURN     (VALS_PER_REV/4)                 // Number of "degrees" in a quarter of a full rotation
  #define CCW              true                             // Functions as macros for setting motor direction
  #define CW               false
  // Create constant parameters for controlling the motor
  #define iMAX    1.5             // While the A4954 driver is rated for 2.0 Amp peak currents, it cannot handle these currents continuously.  Depending on how you operate the Mechaduino, you may be able to safely raise this value; refer to the A4954 datasheet for more info
  #define rSense  0.150           // Sense resistor resistance in ohms
  #define uMAX    (int32_t)((255/3.3)*(iMAX*10*rSense)) // Set the maximum "effort"; (225 duty/3.3 volts) * (iMax amps * rSense ohms * 10) gives us a way to convert from target current to PWM duty cycle

  // Pin numbers
  #define PIN_4    6   // Pin 1,2,3,4 are motor control pins
  #define PIN_3    5
  #define VREF_2   4   // VREF pins set voltage reference over PWM
  #define VREF_1   9
  #define PIN_2    7
  #define PIN_1    8
  #define ledPin  13  // Built-in LED

#endif