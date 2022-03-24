/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for the timer interrupts.
  -----------------------------------------------------------------------------
*/
#include <stdint.h>
#include "MagneticEncoder.h"
#include "MotorCtrl.h"
#include <math.h>

// We want to define this only once
#ifndef __TimeControl_H__
#define __TimeControl_H__
  // Wait until the registers are synchronized
  #define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

  void setup_TCInterrupts();
  void enable_TCInterrupts();
  void disable_TCInterrupts();

  // Initialize flags and define the bits
  extern volatile int32_t flags;
  extern volatile char mode;

  // Initialize other control global variables
  extern volatile uint32_t U;  //control effort (abs)
  extern volatile int32_t u;  //control effort (not abs)
  extern volatile int32_t r;  //setpoint
  extern volatile int32_t y;  // measured angle
  extern volatile int32_t v;  // estimated velocity (velocity loop)
  extern volatile int32_t yw; // wrapped measured angle
  extern volatile int32_t yw_1; // previous wrapped measured angle
  extern volatile int32_t e;  // e = r-y (error)
  extern volatile int32_t e_1;  // previous error
  #define CMD_READY 0
  
  // Define constants
  // Frequency and Period for the control loop
  #define BASECLK              48000000
  #define TARG_CTRL_LOOP_HZ    6500
  #define CTRL_LOOP_PERIOD     (uint32_t)(BASECLK / TARG_CTRL_LOOP_HZ)
  // Frequency and Period for the command loop
  #define CMD_LOOP_HZ          500
  #define CMD_LOOP_PERIOD      (uint32_t)(BASECLK / CMD_LOOP_HZ)
  // PID loop parameters 
  #define PA                   VAL_PER_STEP       // Target position is 1 step away from current position. Can increase for faster repsonse
  #define ITERM_MAX            (200 * VALS_PER_REV / 360) // Integral term winding limit

  // Parameters for positioning mode:
  // Integral term parameters
  #define pKi          (int32_t)(0.15 * FIXED_PT_SCALE)        // integral error multiplier for abs. positioning mode
  // Differential term parameters
  #define pLPF      30                                            // break frequency in Hz, IIR parameter
  #define pLPFa     (FIXED_PT_SCALE * exp(-2*PI*pLPF/TARG_CTRL_LOOP_HZ)) // scaling factor
  #define pLPFb     FIXED_PT_SCALE-pLPFa                           // scaling factor
  #define pKd       250 * FIXED_PT_SCALE                           // differential error multiplier
  // Proportional term parameters
  #define pKp       15 * FIXED_PT_SCALE          // proportional error multiplier

  // Parameters for velocity mode
  // Integral term parameters
  #define vKi       0.015 * FIXED_PT_SCALE        // integral error multiplier for velocity mode
  // Differential term parameters 
  #define vLPF      100                                 // break frequency in Hz, IIR parameter
  #define vLPFa     (FIXED_PT_SCALE *  exp(-2*PI*vLPF/TARG_CTRL_LOOP_HZ)) // scaling factor
  #define vLPFb     FIXED_PT_SCALE-pLPFa                                             // scaling factor
  #define vKd       0.02 * FIXED_PT_SCALE        // differential error multiplier
  // Proportional term parameters
  #define vKp       0.25 * FIXED_PT_SCALE         // proportional error multiplier

#endif