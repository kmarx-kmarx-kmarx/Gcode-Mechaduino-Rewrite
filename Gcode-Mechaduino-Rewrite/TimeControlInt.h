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

  // Initialize global variables for controlling motor operation
  extern volatile int32_t flags; // Flags, misc. instruction commands
  extern volatile char mode;     // Mode: either positioning(x), velocity (v), or park (neither v nor x)
  extern volatile int32_t u_f;   // IIR-filtered "effort". Filter cutoff freq defined by fLPF
  extern volatile int32_t u_f_1; // The previous value of u_f
  extern volatile int32_t set;   // setpoint, the target position.
  extern volatile int32_t ctrl_start;   // start point of some operation
  extern volatile int32_t ctrl_end;     // end point of some operation  

  // Initialize precomputed velocity table
  #define N_ELEM  256
  extern volatile int32_t precalculated_v[N_ELEM];
  // Define flags bit map
  #define COMMAND_SHIFT  (31-3) // Reserve 3 bits at the top for 2^3 different commands
  #define COMMAND_MASK   (0b111 << COMMAND_SHIFT) // Used to filter out the other bits
  #define NULL_COMMAND    0b000 // Nothing
  #define STOP_COMMAND    0b111 // Halt
  #define MOVE_COMMAND    0b001 // Rapid move
  #define LINEAR_COMMAND  0b010 // Linear move
  #define DWELL_COMMAND   0b011 // Dwell
  #define CMD_READY      0   // Set this bit when there is a command in the command buffer ready to execute
  #define CMD_INVALID    1   // Set this bit if the command was read properly but it is invalid for some other reason
  #define DEBUG_MODE     2   // Set if debugging, clear if you don't want debug messages
  #define POS_ABSOLUTE   3   // Set if absolute positioning, clear if relative
  #define UNITS_MM       4   // Set if in units millimeters, clear if inches
  #define IN_PROGRESS    5   // Set if a command is currently in progress

  // Constants
  #define DIST_THRESH   4*MICROSTEPS // If we are within 4 microsteps of our target, we have hit our target
  

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
  
  // Define constants
  // Frequency and Period for the control loop
  #define BASECLK              48000000
  #define TARG_CTRL_LOOP_HZ    6500
  #define CTRL_LOOP_PERIOD     (uint32_t)(BASECLK / TARG_CTRL_LOOP_HZ)
  // Frequency and Period for the command loop
  #define CMD_LOOP_HZ          500
  #define CMD_LOOP_PERIOD      (uint32_t)(BASECLK / (2*CMD_LOOP_HZ)) // divide by 2 to account for clock presale
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
  
  // Filter term parameters
  #define fLPF     0.5 // Break frequency in Hz
  #define fLFPa    (FIXED_PT_SCALE *  exp(-2*PI*vLPF/TARG_CTRL_LOOP_HZ))
  #define fLPFb    FIXED_PT_SCALE - vLPFa

#endif
