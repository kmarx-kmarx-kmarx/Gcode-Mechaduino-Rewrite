/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for the timer interrupts.
  -----------------------------------------------------------------------------
*/

// We want to define this only once
#ifndef __TimeControl_H__
#define __TimeControl_H__
  // Wait until the registers are synchronized
  #define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

  void setup_TCInterrupts();
  void enable_TCInterrupts();
  void disable_TCInterrupts();

  // Initialize flags and define the bits
  extern volatile int flags;
  extern volatile char mode;

  // Initialize other control global variables
  extern volatile long U;  //control effort (abs)
  extern volatile long u;  //control effort (not abs)
  extern volatile long r;  //setpoint
  extern volatile long y;  // measured angle
  extern volatile long v;  // estimated velocity (velocity loop)
  extern volatile long yw; // wrapped measured angle
  extern volatile long yw_1; // previous wrapped measured angle
  extern volatile long e;  // e = r-y (error)
  extern volatile long e_1;  // previous error
  extern const int CMD_READY;

#endif