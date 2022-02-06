/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains the functions necessary to initialize the time control interrupts.
        setup_TCInterrupts()   Initialize the Timer Compare interrupts. We have two; one for the main control loop and another for constant acceleration control
        enable_TCInterrupts()  Enable both interrupts
        disable_TCInterrupts() Disable both interrupts
        TC5_Handler()          Timer-control-interrupt function responsible for motor operation
        

  SHARED VARIABLES: None

  GLOBAL VARIABLES:
        Various built-in interrupt control registers are set; no new global variables are used in this file.

  INCLUDES:
        TimeControlInt.h: Contains macros and function declarations
        MotorCtrl.h: Functions and macros for controlling the motor
        MagneticEncoder.h: Read from the encoder
        Arduino.h: Board utilities
  -----------------------------------------------------------------------------
*/


#include "TimeControlInt.h"
#include "MotorCtrl.h"
#include "MagneticEncoder.h"
#include <Arduino.h>
// Define constants
// Frequency and Period for the control loop
const float BASECLK        = 48000000;
const float CTRL_LOOP_HZ   = 6500;
const int CTRL_LOOP_PERIOD = BASECLK / CTRL_LOOP_HZ;
// Frequency and Period for the command loop
const float CMD_LOOP_HZ    = 500;
const int CMD_LOOP_PERIOD  = BASECLK / CMD_LOOP_HZ;
// PID loop parameters; 
const int FIXED_PT_SCALE = 10000; // Increasing this value increases precision but brings us closer to overflowing
const int PA = VAL_PER_STEP;      // Target position is 1 step away from current position. Can increase for faster repsonse
const int ITERM_MAX = (200/360)*VALS_PER_REV; // Integral term winding limit

// Parameters for positioning mode:
// Integral term parameters
const int pKi = 0.15 * FIXED_PT_SCALE;        // integral error multiplier for abs. positioning mode
// Differential term parameters
const int pLPF = 30;                           // break frequency in Hz, IIR parameter
const int pLPFa = FIXED_PT_SCALE * exp(-2*PI*pLPF/CTRL_LOOP_HZ); // scaling factor
const int pLPFb = FIXED_PT_SCALE-pLPFa;                          // scaling factor
const int pKd   = 250 * FIXED_PT_SCALE;                          // differential error multiplier
// Proportional term parameters
const int pKp   = 15 * FIXED_PT_SCALE;         // proportional error multiplier

// Parameters for velocity mode
// Integral term parameters
const int vKi = 0.015 * FIXED_PT_SCALE;
// Differential term parameters 
const int vLPF  = 100;                                // break frequency in Hz, IIR parameter
const int vLPFa = FIXED_PT_SCALE *  exp(-2*PI*vLPF/CTRL_LOOP_HZ); // scaling factor
const int vLPFb = FIXED_PT_SCALE-pLPFa;                                             // scaling factor
const int vKd   = 0.02 * FIXED_PT_SCALE;        // differential error multiplier
// Proportional term parameters
const int vKp   = 0.25 * FIXED_PT_SCALE;         // proportional error multiplier

// Flag constants
extern const int CMD_READY = 0;

// Define global variables
volatile int flags = 0;
volatile char mode = '0';
volatile long U = 0;     //control effort (abs)
volatile long u = 0;     //real control effort (not abs)
volatile long r = 0;     //setpoint
volatile long y = 0;     // measured angle
volatile long v = 0;     // estimated velocity  (velocity loop)
volatile long yw = 0;    // "wrapped" angle (not limited to 0-360)
volatile long yw_1 = 0;
volatile long e = 0;     // e = r-y (error)
volatile long e_1 = 0;   //previous error
// Define shared variables
volatile long y_1 = 0;     // previous measured angle
volatile long wrap_count = 0; // Number of full revolutions
volatile long ITerm = 0;
volatile long DTerm = 0;
/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setup_TCInterrupts() initializes two timer compare interrupts, timer comare 4 and 5.

  OPERATION:   We first enable timer compare interrupts using timers 4 and 5; we then set up the interrupt for TC5 and then set up TC4. The period of the interrupts is defined in Parameters.h

  ARGUMENTS/RETURNS: none

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES / SHARED VARIABLES: none 

  GLOBAL VARIABLES: Built-in time counter compare registers

  DEPENDENCIES:
        TimeControlInt.h:  contains macro definitions for CTRL_LOOP_PERIOD and CMD_LOOP_PERIOD.
  -----------------------------------------------------------------------------
*/
void setup_TCInterrupts() {
  // Configure the closed-loop interrupt as well as the control interrupt
  // Enable GCLK for TC4 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
  while (GCLK->STATUS.bit.SYNCBUSY);

  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.CC[0].reg = CTRL_LOOP_PERIOD;
  WAIT_TC16_REGS_SYNC(TC5)

  TC5->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC5->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC5->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0

  // Now we also need TC4 for handling the GCode commands
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TCx
  WAIT_TC16_REGS_SYNC(TC4)                      // wait for sync
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;   // Set Timer counter Mode to 16 bits
  WAIT_TC16_REGS_SYNC(TC4)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ; // Set TC as normal Normal Frq
  WAIT_TC16_REGS_SYNC(TC4)
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1;   // Set perscaler; 46.875kHz
  WAIT_TC16_REGS_SYNC(TC4)
  TC4->COUNT16.CC[0].reg = CMD_LOOP_PERIOD; 
  WAIT_TC16_REGS_SYNC(TC4)
  TC4->COUNT16.INTENSET.reg = 0;              // disable all interrupts
  TC4->COUNT16.INTENSET.bit.OVF = 1;          // enable overfollow
  TC4->COUNT16.INTENSET.bit.MC0 = 1;         // enable compare match to CC0
  NVIC_SetPriority(TC4_IRQn, 1);              //Set interrupt priority
  NVIC_EnableIRQ(TC4_IRQn);                  // Enable InterruptVector
  NVIC_SetPriority(TC5_IRQn, 0);              //Set interrupt priority
  // Enable InterruptVector
  NVIC_EnableIRQ(TC5_IRQn);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: enable_TCInterrupts() enables the two timer compare interrupts, timer comare 4 and 5.

  OPERATION:   This sets the interrupt enable bits in the control registers and waits for the counter to stop being busy before continuing on.

  ARGUMENTS/RETURNS: none

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES / SHARED VARIABLES: none 

  GLOBAL VARIABLES: Built-in time counter compare registers

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
void enable_TCInterrupts() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;    //Enable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      //wait for sync
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  WAIT_TC16_REGS_SYNC(TC4)

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: disable_TCInterrupts() disablesenables the two timer compare interrupts, timer comare 4 and 5.

  OPERATION:   This clears the interrupt enable bits in the control registers and waits for the counter to stop being busy before continuing on.

  ARGUMENTS/RETURNS: none

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES / SHARED VARIABLES: none 

  GLOBAL VARIABLES: Built-in time counter compare registers

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
void disable_TCInterrupts() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;   // Disable TC5
  WAIT_TC16_REGS_SYNC(TC5)                      // wait for sync
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  WAIT_TC16_REGS_SYNC(TC4)

  return;
}

void TC5_Handler() {// gets called with CTRL_LOOP_HZ frequency

  unsigned long current_time = micros(); 
  if (TC5->COUNT16.INTFLAG.bit.OVF == 1) {        // A counter overflow caused the interrupt
    y = sine_lookup(encoder_read());                    //read encoder and lookup corrected angle in calibration lookup table

    if ((y - y_1) < -(VALS_PER_REV)/2) wrap_count += 1;      //Check if we've rotated more than a full revolution (have we "wrapped" around from 359 degrees to 0 or ffrom 0 to 359?)
    else if ((y - y_1) > (VALS_PER_REV)/2) wrap_count -= 1;

    y_1 = y;  //copy current value of y to previous value (y_1) for next control cycle before PA angle added

    yw = (y + (VALS_PER_REV * wrap_count));              //yw is the wrapped angle (can exceed one revolution)

    switch (mode) {
      case 'x':         // position control
        e = (r - yw);   // Calculate error from target (r)

        ITerm += (pKi * e)/FIXED_PT_SCALE; // Add it to integral error

        // Prevent ITerm from overflowing
        ITerm = min(max(ITerm, -ITERM_MAX), ITERM_MAX);

        // Calculate the differential error using IIR
        DTerm = (pLPFa * DTerm)/FIXED_PT_SCALE  -  (pLPFb * pKd * (yw - yw_1))/(FIXED_PT_SCALE * FIXED_PT_SCALE);

        // Calculate proportional error and sum the errors
        u = (pKp * e)/FIXED_PT_SCALE + ITerm + DTerm;
        break;

      case 'v':         // velocity control
        // Calculate v, weighted sum of previous velocity and calculated velocity
        v = (vLPFa * v)/FIXED_PT_SCALE +  vLPFb * (yw - yw_1)/FIXED_PT_SCALE; //filtered velocity called "DTerm" because it is similar to derivative action in position loop

        e = (r - v);   //error in degrees per rpm (sample frequency in Hz * (60 seconds/min) / (360 degrees/rev) )

        ITerm += (vKi * e)/FIXED_PT_SCALE;                 
        // Prevent ITerm from overflowing
        ITerm = min(max(ITerm, -ITERM_MAX), ITERM_MAX);

        u = (vKp * e)/FIXED_PT_SCALE + ITerm - (vKd * (e - e_1))/FIXED_PT_SCALE;
        break;

      default:
        u = 0;
        break;
    }


    if (u > 0)          //Depending on direction we want to apply torque, add or subtract a phase angle of PA for max effective torque.  PA should be equal to one full step angle: if the excitation angle is the same as the current position, we would not move!
    { //You can experiment with "Phase Advance" by increasing PA when operating at high speeds
      y += PA;          //update phase excitation angle
      if (u > uMAX)     // limit control effort
        u = uMAX;       //saturation limits max current command
    }
    else
    {
      y -= PA;          //update phase excitation angle
      if (u < -uMAX)    // limit control effort
        u = -uMAX;      //saturation limits max current command
    }

    // Bound u between +- uMAX
    u = min(max(u, -uMAX), uMAX);

    U = abs(u);

    //if (abs(e) < 0.1) ledPin_HIGH();    // turn on LED if error is less than 0.1
    //else ledPin_LOW();                  //digitalWrite(ledPin, LOW);

    output(-y, U);    // update phase currents
  }

  //copy current values to previous values for next control cycle
  e_1 = e;
  yw_1 = yw;
  
  TC5->COUNT16.INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
}
