/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for the calibration and flash storage routine.
  -----------------------------------------------------------------------------
*/
#include <stdint.h>

#ifndef __CALIB_H__
#define __CALIB_H__
  int32_t calibrate();
  void calib_home();

  extern int32_t xmin;
  extern int32_t xmax;

  extern const uint32_t lookup[];

  // Misc. macro constants
  #define N_AVG  16     // Average this many values 
  #define PAGE_SIZE 256 // Bytes per page of flash
  #define INTS_PER_PAGE  PAGE_SIZE/sizeof(int32_t)
  #define BAD_WIRE -2
  #define CALIBRATION_SUCCESS 0
  #define CALIBRATION_FAIL    -1

  // Effort limit for detecting end stops
  // These values are for when there is no load on the motor
  // beyond poseidon itself. You will need to tune these values.
  #define UNLOADED_EFFORT_LIM     (35 * VALS_PER_REV)
  #define UNLOADED_EFFORT_NOM     (25 * VALS_PER_REV)
  #define EFFORT_SLOPE_THRESH     (1 * VALS_PER_REV)
  // Speed at which to do homing, revolutions per minute
  #define HOMING_SPEED   (60 * VALS_PER_REV)

  #define IDLE_US        1000 // idle time in ms

#endif
