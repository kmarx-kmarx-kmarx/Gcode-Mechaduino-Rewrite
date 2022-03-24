/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for the calibration and flash storage routine.
  -----------------------------------------------------------------------------
*/
#include <stdint.h>

#ifndef __CALIB_H__
#define __CALIB_H__
  int calibrate();

  extern const uint32_t lookup[];

  // Misc. macro constants
  #define N_AVG  16     // Average this many values 
  #define PAGE_SIZE 256 // Bytes per page of flash
  #define INTS_PER_PAGE  PAGE_SIZE/sizeof(int32_t)
  #define BAD_WIRE -2
  #define CALIBRATION_SUCCESS 0


#endif