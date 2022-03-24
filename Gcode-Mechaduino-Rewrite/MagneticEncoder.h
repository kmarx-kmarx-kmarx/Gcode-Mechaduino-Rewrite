/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for interacting with the magnetic rotary encoder.
  -----------------------------------------------------------------------------
*/
#include <stdint.h>
// We want to define this only once
#ifndef __MAGNET_H__
#define __MAGNET_H__
  // Function declarations
  void encoder_setup();
  uint16_t encoder_read();

  // macro for quickly setting the CS pin (only works if the CS pin is A2)
  #define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
  #define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09) 
  
  // Misc. constants
  #define SPI_SPEED  10000000        // CLK pin frequency
  #define SPI_PAUSE  10              // Wait this many ms for SPI to finish initializing
  #define CS_PIN     A2              // The CS pin number - references Arduino macro
  #define COUNT_PER_REV  1<<14       // From encoder datasheet, 2^14
  
#endif