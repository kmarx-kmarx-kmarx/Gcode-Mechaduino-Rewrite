/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for interacting with the magnetic rotary encoder.
  -----------------------------------------------------------------------------
*/

// We want to define this only once
#ifndef __MAGNET_H__
#define __MAGNET_H__
  // Function declarations
  void encoder_setup();
  int encoder_read();


  // Definitions:
  #define SPI_SPEED 10000000
  #define SPI_PAUSE 10
  // For quickly setting the CS pin
  #define CHIPSELECT_HIGH() (REG_PORT_OUTSET1 = PORT_PB09)
  #define CHIPSELECT_LOW() (REG_PORT_OUTCLR1 = PORT_PB09) 

#endif