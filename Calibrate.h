/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for the calibration and flash storage routine.
  -----------------------------------------------------------------------------
*/

#ifndef __CALIB_H__
#define __CALIB_H__

  static void write_page();
  static void store_lookup(int lookupAngle);
  int calibrate();
  void findijStart(int readings[], int* istart, int* jstart);

  extern const int lookup[];

#endif