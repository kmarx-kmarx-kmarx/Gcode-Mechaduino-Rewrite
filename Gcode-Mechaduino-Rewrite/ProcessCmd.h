/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for ProcessCmd.cpp.
  -----------------------------------------------------------------------------
*/

#ifndef __PROCESSCOM_H__
#define __PROCESSCOM_H__

  void process_cmd(char command[]);

  // Definitions
  #define NO_CMD     -9000
  #define NOT_FOUND  -8000
  // More GCode command defines
  #define HOME           28
  #define RAPID_MOV      0
  #define LINEAR_MOV     1
  #define CHANGE_UNIT_IN 20
  #define CHANGE_UNIT_MM 21
  #define DWELL          4
  #define SET_HOME       92
  #define SET_ABS        90
  #define SET_REL        91
  // M code defines
  #define DEBUG          111

  // Misc. constants
  #define CHECK_TIME_US  200

#endif