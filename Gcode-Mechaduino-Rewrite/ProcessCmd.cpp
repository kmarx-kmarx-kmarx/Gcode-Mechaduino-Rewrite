/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains the functions necessary translate serial commands to actions for the motor.
        

  SHARED VARIABLES: float feedrate: the speed of the movement in inches or mm per minute

  GLOBAL VARIABLES: This file access the global control variables in TimeControlInt.h to set the motor movement

  INCLUDES:
        TimeControlInt.h: Access motor control global variables
        SerialCommand.h:  Command parameters
        MotorCtrl.h:      Motor control macros
        Arduino.h: Has macros and definitions for the board
  -----------------------------------------------------------------------------
*/

#include "ProcessCmd.h"
#include "SerialCommand.h"
#include "TimeControlInt.h"
#include "MotorCtrl.h"
#include "Calibrate.h"
#include <Arduino.h>

volatile float feedrate = 0;

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: search_code(char key, char instruction[]) searches the instruction string for key character and returns the floating point number after it.

  OPERATION:   G-code commands are of the form <key letter> <space> <floating point number>. We iterate through the instruction to find the key character and then put the following characters into a character array. We then convert the character array into a float.

  ARGUMENTS: char command[]; the pointer to the command buffer array.
             char key; the key character we are looking for

  RETURNS: float code, the code after the character.

  LOCAL VARIABLES: 
        temp[CODE_LEN]: character array holding the code

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        SerialCommand.h: Contains macros for timing.
  -----------------------------------------------------------------------------
*/ 
float search_code(char key, char instruction[])
{
  // Temp char string for holding the code
  // Codes are always CODE_LEN or fewer characters long
  char temp[CODE_LEN] = "";

  // Search through the string
  for (int i=0; i < COMMAND_SIZE; i++){
    // When the key is found, search through the following characters
    if (instruction[i] == key){
      i++;      
      int k = 0;
      while (i < COMMAND_SIZE && k < CODE_LEN){
        // If the character is a space, we are done reading. We can expect a space because SerialCommand.cpp ensures the command is formatted properly
        if (instruction[i] == ' ')
          break;
        // Otherwise, add it to temp
        temp[k] = instruction[i];
        i++;
        k++;
      }
      // Return the string turned into a float
      if(temp == ""){
        // Return NO_CMD if there is no command
        return NO_CMD;
      }
      return String(temp).toFloat();
    }
  }
  // Othewise, say it was not found.
  return NOT_FOUND;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: process_m(char instruction[]) executes the m code. M codes typically change the settings on the device. M112, e-stop, is handled in main

  OPERATION:   We read the integer after 'M' and go through a lookup table to see what to do. Then, the command is executed.

  ARGUMENTS: char command[]; the pointer to the command buffer array.

  RETURNS: None

  LOCAL VARIABLES: 
        code_f, code_i: The value we searched for


  SHARED VARIABLES: None

  GLOBAL VARIABLES: 
        flags: we change the global flags to set parameters

  DEPENDENCIES:
        ProcessCmd.h: Contains macros.
        TimeControlInt.h: For accessing flags
  -----------------------------------------------------------------------------
*/ 
void process_m(char instruction[]){
  float code_f;          // For managing the readings
  code_f = search_code('M', instruction);
  int32_t code_i = code_f; // Convert from float to int so we can use it in the switch statement

  switch(code_i){
    case NO_CMD:
      break;

    case DEBUG:
      code_f = search_code('S', instruction);
      if(code_f == NOT_FOUND){
        return;
      }
      else if(code_f == 0){
        // If S0, turn off debugging mode
        flags &= ~(1<<DEBUG_MODE);
      }
      else{
        // Otherwise, we are debugging
        flags |= (1<<DEBUG_MODE);
        // Print out our calibration table
        for(uint32_t i = 0; i < 16384; i++){
          SerialUSB.print(i);
          SerialUSB.print(", ");
          SerialUSB.println(lookup[i]);
        }
      }
      break;
    default:
      break;
    }
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: interpolate_pos(float target) checks which units we are using, mm or inches, the converts the target in that unit to a number of values.

  OPERATION:   We rescale target by a factor of VALS_PER_REV/MM_PER_ROT if we are in mm mode or VALS_PER_REV/IN_PER_ROT if we are using inches. We then convert to a int32_t.

  ARGUMENTS:  float target: target position in units mm or in

  RETURNS:    int32_t result: target position in units VALS_PER_REV

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: 
        flags: check which unit we are using

  DEPENDENCIES:
        TimeControlInt.cpp for the flags

  -----------------------------------------------------------------------------
*/
int32_t interpolate_pos(float target){
  // Convert the target position in millimeters to the target degree rotation
  int32_t result;
  if(flags & 1<<UNITS_MM){
    result = (int32_t)(target * ((float)VALS_PER_REV/((float)MM_PER_ROT)));
  }
  else{
    result = (int32_t)(target * ((float)VALS_PER_REV/((float)IN_PER_ROT)));
  }
  return result;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: bound_pos(int32_t target) takes a position in units VALS_PER_REV and bounds it between xmin and xmax

  OPERATION:   We evaluate min(max(target, xmin), xmax)

  ARGUMENTS:  int32_t target: target position in units mm or in

  RETURNS:    int32_t result: target position in units VALS_PER_REV

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: xmin, xmax: the minimum and maximum x values from calibration

  DEPENDENCIES:
        Calibrate.h: for xmin, xmax.

  -----------------------------------------------------------------------------
*/
int32_t bound_pos(int32_t target){
  // Bound the target position between xmin and xmax
  int32_t result;

  result = min(max(target, xmin), xmax);

  return result;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: process_g(char instruction[]) executes the g code. This involves setting flags which either change the device settings or set the motor to move.

  OPERATION:   We read the integer after 'G' and go through a lookup table to see what to do. Then, the command is executed.

  ARGUMENTS: char command[]; the pointer to the command buffer array.

  RETURNS: None

  LOCAL VARIABLES: 
        code_f, code_i: The value we searched for


  SHARED VARIABLES: None

  GLOBAL VARIABLES: 
        flags: we change the global flags to set parameters
        feedrate: 

  DEPENDENCIES:
        ProcessCmd.h: Contains macros.
        TimeControlInt.h: Contains global vairables 

  -----------------------------------------------------------------------------
*/ 
void process_g(char instruction[]){
  float code_f, code_f2;          // For managing the readings
  code_f = search_code('G', instruction);
  int code_i = code_f; // Convert from float to int so we can use it in the switch statement
  int32_t target;

  switch(code_i){
    case NO_CMD:
      break;

    case RAPID_MOV:
      // Move to target point at maximum feedrate
      code_f = search_code('X', instruction);
      if(code_f == NOT_FOUND){
        return;
      }
      // Convert the reading from 
      target = interpolate_pos(code_f);

      if(flags & 1<<POS_ABSOLUTE){
        // If doing absolute positioning, use home as reference point
        target = xmin - target;
      }
      else{
        // Else, add on to current position
        target = yw - target;
      }
      
      // wait until we are no longer IN_PROGRESS before starting the new command
      while(flags & 1<<IN_PROGRESS){
        delayMicroseconds(CHECK_TIME_US);
      }
      mode = 'x';
      flags |= 1<<IN_PROGRESS;
      flags |= MOVE_COMMAND<<COMMAND_SHIFT;
      // Keep output position within boundaries
      r = bound_pos(target);
      break;
    case LINEAR_MOV:
      // First, we handle the case "G1 Fxxx" and set the feedrate to xxx without
      // doing any movement. This occurs when there is no X command.
      code_f = search_code('F', instruction);
      code_f2 = search_code('X', instruction);
      if(code_f2 == NOT_FOUND){
        // If no x position is given, update the feedrate and return.
        if(code_f != NOT_FOUND){
          feedrate = code_f;
        }
        return;
      }
      // Otherwise, we have something like "G1 Xxxx" or "G1 Xxxx Fxxx"
      // Hop into a function to do this
      //linear_move_action(reading_x, reading_misc);
      break;

    case SET_ABS:
      // Set absolute positioning
      flags |= 1<<POS_ABSOLUTE;
      break;

    case SET_REL:  
      // Set relative positioning
      flags &= ~(1<<POS_ABSOLUTE);
      break;

    case CHANGE_UNIT_IN:
      // Change units to inches
      flags &= ~(1<<UNITS_MM);
      break;

    case CHANGE_UNIT_MM:
      // Change units to mm
      flags |= 1<<UNITS_MM;
      break;

    case SET_HOME:
      // Set the current location to home
      xmin = yw;
      break;  
    case HOME:
      // Check if we want to do a full calibration - uses "A" code
      if(search_code('A', instruction) != NOT_FOUND){
        xmin = 0;
        xmax = 0;
        calibrate();
        calib_home();
      }
      if((search_code('X', instruction) == NOT_FOUND) || (xmin != 0 || xmax != 0)){
        // Do the full calibration
        if(U > UNLOADED_EFFORT_LIM){
          return; // Can't home if effort's too high
        }
        // Run position calibration and home
        calib_home();        
      }
      // Then, if calibrated, move it home at full speed.
      mode = 'x';
      r = xmin;
      break;
    case DWELL:
      code_f = search_code('P', instruction);
      if(code_f != NOT_FOUND){
        // Set global variables with timing info
        // and the command that we are doing
        target = code_f;
        //data1 = millis();
        flags |= 1<<IN_PROGRESS;
        flags |= DWELL_COMMAND<<COMMAND_SHIFT;
      }
      else{
        ;
      }
      break;
    default:
      break;
  }
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: process_cmd(char command[COMMAND_SIZE]) reads in the command and performs the operation.

  OPERATION:   From SerialCommand.cpp, we know the first character is either M or G. We call the appropriate function based on the first character.

  ARGUMENTS: char command - character array containing the command

  RETURNS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: Flags; we set a bit to indicate whether the command was read successfully

  DEPENDENCIES: None
        
  -----------------------------------------------------------------------------
*/ 
void process_cmd(char command[]){
  char cmdType = command[0];
  switch(cmdType){
    case 'M': // M codes and G codes have to be processed separately
      process_m(command);
      break;
    case 'G':
      process_g(command);
      break;
    default:
      flags |= (1 << CMD_INVALID);
      break;
  }
  // We processed the command so clear the command ready flag
  flags &= ~(1<<CMD_READY);
  return;
}
