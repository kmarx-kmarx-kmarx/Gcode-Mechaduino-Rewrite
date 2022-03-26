/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains the functions necessary translate serial commands to actions for the motor.
        

  SHARED VARIABLES: None

  GLOBAL VARIABLES: This file access the global control variables in TimeControlInt.h to set the motor movement

  INCLUDES:
        TimeControlInt.h: Access motor control global variables
        SerialCommand.h:  Command parameters
        Arduino.h: Has macros and definitions for the board
  -----------------------------------------------------------------------------
*/

#include "ProcessCmd.h"
#include "SerialCommand.h"
#include "TimeControlInt.h"
#include <Arduino.h>

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
  DESCRIPTION: process_cmd(char command[COMMAND_SIZE]) reads in the command and performs the operation.

  OPERATION:   We first use string manipulation to process the command. We then set global flags and parameters for the control loop to read and process.
               In SerialComand.cpp, the string is formatted to make it easier to manipulate. The first character is either G or M; we then look up the command and execute it

  ARGUMENTS: char command - character array containing the command

  RETURNS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: There are several variables shared between this file and TimeControlInt.cpp.

  DEPENDENCIES:
        
  -----------------------------------------------------------------------------
*/ 
void process_cmd(char command[]){
  switch(command[0]){
    case 'M':
      break;
    case 'G':
      break;
  }
  return;
}

