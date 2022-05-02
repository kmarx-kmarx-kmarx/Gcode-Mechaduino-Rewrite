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
  for (int i = 0; i < COMMAND_SIZE; i++) {
    // When the key is found, search through the following characters
    if (instruction[i] == key) {
      i++;
      int k = 0;
      while (i < COMMAND_SIZE && k < CODE_LEN) {
        // If the character is a space, we are done reading. We can expect a space because SerialCommand.cpp ensures the command is formatted properly
        if (instruction[i] == ' ')
          break;
        // Otherwise, add it to temp
        temp[k] = instruction[i];
        i++;
        k++;
      }
      // Return the string turned into a float
      if (temp == "") {
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

  OPERATION:   We read the integer after 'M' and go through a lookup switch statement to see what to do. Then, the command is executed.

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
void process_m(char instruction[]) {
  float code_f;          // For managing the readings
  code_f = search_code('M', instruction);
  int32_t code_i = code_f; // Convert from float to int so we can use it in the switch statement

  switch (code_i) {
    case NO_CMD:
      break;

    case DEBUG:
      code_f = search_code('S', instruction);
      if (code_f == NOT_FOUND) {
        return;
      }
      else if (code_f == 0) {
        // If S0, turn off debugging mode
        flags &= ~(1 << DEBUG_MODE);
      }
      else {
        // Otherwise, we are debugging
        flags |= (1 << DEBUG_MODE);
        // Print out our calibration table
        for (uint32_t i = 0; i < 16384; i++) {
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
int32_t interpolate_pos(float target) {
  // Convert the target position in millimeters to the target degree rotation
  int32_t result;
  if (flags & 1 << UNITS_MM) {
    result = (int32_t)(target * ((float)VALS_PER_REV / ((float)MM_PER_ROT)));
  }
  else {
    result = (int32_t)(target * ((float)VALS_PER_REV / ((float)IN_PER_ROT)));
  }
  return result;
}
/*
  -----------------------------------------------------------------------------
  DESCRIPTION: interpolate_vel(float target) checks which units we are using, mm or inches, the converts the target in that unit to a number of values.

  OPERATION:   We rescale target by a factor of 1/MM_PER_ROT if we are in mm mode or 1/IN_PER_ROT if we are using inches. We then convert to a int32_t.

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
int32_t interpolate_vel(float target){
  // Convert the target velocity in millimeters/min to the target rot/min
  float result;
  if(controller_flag & 1<<UNITS_MM){
    result = (int32_t)((float)target /((float)MM_PER_ROT));
  }
  else{
    result = (int32_t)((float)target /((float)IN_PER_ROT));
  }
  return abs(result);
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
int32_t bound_pos(int32_t target) {
  // Bound the target position between xmin and xmax
  int32_t result;

  result = min(max(target, xmin), xmax);

  return result;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: bound_vel(int32_t target) takes a position in units VALS_PER_REV per minute and bounds it between MIN_SPEED and MAX_SPEED
  
  OPERATION:   We bound the absolute value of the velocity and bound it between the minimum and maximum values. We then multiply it by its sign to get back the positive/negative

  ARGUMENTS:  int32_t target: target position in units mm or in

  RETURNS:    int32_t result: target position in units VALS_PER_REV per minute

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        MotorCtrl.h: for MIN_SPEED and MAX_SPEED

  -----------------------------------------------------------------------------
*/
int32_t bound_vel(int32_t target) {
  // Bound the target position between xmin and xmax
  int32_t sign = target/abs(target);

  return sign * min(max(target, MIN_SPEED), MAX_SPEED);
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: find_target converts our input setpoint to an actual position along the path.

  OPERATION:   We check if we are in absolute or relative mode. In absolute mode, we subtract from xmax, our home position, to find our new position but in relative mode we subtract from our current position to get our new position.

  ARGUMENTS:   int32_t pos, the position in degrees we want to move
               int32_t x_init, the current position in degrees

  RETURNS:     int32_t result, the setpoint of our target position

  LOCAL VARIABLES: int32_t result, for storing the result

  SHARED VARIABLES: None

  GLOBAL VARIABLES:
    xmax:        from calibrating; we need to know where home is to use it as a reference

  INPUTS/OUTPUTS: None

  DEPENDENCIES:
        TimeControlInt.h        for flag macros
        Calibrate.h:            For xmax calibration values.
  -----------------------------------------------------------------------------
*/
int32_t find_target(int32_t pos, int32_t x_init) {
  int32_t result;
  if (flags & (1 << POS_ABSOLUTE)) {
    // If doing absolute positioning, use home as reference point
    result = xmax - pos;
  }
  else {
    // Else, use current position as reference
    result = x_init - pos;
  }

  return result;
}

float calc_v(float v0, float vf, int32_t x0, int32_t xf, int32_t x){
  float sign = (xf > x0) - (xf < x0); // figure out which direction we are going
  v0 *= sign;
  vf *= sign; // point velicities in the correct direction
  float dv = vf - v0; // delta v
  float dx = xf - x0; // delta x
  float dxi = x0 - x; // negative displacement in x
  float vi = v0 + dv/2.0; // intermediate value
  float ac = (dv * dx) / vi;

  // If we acceleration is 0 and we are in bounds, return the constant velocity
  if(ac == 0 && x > x0 && x < xf){
    return v0;
  }
  // If we are out of bounds return 0
  if (sign * x > sign * xf){
    return 0.0;
  }
  
  // Calculate time as a function of position. We use all floats here for added precision
  float t_est = abs(vi/(dv*dx) * (-sign*v0 + sqrt(v0*v0 - 2*dv*vi*dxi/dx)))
    
  return v0 + ac * t_est;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: linear_move_action takes a feedrate and target position and precalucaltes a velocity-over-position lookup table. It then sets the global variables to execute the move.

  OPERATION:   We first calculate our initial and final positions. We next take our initial and final feedrates and interpolate them as functions of position. It would create less error to recalculate the velocity at every position every iteration loop but this is too slow. Precomputing a velocity at a function of distance is good enough, runs faster, and is more resistant to errors.
               For the sake of example, let N_ELEM = 5. We thus want to have a velocity for position x=x0, 1/5th of the way from x0 to xf, 2/5ths, etc. to 4/5ths. At x=x0 we set the feedrate to our initial feedrate and at 4/5ths of the way through we set the feedrate to our final feedrate. The intermediate feedrates are set to mimic constant acceleration.

  ARGUMENTS:   float setpoint, the target position entered by the user
               float fr, the end feedrate entered by the user

  RETURNS:     None

  LOCAL VARIABLES: int32_t result, for storing the result

  SHARED VARIABLES: None

  GLOBAL VARIABLES:
    volatile int32_t precalculated_v[N_ELEM]: This holds the precomputed
    
  INPUTS/OUTPUTS: None

  DEPENDENCIES:
        TimeControlInt.h        for flag, precalculated_v, and macros
  -----------------------------------------------------------------------------
*/
void linear_move_action(float setpoint, float fr) {
  int32_t x0 = yw;           // Capture initial position from global variable
  int32_t xf;                // Setpoint as an int  
  int32_t xe;                // Intermediate value

  if (fr == NOT_FOUND) { // If no feedrate given, end feedrate will be the same as start feedrate
    fr = feedrate;
  }

  // Convert from inches/mm to motor rotation amount to our destination
  xf = interpolate_pos(setpoint);
  xf = find_target(xf, x0);
  xf = bound_pos(xf);

  // Find the position (N_ELEM-1)/N_ELEM of the way through the travel
  xe = (xf-x0)*N_ELEM/(N_ELEM-1) + x0;
  
  for(uint32_t i = 0; i < (N_ELEM-1); i++){
    // First create our list of key positions
    precalculated_v[i] = map(i, 0, N_ELEM-1,x0, xe);

    // Calculate the velocity at each position, convert it to a int32_t, and bound it
    precalculated_v[i] = bound_vel(interpolate_vel(calc_v(feedrate, fr, x0, xf, precalculated_v[i])));
  }

  precalculated_v[N_ELEM-1] = bound_vel(interpolate_vel(fr)); // end with user set feedrate

  feedrate = fr; // update feedrate with what the user input
  return;
}


/*
  -----------------------------------------------------------------------------
  DESCRIPTION: process_g(char instruction[]) executes the g code. This involves setting flags which either change the device settings or set the motor to move.

  OPERATION:   We read the integer after 'G' and go through a lookup switch statement to see what to do. Then, the command is executed.

  ARGUMENTS: char command[]; the pointer to the command buffer array.

  RETURNS: None

  LOCAL VARIABLES:
        code_f, code_i: The value we searched for


  SHARED VARIABLES:
        feedrate:

  GLOBAL VARIABLES:
        flags: we change the global flags to set parameters

  DEPENDENCIES:
        ProcessCmd.h: Contains macros.
        TimeControlInt.h: Contains global vairables

  -----------------------------------------------------------------------------
*/
void process_g(char instruction[]) {
  float code_f, code_f2;          // For managing the readings
  code_f = search_code('G', instruction);
  int code_i = code_f; // Convert from float to int so we can use it in the switch statement
  int32_t target;

  switch (code_i) {
    case NO_CMD:
      break;

    case RAPID_MOV:
      // Move to target point at maximum feedrate
      code_f = search_code('X', instruction);
      if (code_f == NOT_FOUND) {
        return;
      }
      // Convert the reading from inches/mm to rotation
      target = interpolate_pos(code_f);
      // Find target position
      target = find_target(target, yw);

      // wait until we are no longer IN_PROGRESS before starting the new command
      while (flags & 1 << IN_PROGRESS) {
        delayMicroseconds(CHECK_TIME_US);
      }
      mode = 'x';
      flags |= 1 << IN_PROGRESS;
      flags |= MOVE_COMMAND << COMMAND_SHIFT;
      // Keep output position within boundaries
      r = bound_pos(target);
      break;
    case LINEAR_MOV:
      // First, we handle the case "G1 Fxxx" and set the feedrate to xxx without
      // doing any movement. This occurs when there is no X command.
      code_f = search_code('F', instruction);
      code_f2 = search_code('X', instruction);

      if (code_f2 == NOT_FOUND) {
        // If no x position is given, update the feedrate and return.
        if (code_f != NOT_FOUND) {
          feedrate = code_f;
        }
        break;
      }
      else {
        // Otherwise, we have something like "G1 Xxxx" or "G1 Xxxx Fxxx"
        // Hop into a function to do this
        linear_move_action(code_f2, code_f);
      }
      break;

    case SET_ABS:
      // Set absolute positioning
      flags |= 1 << POS_ABSOLUTE;
      break;

    case SET_REL:
      // Set relative positioning
      flags &= ~(1 << POS_ABSOLUTE);
      break;

    case CHANGE_UNIT_IN:
      // Change units to inches
      flags &= ~(1 << UNITS_MM);
      break;

    case CHANGE_UNIT_MM:
      // Change units to mm
      flags |= 1 << UNITS_MM;
      break;

    case SET_HOME:
      // Set the current location to home
      xmin = yw;
      break;

    case HOME:
      // Check if we want to do a full calibration - uses "A" code
      if (search_code('A', instruction) != NOT_FOUND) {
        xmin = 0;
        xmax = 0;
        calibrate();
        calib_home();
      }
      if ((search_code('X', instruction) == NOT_FOUND) || (xmin != 0 || xmax != 0)) {
        // Do the full calibration
        if (U > UNLOADED_EFFORT_LIM) {
          break; // Can't home if effort's too high
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
      if (code_f != NOT_FOUND) {
        // Set global variables with timing info and the command that we are doing
        ctrl_end = code_f;
        ctrl_start = millis();
        flags |= 1 << IN_PROGRESS;
        flags |= DWELL_COMMAND << COMMAND_SHIFT;
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
void process_cmd(char command[]) {
  char cmdType = command[0];
  switch (cmdType) {
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
  flags &= ~(1 << CMD_READY);
  return;
}
