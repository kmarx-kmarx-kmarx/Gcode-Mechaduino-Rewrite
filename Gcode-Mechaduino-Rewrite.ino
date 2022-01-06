/*
  -----------------------------------------------------------------------------
  Poseidon Firmware v0.1
  SAM21D18 (Arduino Zero compatible), AS5047 encoder, A4954 driver

  This work is based on the Mechaduino firmware and PCB.
  All Mechaduino related materials are released under the
  Creative Commons Attribution Share-Alike 4.0 License
  https://creativecommons.org/licenses/by-sa/4.0/
  -----------------------------------------------------------------------------
*/

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains #include statements for other files necessary for operation as well as the setup and loop functions.
        setup()         Begins Serial and SPI communications, sets up pins for digital IO with the motor controller, and begins the control loop interrupts.
        
        loop()          Here, we manage reading in Serial data, outputting the ack signal, outputting debug info, and processing the commands.
        
  SHARED VARIABLES:
        char cmd[COMMAND_SIZE]:  This is the command buffer.

  GLOBAL VARIABLES:
        motor_flags: Used to communicate the motor controller interrpts's state

  INCLUDES:
        SerialCommand.h   Read data from Serial buffer and put it in the command buffer
        MagnetEncoder.h   Initialize and read data from the magnetic encoder
        TimeControlInt.h  Initialize and run timer control interrupts
        ProcessCmd.h      Read from the command buffer and execute the command
        MotorCtrl.h       Low-level motor control interface functions
        string.h          For array operations
  -----------------------------------------------------------------------------
*/

#include "SerialCommand.h"
#include "MagnetEncoder.h"
#include "TimeControlInt.h"
#include "ProcessCmd.h"
#include "MotorCtrl.h"
#include <string.h>

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: setup() runs once at startup and prepares the board for operation.

  OPERATION:   

  ARGUMENTS/RETURNS: none

  LOCAL VARIABLES: none

  SHARED VARIABLES:
        char cmd[COMMAND_SIZE]:  This is the command buffer. COMMAND_SIZE is defined in SerialCommand

  GLOBAL VARIABLES:

  DEPENDENCIES:
        SerialCommand.h  Has Serial baudrate definition
        MagnetEncoder.h  For interfacing with the magnetic encoder
        TimeControlInt.h Initialize the interrupts
        MotorCtrl.h      Initialize the motor controller IC
        string.h         Initialize the command buffer
  -----------------------------------------------------------------------------
*/
char cmd[COMMAND_SIZE];

void setup()
{      
  // Begin communications
  SerialUSB.begin(BAUDRATE);     // BAUDRATE is defined in Parameters.h

  setupPins();         // configure pinouts for controlling the motor
  setupTCInterrupts(); // configure controller timed interrupts
  setupSPI();          // Sets up SPI for communicating with encoder

  // Enable the timed interrupts
  enableTCInterrupts();

  // Clear the command buffer
  memset(cmd, 0, COMMAND_SIZE);

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: loop() runs repeatedly and manages Serial IO and the motor commands.

  OPERATION:   Every cycle, loop() checks if we want to emergency stop, runs commands, processes serial input, requests commands, and sends debug information.      
  
  ARGUMENTS/RETURNS: none

  LOCAL VARIABLES: none

  SHARED VARIABLES: 
        char cmd[COMMAND_SIZE]:  This is the command buffer. COMMAND_SIZE is defined in SerialCommand

  GLOBAL VARIABLES:
        motor_flags: Used to communicate the motor controller interrpts's state, from TimeControlInt
        debugging:   Used to determine if we are in debugging mode, set by SerialCommand
        stopped:     Used to determine of the motor should be stopped, set by SerialCommand

  DEPENDENCIES:
        
  -----------------------------------------------------------------------------
*/ 
void loop()
{
  bool have_cmd = !(cmd[0] == 0); // We have a command if the first character of the command buffer is nonzero

  // First, we check to see if we should stop.
  if(have_cmd){
    char stop_check[5];
    strncpy(stop_check, cmd, 4);
    stop_check[4] = '\0';
    char STOP[5] = "M112\0";
    // Compare the first 4 characters to the e-stop command
    if(strcmp(stop_check, STOP) == 0){
      stopped = true;
      stop_motor();
    }

    // We next check if we are ready for a new command
    if(flags & (1<<CMD_READY)){
      // If not stopped, run the command
      if(!stopped){
        process_cmd(cmd);
      }
      // Clear the command buffer
      memset(cmd, 0, COMMAND_SIZE);
    }
  }
  // If we don't have a command, check the serial buffer
  else{
    // If we do have something in the serial buffer, read it in
    if(SerialUSB.available()>0){
      read_serial(cmd);
      send_ack();
    }
    // If there is nothing on the serial buffer, request a command
    else{
      request_new();
    }
  }
  // Finally, if we are debugging, send the debug information
  if(debugging){
    send_debug();
  }
}