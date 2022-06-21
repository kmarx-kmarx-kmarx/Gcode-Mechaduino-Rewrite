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
        uint32_t interval:            Time in ms between debug messages

  GLOBAL VARIABLES:
        motor_flags: Used to communicate the motor controller interrpts's state

  INCLUDES:
        SerialCommand.h   Read data from Serial buffer and put it in the command buffer
        MagneticEncoder.h   Initialize and read data from the magnetic encoder
        TimeControlInt.h  Initialize and run timer control interrupts
        ProcessCmd.h      Read from the command buffer and execute the command
        MotorCtrl.h       Low-level motor control interface functions
        string.h          For array operations
  --------------------------------------  ---------------------------------------
*/

#include "SerialCommand.h"
#include "MagneticEncoder.h"
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
  SerialUSB.begin(BAUDRATE);     // BAUDRATE is defined in SerialCommand.h

  setup_pins();         // configure pinouts for controlling the motor
  setup_TCInterrupts(); // configure controller timed interrupts
  encoder_setup();      // Sets up SPI for communicating with encoder

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
        uint32_t interval: time between debug statements, milliseconds
        bool debugging: true if we want to send the debug messages

  GLOBAL VARIABLES:
        motor_flags: Used to communicate the motor controller interrpts's state, from TimeControlInt
        debugging:   Used to determine if we are in debugging mode, set by SerialCommand
        stopped:     Used to determine of the motor should be stopped, set by SerialCommand

  DEPENDENCIES:
        MotorCtrl.h:  for controlling the motor
        ProcessCmd.h: turn the command string into instructions for the motor
        string.h:     string manipulations
        SerialCommand:read the command, send acknowledgement, and send debug info
  -----------------------------------------------------------------------------
*/
bool stopped = false;
bool requested = false;
void loop()
{
  // First, we check to see if we should stop.
  if(flags & (1<<CMD_READY)){
    requested = false;
    // Create two null-terminated strings, one which is the first 4 characters of command and another which is "M112", the stop command
    char stopCheck[5];
    strncpy(stopCheck, cmd, 4);
    stopCheck[4] = '\0';
    char STOP[5] = {'M', '1', '1', '2', '\0'};

    // Check if the strings are the same; if they are, we received a stop command and should stop
    if(strcmp(stopCheck, STOP) == 0){
      stopped = true;
      stop_motor();
    }

    // If not stopped, run the command
    if(!stopped){
      process_cmd(cmd);
    }
    // Clear the command buffer - set it to be all 0
    memset(cmd, 0, COMMAND_SIZE);
  }
  // If we don't have a command, check the serial buffer
  else{
    // If we do have something in the serial buffer, read it in and keep it in the buffer
    if(SerialUSB.available()>0){
      read_serial(cmd);
      send_ack(flags & (1<<CMD_READY));
    }
    // If there is nothing on the serial buffer, request a command
    else{
      if(!requested){
        request_new();
        requested = true;
      }
    }
  }
  // Finally, if we are debugging, send the debug information
  if(flags & (1<<DEBUG_MODE)){
    send_debug(DEBUG_DEFAULT);
  }
}
