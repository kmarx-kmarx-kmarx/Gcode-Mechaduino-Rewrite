/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains #include statements for other files necessary for operation as well as the data IO functions.
        
        read_serial(char command[]) We pass the command array to this function by reference; we remove leading whitespace and format the characters properly
        
        send_ack()                  This function sends a message acknowledging the message was received properly

        request_new()               This function sends message indicating it is ready for a new command

        send_debug(int interval)     This function sends the debug information every interval (in milliseconds)
        
  SHARED VARIABLES:
        uint32_t prevDebugTime: Used to keep track of time since the last debug message was sent

  GLOBAL VARIABLES:

  INCLUDES:
        Arduino.h:       Built-in utilities
        SerialCommand.h: Contains functions and macros for reading and writing data over Serial.
        TimeControlInt.h: Contains motor state variables
  -----------------------------------------------------------------------------
*/
#include <Arduino.h>
#include "SerialCommand.h"
#include "TimeControlInt.h"

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: read_serial(char command[]) reads in the data stream and puts the characters into the command[] character array.

  OPERATION:   This function has strict requirements for a valid Gcode command. 
               First, commands must start with "M" or "G"; it does not support line numbers
               The M or G then must be immediately followed by an integer number. Afterwards, there must be a space.
               Then, if the command has parameters, we must see a capital letter followed by another number and a space.
               This template must follow for all parameters.
               Then, the command must be ended with an optional asterisk, checksum, and newline or just a newline.

               For example, "G62 F12.3 X2345 *40 \n" would be a valid command. Note the spaces. 
               The command must be fewer than (COMMAND_SIZE - 1) characters long.
               If the command does not comply with these requirements, the Serial buffer will be flushed and the device will return false, indicating a bad command.

  ARGUMENTS: char command[]; the pointer to the command buffer array.

  RETURNS: bool isValid; true if the command was valid and false otherwise

  LOCAL VARIABLES: 
        char inChar     The character read in from the data stream

        char sigChar    Track the previous signal character to ensure the proper value appears after it

        int delays      The number of times we had to wait for new data to come in. If this exceeds DELAY_COUNT, we stop reading.

        unsigned int cmdIdx The index of the command buffer we are writing to

        bool isValid    The value we want to return

        int csumCalc    The calculated checksum

        char csumRead[] The checksum sent over Serial

  SHARED VARIABLES: 
        char command[]  The command buffer. This function takes the pointer to the command buffer and writes to the command buffer.

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        SerialCommand.h: Contains macros for timing.
  -----------------------------------------------------------------------------
*/ 
bool read_serial(char command[]){
  char inChar;      // Character being read off the serial buffer
  char sigChar; //
  uint8_t delays = 0;  // Amount of times we have had to wait for the serial buffer to have data
  uint16_t  cmdIdx = 0; // Index of command
  int32_t csumCalc;
  uint16_t csumIdx = 0; // Index on the csum string
  char csumStr[3]; // The checksum is one byte in hexdecimal; we need two characters and then null termination
  int16_t csumRead;
  // Assume the command is valid and reset the command buffer 0
  bool isValid = true;
  memset(command, 0, COMMAND_SIZE);

  // Try to read from the serial buffer until we either time out or finish reading
  // While we haven't timed out...
  while(delays < DELAY_COUNT){
    // If we have Serial data on the buffer...
    if(SerialUSB.available()>0){
      // Read the character
      inChar = SerialUSB.read();
      // First, ensure we don't write beyond our buffer
      if(cmdIdx >= COMMAND_SIZE){
        isValid = false;
        break;
      }
      // Check the first character
      else if(cmdIdx == 0){
        // If it's M or G, read it in
        if(inChar == 'M' || inChar == 'G'){
          command[cmdIdx++] = inChar;
          sigChar = inChar;
        }
        // Otherwise, the command is not valid and exit the loop
        else{
          isValid = false;
          break;
        }
      }
      // Now ensure an integer appears after the M or G
      else if(cmdIdx == 1){
        if(IS_NUMBER(inChar)){
          command[cmdIdx++] = inChar;
        }
        else{
          isValid = false;
          break;
        }
      }
      // If the signal character is M or G, only allow numbers or space
      else if(sigChar == 'M' || sigChar == 'G'){
        if(IS_NUMBER(inChar)){
          command[cmdIdx++] = inChar;
        }
        else if(inChar == ' '){
          command[cmdIdx++] = inChar;
          // Track that we got a space
          sigChar = ' ';
        }
        else{
          isValid = false;
          break;
        }
      }
      // Handle sigChar = ' '; either get a new capital letter, checksum, or newline
      else if(sigChar == ' '){
        // Read in the capital letter
        if(IS_CAPITAL(inChar)){
          sigChar = inChar;
          command[cmdIdx++] = inChar;
        }
        // Signal the checksum is coming up
        else if(inChar == '*'){
          sigChar = '*';
        }
        else if(END_OF_LINE(inChar)){
          // Null-terminate our string exit the loop
          command[cmdIdx] = '\0';
          break;
        }
        else{
          isValid = false;
          break;
        }
      }
      // If we have a captial letter, read numbers, period, or space
      else if(IS_CAPITAL(sigChar)){
        if(IS_NUMBER(inChar)){
          command[cmdIdx++] = inChar;
        }
        else if(inChar == ' ' || inChar == '.'){
          sigChar = inChar;
          command[cmdIdx++] = inChar;
        }
        else{
          isValid = false;
          break;
        }
      }
      // If we have a period, we only want to read more numbers or space
      else if(sigChar == '.'){
        if(IS_NUMBER(inChar)){
          command[cmdIdx++] = inChar;
        }
        else if(inChar = ' '){
          sigChar = inChar;
          command[cmdIdx++] = inChar;
        }
        else{
          isValid = false;
          break;
        }
      }
      // Process the checksum
      else if(sigChar = '*'){
        // Null terminate the command
        command[cmdIdx] = '\0';

        // First, check if we have a space. If so, try to perform the checksum
        if(inChar == ' '){
          // Index >= 2 means we have read the two characters
          if(csumIdx >=2){
            // Null-terminate the csum string
            csumStr[2] = '\0';
            // Convert it to an int
            csumRead = (int) strtoul(csumStr, 0, 16);

            // Calculate the checksum
            csumCalc = 0;
            // Perform CRC
            for(int i = 0; command[i] != '\0'; i++){
              csumCalc = csumCalc ^ command[i];
            }
            // Trim off extra bits just in case
            csumCalc &= 0xFF;
          }
          else{
            isValid = false;
            break;
          }
        }
        // If it isn't a space, check if we have already read enough characters
        else if(csumIdx >= 2){
          isValid = false;
          break;
        }
        // If it isn't a space and we still want to read characters, check if it's hex
        else if(IS_HEX(inChar)){
          csumStr[csumIdx++] = inChar;
        }
        // Otherwise, it's invalid
        else{
          isValid = false;
          break;
        }

      }
    }
    // If there is no data on the buffer and we haven't finished reading, wait for more data
    else{
      delays++;
      delayMicroseconds(DELAY_TIME);
    }
  }


  // Read the characters until we hit a "M" or "G"
  while(SerialUSB.available()>0){
    inChar = SerialUSB.read();

    if(inChar == 'M' || inChar == 'G'){
      break;
    }
  }
  // At this point, inChar is either 'A', the default value (the command was invalid), 'M' (we have a M command), or 'G' (we have a G command)
  if(inChar == 'M' || inChar == 'G'){
    // If it's M or G, start reading in the rest of the command.
    // Get the first byte stored into our string
    command[0] = inChar;
    // While we haven't exceeded our delay limit...
    while(delays < DELAY_COUNT){
      // Check if there's any data available
      if(SerialUSB.available()>0){
        // Read in the new character
        inChar = SerialUSB.read();
        // Check - is it end of line?
        if(END_OF_LINE(inChar)){
          // We are done reading, exit the loop. Everything after the newline is considered to be trash.
          // Ending with a newline ensures the command was valid
          isValid = true;
          break;
        }
        else if(cmdIdx > (COMMAND_SIZE-1)){
          // If a command is too long, we might have missed something important. We will signal the command is bad. We want to exit the while loop here
          break; 
        }
        else{
          // Otherwise, read in the character
          command[cmdIdx++] = inChar;
        }
      }
      else{
        // If we don't have any new bytes available, wait a little bit in case any more come in
        delays++;
        delayMicroseconds(DELAY_TIME);
      }
    }
  }

  // If not valid, clear the first character of the command to indicate it is not valid
  if(!isValid){
    command[0] = 0;
  }

  // Null-terminate the string
  command[cmdIdx] = '\0';

  // Clear the buffer (there's nothing more to read)
  SerialUSB.flush();

  return isValid;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: send_ack(bool isValid) sends a byte to the host

  OPERATION:   We send a single character to indicate success. The host then looks for this character to confirm that the command was received properly.
  
  ARGUMENTS: bool isValid: true if the received command was valid, false otherwise

  RETURNS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES:
        None

  DEPENDENCIES:
        SerialCommand.h: Contains macro.
  -----------------------------------------------------------------------------
*/ 
void send_ack(bool isValid){
  if(isValid){
    SerialUSB.println(OK_CMD_CHAR);
  } 
  else{
    SerialUSB.println(BAD_CMD_CHAR);
  }

  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: send_debug(int interval) checks the time since the last debug message was sent and if interval ms elapsed, it sends a new debug message.

  OPERATION: 
  
  ARGUMENTS: int interval, the number of milliseconds between each debug

  RETURNS: None

  LOCAL VARIABLES: 
        formattedFlag: the binaryInt representation of the flag

  SHARED VARIABLES: 
        prevDebugTime: the time since the last debug message was sent

  GLOBAL VARIABLES:
        flags: We read flags to send it out as our ack
        yw:    Motor position
        u:     Motor "effort", a measure of torque
        r:     Motor setpoint, either the target position or target velocity

  DEPENDENCIES:
        State.h:      for the flags, position, setpoint, and effort variables
  -----------------------------------------------------------------------------
*/ 
uint32_t prevDebugTime = 0;
void send_debug(int interval){
  // If we are debugging, send state variables as bytes
  // Here, we are sending out:
  //    0) flags (i.e. what command to run)
  //    1) time in ms
  //    2) position in degrees
  //    3) effort 
  //    4) setpoint 
  uint32_t time = millis(); // get the time
  // copy all these values into byte array representations
  // note: all these are 4 byte values
  uint8_t * timebytes = (uint8_t *) &time;
  uint8_t * flagbytes = (uint8_t *) &flags;
  uint8_t * posbytes = (uint8_t *) &yw;
  uint8_t * effbytes = (uint8_t *) &u;
  uint8_t * setbytes = (uint8_t *) &r;
  // put them into an array for easier manipulation
  uint8_t * data[] = {timebytes, flagbytes, posbytes, effbytes, setbytes};

  // We check the time
  if(prevDebugTime + interval > time){
    prevDebugTime = time;
    // Next, write all our bytes as hex for easy manipulation
    for(int i = 0; i < (sizeof(data) / sizeof(data[0])); i++){
      uint8_t * ptr = data[i];
      for(int j = 0; j < sizeof(ptr); j++){
        char hexCar[2];
        sprintf(hexCar, "%02X", ptr[j]);
        SerialUSB.print(hexCar);
      }
      if(i < ((sizeof(data) / sizeof(data[0]))-1)){
        SerialUSB.print(",");
      }
    }
    SerialUSB.print("\n");
  }
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: request_new() sends a byte to indicate it is ready to receive a new command. We are ready to receive a new command in two cases: first, if the current command was completed then we are ready for a new command and second, if the previously sent command couldn't be parsed for some reason we want the command to be re-sent

  OPERATION:    We SerialUSB.println a byte.
  
  ARGUMENTS: None

  RETURNS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None
        
  DEPENDENCIES:
        SerialCommand.h: Contains macros.
  -----------------------------------------------------------------------------
*/ 
void request_new(){
  SerialUSB.println(REQ_CHAR);
  return;
}