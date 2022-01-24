/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains macro definitions and function delcarations for SerialCommad.cpp.
  -----------------------------------------------------------------------------
*/

#ifndef __SERIALCOM_H__
#define __SERIALCOM_H__
  // Expression for determining if a character is an end-of-line character
  #define END_OF_LINE(ch)(((char)ch == '\n') || ((char)ch == '\r'))
  // Expression for determining if character is a number character (0 to 9)
  #define IS_NUMBER(ch)(((int)ch >= 48) || ((int)ch <= 57))
  // Expression for determining if a character is a capital letter
  #define IS_CAPITAL(ch)(((int)ch >= 65) || ((int)ch <= 90))
  // Expression for determining if a character is hex (0 to F)
  #define IS_HEX(ch)((((int)ch >= 48) || ((int)ch <= 57)) || (((int)ch >= 65) || ((int)ch <= 70)))

  #define COMMAND_SIZE  32   // Commands are 31 characters long (at most); we need to terminate with a null
  #define DEBUG_DEFAULT 500  // We send a debug message every 500 ms by default
  #define DELAY_COUNT   10   // The data can be late this many times before we give up on sending it
  #define DELAY_TIME    50   // Wait this many microseconds for new data to appear on the serial buffer
  #define REQ_CHAR      "p"  // Send this character for request. We choose a character that does not appear in any other communication
  #define BAD_CMD_CHAR  "q"
  #define OK_CMD_CHAR   "r"

  #define BAUDRATE 115200    // Set the baud rate

  bool read_serial(char command[]);
  void send_ack(bool isValid);
  void request_new();
  void send_debug(int interval);

#endif