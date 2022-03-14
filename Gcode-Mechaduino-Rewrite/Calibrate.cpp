/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains the functions necessary to calibrate the motor and write the data to flash storage.
        write_page(): erase the data at shared variable page_ptr and write the data at shared variable page to it

        store_lookup(int lookupAngle): put data into buffer and if the buffer is full, write it to flash

        calibrate(): Step the motor through all positions and save the data

        findijStart(int readings[], int* istart, int* jstart): Find where in the lookup table the reading should go

        wired_correctly(): indicates whether the motor was wired correctly

  SHARED VARIABLES: Various variables for tracking the flash data.

  GLOBAL VARIABLES: lookup[]: lookup table for encoder-value-to-angle conversions

  INCLUDES:
        MotorCtrl.h: Contains macros and function declarations
        TimeControlInt.h: We need this to turn off the interrupts
        MagneticEncoder.h: For macros and encoder reading functions
        Arduino.h: Has macros and definitions for the board
        stdint.h: 
  -----------------------------------------------------------------------------
*/


#include <FlashStorage.h>
#include "MagneticEncoder.h"
#include "MotorCtrl.h"
#include "TimeControlInt.h"
#include <Arduino.h>

// Initialize lookup table with 2^14 elements, one for each possible encoder value
extern const int __attribute__((__aligned__(256))) lookup[16384] = {};

// Initialize flash parameters
static FlashClass flash;               // Create new flash object

static unsigned page_count;

static int page[INTS_PER_PAGE];        // Buffer everthing that will be stored in ram
static const void * page_ptr;          // Pointer to flash page

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: write_page() writes the page buffer to flash storage.

  OPERATION:   We first erase the page that page_ptr points to and then we write page to page_ptr.

  ARGUMENTS/RETURNS: none

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: void * page_ptr: This void points to the start of the flash page we want to write to
                    int page[]: This is the page buffer array 

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        FlashStorage.h:  has definitions for flash manipulations.
  -----------------------------------------------------------------------------
*/
static void write_page() {
  flash.erase((const void*) page_ptr, sizeof(page));
  flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: store_lookup(int lookupAngle) stores data to either the page buffer or flash.

  OPERATION:   We add the lookupAngle to our page buffer. Then, if the page buffer is full, we write the page, reset the buffer, and index the page pointer.

  ARGUMENTS:   int lookupAngle: the angle to store

  RETURNS:     None

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: None

  SHARED VARIABLES: int page_count: This is our index of the page buffer array
                    int page[]: This is the page buffer array 
                    void * page_ptr: This void points to the start of the flash page we want to write to

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        FlashStorage.h:  has definitions for flash manipulations.
  -----------------------------------------------------------------------------
*/
static void store_lookup(int lookupAngle) {
  // Add angle to the page
  page[page_count++] = lookupAngle;
  // If we don't have a full page, return
  if(page_count < INTS_PER_PAGE){
    return;
  }

  // we've filled an entire page, write it to the flash
  write_page();

  // reset our counters and increment our flash page
  page_ptr = (char*)page_ptr + sizeof(page); // cast to char to adhere to GCC standard
  page_count = 0;
  memset(page, 0, sizeof(page));
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: findijStart(int readings[], int* istart, int* jstart) relates the encoder reading to an angle reading so each lookup index has the correct 

  OPERATION:   We look at the readings to find what index of readings and what offset corresponds to the 0 position of the magnetic encoder. Once we find this, we can step through the readings and interpolate a value for every encoder reading.

  ARGUMENTS:   int readings[]: the encoder reading at each step
               int* istart:    pointer to istart; we want to modify this variable without making it shared
               int* jstart:    pointer to jstart.

  RETURNS:     None

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: int ticks, stepNo: Track the position along the readings

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: None
  -----------------------------------------------------------------------------
*/
static void findijStart(int readings[], int* istart, int* jstart){
  int ticks;
  int stepNo;
  // We know readings[] will always be STEP_PER_REV long
  for(int i =0; i<STEP_PER_REV; i++){
    // Take the difference of two consecutive items (wrapping around)
    ticks = readings[((i + 1) % STEP_PER_REV)] - readings[(i % STEP_PER_REV)];
    // if a step causes wrapping around, add as needed
    if (ticks < -VALS_PER_REV/2) {
      ticks += VALS_PER_REV;
    }
    // We now have all positive ticks (enforced by calibration routine)
    for (int j = 0; j < ticks; j++){
      // Interpolate between start and end val of the tick to find
      // when the encoder would read 0
      stepNo = ((readings[i] + j) % VALS_PER_REV);
      if(stepNo==0){
        // Record the step number and 
        *istart = i;
        *jstart = j;
        return;
      }
    }
  }
  return;
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: wired_correctly() returns true if the motor is wired correctly and false if it's wired backwards. 

  OPERATION:   We take a few steps in the forwards direction then check to see if the encoder reading increased a small amount. If it underflowed or decreased, the motor is wired backwards.

  ARGUMENTS:   None

  RETURNS:     bool; true if wired correctly and false otherwise

  INPUTS:     We read values from the encoder.
  
  OUTPUTS:   The motor moves. We also send error messages over serial. We set the motor position to 0 before and after running this operation

  LOCAL VARIABLES: int stepIdx: current step index
               int lastencoderReading, encoderReading: the encoder readings before and after the steps respectively
               int diff: the difference between encoderReading and lastencoderReading

  SHARED VARIABLES: None

  GLOBAL VARIABLES: None

  DEPENDENCIES: MagneticEncoder.h: Read from the encoder
                MotorCtrl.h: Move the motor
  -----------------------------------------------------------------------------
*/

static bool wired_correctly(){
  int encoderReading = 0;
  int diff = 0;
  int lastencoderReading = 0;
  int stepIdx = 0;

  SerialUSB.println("Checking wiring...");
  disable_TCInterrupts();
  output(0, uMAX >> 2); // go to point 0
  // Take a couple steps to make sure it's working right; sometimes the initial steps cause errors.
  stepIdx = one_step(CW, stepIdx);
  delay(MOTOR_SETTLE);
  stepIdx = one_step(CW, stepIdx);
  delay(MOTOR_SETTLE);
  //average multple readings at each step
  for (int reading = 0; reading < N_AVG; reading++){  
    // It is unlikely we will be reading values right at the wrap point so we will ignore this
    lastencoderReading += encoder_read();
    // Wait a bit before the next reading
    delay(MOTOR_SETTLE);
  }
  lastencoderReading /= N_AVG;

  // Take another step and take another reading
  stepIdx = one_step(CW, stepIdx);
  delay(MOTOR_SETTLE);
  for (int reading = 0; reading < N_AVG; reading++) {  //average multple readings at each step
    encoderReading += encoder_read();
    delay(MOTOR_SETTLE);
  }
  encoderReading /= N_AVG;

  // Take the difference
  diff = (encoderReading - lastencoderReading);
  // Return to stepIdx 0
  stepIdx = 0;
  output(0, uMAX >> 2);
  delay(MOTOR_SETTLE);
  // Wired backwards if:
  // 1) we see a rollover from low to high (should be high to low)
  // 2) we see a small step lower (should be higher)
  if((diff > (VALS_PER_REV/2)) || (diff < 0)){
    SerialUSB.println("Appears to be backwards");
    return false;
  }
  else{
    return true; // If wired correctly, return true
  }
}

/*
  -----------------------------------------------------------------------------
  DESCRIPTION: calibrate() runs the calibration routine  

  OPERATION:   We first take a couple steps to verify the motor is operating properly; if the encoder shows the motor is turning the opposite direction, the motor wires must be reversed. We then step through each step, read the encoder, and interpolate 

  ARGUMENTS:   none

  RETURNS:     None

  INPUTS / OUTPUTS: None

  LOCAL VARIABLES: int ticks, stepNo: Track the position along the readings and 

  SHARED VARIABLES: int page_count: This is our index of the page buffer array
                    int page[]: This is the page buffer array 
                    void * page_ptr: This void points to the start of the flash page we want to write to
                    int lookup[]: the lookup table

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        FlashStorage.h:  has definitions for flash manipulations.
        MotorCtrl.h:     For interacting with the motor
        MagneticEncoder.h: For reading from the encoder
  -----------------------------------------------------------------------------
*/
int calibrate() {
  int encoderReading = 0;
  int currentencoderReading = 0;
  int lastencoderReading = 0;
  
  int iStart = 0;     //encoder zero position index
  int jStart = 0;
  
  int fullStepReadings[STEP_PER_REV]; // Reading at every step
    
  int fullStep = 0;
  int ticks = 0;
  int lookupAngle = 0;

  int stepIdx = 0;

  // Check to see if we are wired backwards
  if(!wired_correctly){
    return ;
  }

  
  //step through all full step positions, recording their encoder readings
  for (int x = 0; x < VALS_PER_REV; x++) {     
    encoderReading = 0;               // init. as 0 for averages
    delay(MOTOR_SETTLE);              //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = encoder_read();
    
    for (int reading = 0; reading < N_AVG; reading++) {  //average multple readings at each step
      currentencoderReading = encoder_read();

      // If we are on the edge of wrapping around, add
      // or subtract as needed to keep the value correct
      if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
        currentencoderReading += cpr;
      }
 
      encoderReading += currentencoderReading;
      delay(READ_TIME);
    }
    // Take the average
    encoderReading = encoderReading / avg;
    // Put it back in range of the 14 bit value
    if (encoderReading>=cpr){
      encoderReading-= cpr;
    }
    else if (encoderReading<0){
      encoderReading+= cpr;
    }

    fullStepReadings[x] = encoderReading;
    
    // go to next step
    oneStep();
  }
  // Once we know everything, we can analyze the data.
  findijStart(fullStepReadings, &iStart, &jStart);

  // The code below generates the lookup table by intepolating between
  // full steps and mapping each encoder count to a calibrated angle
  // The lookup table is too big to store in volatile memory,
  // so we must generate and store it into the flash on the fly

  // begin the write to the calibration table
  page_count = 0;
  page_ptr = (const uint8_t*) lookup;
  // Start counting at iStart
  for (int i = iStart; i < (iStart + spr + 1); i++) {
    ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

    if (ticks < -cpr/2) {           //check if current interval wraps over encoder's zero positon
      ticks += cpr;
    }
    if (i == iStart) { //this is an edge case
      // starting at 0, go through the ticks and assign an angle
      // given that 1 tick = 1 aps
      // For this case, we only care about the vals between jStart and ticks
      for (int j = jStart; j < (ticks); j++) {
	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
    else if (i == (iStart + spr)) { //this is an edge case
      // this time, we are ending at 0, making sure not to double-count
      // the ones covered in the previous case
      for (int j = 0; j < jStart; j++) {
	     store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
    else {                        //this is the general case
      for (int j = 0; j < ticks; j++) {
	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
      }
    }
  }

  // Store unwritten page
  if (page_count != 0)
	write_page();
  SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
  return CALIBRATION_SUCCESS;
}


static int read_angle(int avg) {
  int prevReading = readEncoder();
  int encoderReading = readEncoder();


  disableTCInterrupts(); // Ensure interrupts don't move the motor

  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    encoderReading += readEncoder();
    delay(MOTOR_SETTLE);
  }

  // Angle reading is the index of the lookup table
  return lookup[encoderReading / avg];
}