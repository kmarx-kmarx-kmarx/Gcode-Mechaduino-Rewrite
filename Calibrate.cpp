/*
  -----------------------------------------------------------------------------
  DESCRIPTION: This file contains the functions necessary to calibrate the motor and write the data to flash storage.
        write_page(): erase the data at shared variable page_ptr and write the data at shared variable page to it

        store_lookup(int lookupAngle): put data into buffer and if the buffer is full, write it to flash

        calibrate(): Step the motor through all positions and save the data

        findijStart(int readings[], int* istart, int* jstart): Find where in the lookup table the reading should go

  SHARED VARIABLES: Various variables for tracking the flash data.

  GLOBAL VARIABLES: lookup[]: lookup table for encoder-value-to-angle conversions

  INCLUDES:
        MotorCtrl.h: Contains macros and function declarations
        TimeControlInt.h: We need this to turn off the interrupts
        MagneticEncoder.h: For macros and encoder reading functions
        Arduino.h: Has macros and definitions for the board
  -----------------------------------------------------------------------------
*/


#include <FlashStorage.h>
#include "MagneticEncoder.h"
#include "MotorCtrl.h"
#include <Arduino.h>

// Initialize lookup table with 2^14 elements
extern const int __attribute__((__aligned__(256))) lookup[16384] = {};

// Initialize flash parameters
static FlashClass flash;               // Create new flash object

static const unsigned page_size = 256; // Parameters for allocating data
static unsigned page_count;
static const unsigned ints_per_page = page_size / sizeof(int);

static int page[ints_per_page];        // Buffer everthing that will be stored in ram
static const void * page_ptr;          // Pointer to flash page

// Constants for operation
const int N_AVG = 16;

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
  if(page_count < ints_per_page){
    return;
  }

  // we've filled an entire page, write it to the flash
  write_page();

  // reset our counters and increment our flash page
  page_ptr += sizeof(page);
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

  LOCAL VARIABLES: int ticks, stepNo: Track the position along the readings and 

  SHARED VARIABLES: int page_count: This is our index of the page buffer array
                    int page[]: This is the page buffer array 
                    void * page_ptr: This void points to the start of the flash page we want to write to

  GLOBAL VARIABLES: None

  DEPENDENCIES:
        FlashStorage.h:  has definitions for flash manipulations.
  -----------------------------------------------------------------------------
*/
void findijStart(int readings[], int* istart, int* jstart){
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
  DESCRIPTION: calibrate() runs the calibration routine  

  OPERATION:   

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

  SerialUSB.println("Beginning calibration routine...");
  disableTCInterrupts();
  // Take a couple steps to make sure it's working right
  stepIdx = oneStep(CW, stepIdx);
  delay(MOTOR_SETTLE);
  stepIdx = oneStep(CW, stepIdx);
  delay(MOTOR_SETTLE);
  //average multple readings at each step
  for (int reading = 0; reading < N_AVG; reading++){  
    // add VALS_PER_REV to each reading and then modulo to prevent wraparound errors
    currentencoderReading = encoder_read() + VALS_PER_REV;
    lastencoderReading += currentencoderReading;
    // Wait a bit before the next reading
    delay(MOTOR_SETTLE);
  }
  lastencoderReading /= avg;
  lastencoderReading = lastencoderReading % VALS_PER_REV;

  // Take another step and take another reading
  stepIdx = oneStep(CW, stepIdx);
  delay(MOTOR_SETTLE);
  for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
    currentencoderReading = encoder_read() + VALS_PER_REV;
    encoderReading += currentencoderReading;
    delay(MOTOR_SETTLE);
  }
  encoderReading /= avg;
  encoderReading = encoderReading % VALS_PER_REV;

  // Take the difference
  currentencoderReading = (encoderReading - lastencoderReading);
  // Wired backwards if:
  // 1) we see a rollover from low to high (should be high to low)
  // 2) we see a small step lower (should be higher)
  if((currentencoderReading > (VALS_PER_REV/2)) || (currentencoderReading < 0)){
    SerialUSB.println("Try again. If problem persists, swap wiring");
    return CALIBRATION_FAIL;
  }

  // Return to stepIdx 0
  stepIdx = 0;
  output(0, uMAX >> 2);
  delay(MOTOR_SETTLE);
  
  //step through all full step positions, recording their encoder readings
  for (int x = 0; x < VALS_PER_REV; x++) {     
    encoderReading = 0;               // init. as 0 for averages
    delay(MOTOR_SETTLE);              //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
    lastencoderReading = readEncoder();
    
    for (int reading = 0; reading < N_AVG; reading++) {  //average multple readings at each step
      currentencoderReading = readEncoder();

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


int read_angle(avg) {
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