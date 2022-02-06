
// #include <FlashStorage.h>
// #include "MagneticEncoder.h"
// #include "MotorCtrl.h"

// // Initialize flash parameters
// static FlashClass flash;               // Create new flash object

// static const unsigned page_size = 256; // Parameters for allocating data
// static unsigned page_count;
// static const unsigned ints_per_page = page_size / sizeof(int);

// static int page[ints_per_page];        // Buffer everthing that will be stored in ram
// static const void * page_ptr;          // Pointer to flash page


// static void write_page()
// {
//   flash.erase((const void*) page_ptr, sizeof(page));
//   flash.write((const void*) page_ptr, (const void *) page, sizeof(page));
// }

// static void store_lookup(float lookupAngle)
// {
//   page[page_count++] = lookupAngle;
//   if(page_count != ints_per_page)
//     return;

//   // we've filled an entire page, write it to the flash
//   write_page();

//   // reset our counters and increment our flash page
//   page_ptr += sizeof(page);
//   page_count = 0;
//   memset(page, 0, sizeof(page));
// }


// int calibrate() {   /// this is the calibration routine

//   int encoderReading = 0;
//   int currentencoderReading = 0;
//   int lastencoderReading = 0;
//   int avg = 16;

//   int iStart = 0;     //encoder zero position index
//   int jStart = 0;
  
//   int fullStepReadings[STEP_PER_REV];
    
//   int fullStep = 0;
//   int ticks = 0;
//   float lookupAngle = 0.0;
//   SerialUSB.println("Beginning calibration routine...");
//   disableTCInterrupts();
//   // Take a couple steps to make sure it's working right
//   oneStep();
//   delay(SETTLE_TIME);
//   oneStep();
//   delay(READ_TIME);
//   for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
//     currentencoderReading = mod(readEncoder(),cpr);
//     encoderReading += currentencoderReading;
//     delay(READ_TIME);
//   }
//   encoderReading/=avg;
//   dir = CW;  // Take a step clockwise, wait for vibrations to settle
//   oneStep();
//   delay(SETTLE_TIME);
//   for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
//     currentencoderReading = mod(readEncoder(),cpr);
//     lastencoderReading += currentencoderReading;
//     delay(READ_TIME);
//   }
//   lastencoderReading/=avg;
//   // Take the difference
//   currentencoderReading = (lastencoderReading - encoderReading);
//   // Wired backwards if:
//   // 1) we see a rollover from low to high (should be high to low)
//   // 2) we see a small step lower (should be higher)
//   SerialUSB.println(String(currentencoderReading));
//   if(currentencoderReading > cpr/2 || currentencoderReading<0){
//     SerialUSB.println("Try again. If problem persists, swap wiring");
//     return CALIBRATION_FAIL;
//   }
//   while (stepNumber != 0) {       //go to step zero
//     if (stepNumber > 0) {
//       dir = CW;
//     }
//     else
//     {
//       dir = CCW;
//     }
//     oneStep();
//     delay(QUICK_SETTLE/2);
//   }
  
//   dir = CW;

//   for (int x = 0; x < spr; x++) {     //step through all full step positions, recording their encoder readings

//     encoderReading = 0;               // init. as 0 for averages
//     delay(QUICK_SETTLE);                        //moving too fast may not give accurate readings.  Motor needs time to settle after each step.
//     lastencoderReading = readEncoder();
        
//     for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
//       currentencoderReading = readEncoder();

//       // If we are on the edge of wrapping around, add
//       // or subtract as needed to keep the value correct
//       if ((currentencoderReading-lastencoderReading)<(-(cpr/2))){
//         currentencoderReading += cpr;
//       }
 
//       encoderReading += currentencoderReading;
//       delay(READ_TIME);
//     }
//     // Take the average
//     encoderReading = encoderReading / avg;
//     // Put it back in range of the 14 bit value
//     if (encoderReading>=cpr){
//       encoderReading-= cpr;
//     }
//     else if (encoderReading<0){
//       encoderReading+= cpr;
//     }

//     fullStepReadings[x] = encoderReading;
    
//     // go to next step
//     oneStep();
//   }
//   // Once we know everything, we can analyze the data.
//   findijStart(fullStepReadings, &iStart, &jStart);

//   // The code below generates the lookup table by intepolating between
//   // full steps and mapping each encoder count to a calibrated angle
//   // The lookup table is too big to store in volatile memory,
//   // so we must generate and store it into the flash on the fly

//   // begin the write to the calibration table
//   page_count = 0;
//   page_ptr = (const uint8_t*) lookup;
//   // Start counting at iStart
//   for (int i = iStart; i < (iStart + spr + 1); i++) {
//     ticks = fullStepReadings[mod((i + 1), spr)] - fullStepReadings[mod((i), spr)];

//     if (ticks < -cpr/2) {           //check if current interval wraps over encoder's zero positon
//       ticks += cpr;
//     }
//     if (i == iStart) { //this is an edge case
//       // starting at 0, go through the ticks and assign an angle
//       // given that 1 tick = 1 aps
//       // For this case, we only care about the vals between jStart and ticks
//       for (int j = jStart; j < (ticks); j++) {
// 	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
//       }
//     }
//     else if (i == (iStart + spr)) { //this is an edge case
//       // this time, we are ending at 0, making sure not to double-count
//       // the ones covered in the previous case
//       for (int j = 0; j < jStart; j++) {
// 	     store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
//       }
//     }
//     else {                        //this is the general case
//       for (int j = 0; j < ticks; j++) {
// 	      store_lookup(0.001 * mod(1000 * ((aps * i) + ((aps * j ) / float(ticks))), 360000.0));
//       }
//     }
//   }

//   // Store unwritten page
//   if (page_count != 0)
// 	write_page();
//   SerialUSB.println("The calibration table has been written to non-volatile Flash memory!");
//   return CALIBRATION_SUCCESS;
// }

// void findijStart(int readings[], int* istart, int* jstart){
//   int ticks;
//   int stepNo;
//   // We know readings[] will always be spr long
//   for(int i =0; i<spr; i++){
//     // Take the difference of two consecutive items (wrapping around)
//     ticks = readings[mod((i + 1), spr)] - readings[mod((i), spr)];
//     // if a step causes wrapping around, add as needed
//     if (ticks < -cpr/2) {
//       ticks += cpr;
//     }
//     // We now have all positive ticks (enforced by calibration routine)
//     for (int j = 0; j < ticks; j++){
//       // Interpolate between start and end val of the tick to find
//       // when the encoder would read 0
//       stepNo = (mod(readings[i] + j, cpr));
//       if(stepNo==0){
//         // Record the step number and 
//         *istart = i;
//         *jstart = j;
//         return;
//       }
//     }
//   }
//   return;
// }


// float read_angle()
// {
//   const int avg = 10;            //average a few readings
//   int encoderReading = 0;

//   disableTCInterrupts();        //can't use readEncoder while in closed loop

//   for (int reading = 0; reading < avg; reading++) {  //average multple readings at each step
//     encoderReading += readEncoder();
//     delay(10);
//     }

//   //return encoderReading * (360.0 / 16384.0) / avg;
//   return lookup[encoderReading / avg];
// }