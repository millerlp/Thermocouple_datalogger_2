/* Store_calibration.ino
 *  Luke Miller 2017
 *  
 *  Use this sketch to upload thermocouple channel calibration
 *  coefficients for a TempTracker_RevB board. After entering
 *  your calibration values in tcOffset and tcSlope, you only 
 *  need to upload this program to the TempTracker board and run
 *  it one time. Then upload the main datalogging program. 
 *  More explanation below.
 *  
 *  You will have determined calibration coefficients using an
 *  external calibrator such as an Omega CL3515R that feeds a
 *  known voltage into the TempTracker board. Use the sketch
 *  Calibration_routine_OLED_RevB to run these calibration trials.
 *  The calibration values should consist of a slope and intercept 
 *  (aka offset) for each MAX31856 channel. These values could be
 *  determined by fitting a linear regression between observed 
 *  temperature on datalogger (on the x-axis) and expected temperature
 *  from the calibrator (on the y-axis), using any basic program
 *  like Excel that fits straight-line regressions (aka trendline).
 *  
 *  Ideally the intercept is 0.0 and the slope is 1.0, but individual 
 *  MAX31856 chips may end up with slight
 *  deviations from this. In that case, this program will write your
 *  calibration data to the EEPROM memory on board the TempTracker
 *  so that it will be available when the main TempTracker datalogging
 *  program is used. You should only need to write the calibration 
 *  data to the TempTracker board 1 time using this program, and it
 *  will persist even when the power is removed from the TempTracker
 *  board. 
 * 
 * 
 */

#include <EEPROM.h>
#include "TClib2.h" // https://github.com/millerlp/TClib2

// Enter your calibation values here (offset = intercept, slope = slope) for each
// channel. If you lack values for any channels, leave that channel's offset value
// set to 0.00 and slope set to 1.00. 
//                      Ch0         Ch1       Ch2         Ch3       Ch4         Ch5       Ch6         Ch7                  
float tcOffset[] = { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000}; 
float tcSlope[] =  { 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000};

float readout = 0;

void setup()
{
 Serial.begin(57600);
 Serial.println("Resetting eeprom");
 for (uint16_t i = 0; i < (4*16); i++){  // reset 4bytes * 16 values to make space for float values
  EEPROM.write(i, 0xFF); // 0xFF is the factory-default value (= 255)
 }
 Serial.println("EEPROM cleared");
 Serial.println("Writing tcOffset to EEPROM");
 for (int i = 0; i < 8; i++){
  EEPROM_WriteFloat(&tcOffset[i], i);
 }
 Serial.println("Writing tcSlope to EEPROM");
 int shiftAddr = 8;
 for (int i = 0; i < 8; i++){
  EEPROM_WriteFloat(&tcSlope[i], (i+shiftAddr));
 } 

// Read the results back out to show that the writing worked
 for (int memEntry = 0; memEntry < 16; memEntry++){
  EEPROM_ReadFloat(&readout, memEntry);
  if (memEntry < 8){
    if (memEntry == 0) Serial.println("Checking tcOffset");
    if ( abs(readout - tcOffset[memEntry]) < 0.00001){
      // If the difference between readout and original tcOffset value
      // is small, this was successful.
      Serial.print(readout,DEC);
      Serial.print("\t");
      Serial.print( tcOffset[memEntry],DEC);
      Serial.println("\t MATCH");
    } else {
      Serial.print(readout,DEC);
      Serial.print("\t");
      Serial.print( tcOffset[memEntry],DEC);
      Serial.println("\t NO MATCH");
    }
  } else if (memEntry >= 8) {
    if (memEntry == 8) Serial.println("Checking tcSlope");
    // Now check against the tcSlope values
    if ( abs(readout - tcSlope[(memEntry-8)]) < 0.00001){
      // If the difference between readout and original tcSlope value
      // is small, this was successful.
      Serial.print(readout,DEC);
      Serial.print("\t");
      Serial.print( tcSlope[(memEntry-8)],DEC);
      Serial.println("\t MATCH");
    } else {
      Serial.print(readout,DEC);
      Serial.print("\t");
      Serial.print( tcSlope[(memEntry-8)],DEC);
      Serial.println("\t NO MATCH");
    }
  }
  
 }
 
} // end of setup()

void loop()
{
}



