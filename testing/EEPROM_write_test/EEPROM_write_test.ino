/* Store_calibration.ino
 *  Luke Miller 2017
 *  
 *  Use this sketch to upload thermocouple channel calibration
 *  coefficients for a TempTracker_RevB board. See explanation below.
 *  
 *  You will have determined calibration coefficients using an
 *  external calibrator such as an Omega CL3515R that feeds a
 *  known voltage into the TempTracker board. Use the sketch
 *  Calibration_routine_OLED_RevB to run these calibration trials.
 *  
 *  After you have fit a linear regression to the calibration data
 *  for each channel, you should have a slope and an intercept value
 *  for each of the 8 channels. Ideally the intercept is 0.0 and the 
 *  slope is 1.0, but individual MAX31856 chips may end up with slight
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


//                      Ch0         Ch1       Ch2         Ch3       Ch4         Ch5       Ch6         Ch7                  
float tcOffset[] = { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000}; 
float tcSlope[] =  { 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000};

float readout = 0;

void setup()
{
 Serial.begin(57600);
 Serial.println("Resetting eeprom");
 for (uint16_t i = 0; i < EEPROM.length(); i++){
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


 for (int memEntry = 0; memEntry < 16; memEntry++){
  EEPROM_ReadFloat(&readout, memEntry);   
  Serial.println(readout, DEC);   
 }
 
} // end of setup()

void loop()
{
}


//**********EEPROM_WriteFloat***************************
// Function to write a float value (4 bytes) to EEPROM
void EEPROM_WriteFloat(float *num, int MemPos)
{
 byte ByteArray[4];
 memcpy(ByteArray, num, 4);
 for(int x = 0; x < 4; x++)
 {
   EEPROM.write((MemPos * 4) + x, ByteArray[x]);
 }  
}
//*********EEPROM_ReadFloat*********************************
// Function to read back a float value (4 bytes) from EEPROM
void EEPROM_ReadFloat(float *num, int MemPos)
{
 byte ByteArray[4];
 for(int x = 0; x < 4; x++)
 {
   ByteArray[x] = EEPROM.read((MemPos * 4) + x);    
 }
 memcpy(num, ByteArray, 4);
}


