/* EEPROM_read_test.ino
 *  Luke Miller 2017
 *  
 *  Use this sketch to read thermocouple channel calibration
 *  coefficients for a TempTracker_RevB board. See explanation below.
 *  
 * 
 * 
 * 
 */

#include <EEPROM.h>

// Fill two arrays with the default values for cases where a calibration is missing
//                      Ch0         Ch1       Ch2         Ch3       Ch4         Ch5       Ch6         Ch7                  
float tcOffset[] = { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000}; 
float tcSlope[] =  { 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000};

float readout = 0;

void setup()
{
 Serial.begin(57600);

 Serial.println("Reading tcOffset from EEPROM");
  
 for (int memEntry = 0; memEntry < 8; memEntry++){
  // Read eeprom at memEntry address and store in readout
  EEPROM_ReadFloat(&readout, memEntry);   
  // Handle cases where the value coming back is 0xFFFFFFFF
  // which is what happens if EEPROM was never written to.
  if (isnan(readout)){
    // If true, there was no value stored, so keep the 
    // default value in tcOffset
  } else if (!isnan(readout)){
    // Stick the value in the tcOffset array
    tcOffset[memEntry] = readout;
  }
 }
 Serial.println("Reading tcSlope from EEPROM");
  // Now read out memory positions 8-16, which should contain
  // slope values for the calibration.
  for (int memEntry = 8; memEntry < 16; memEntry++){
  // Read eeprom at memEntry address and store in readout
  EEPROM_ReadFloat(&readout, memEntry);   
  // Handle cases where the value coming back is 0xFFFFFFFF
  // which is what happens if EEPROM was never written to.
  if (isnan(readout)){
    // If true, there was no value stored, so keep the 
    // default value in tcOffset
  } else if (!isnan(readout)){
    // Stick the value in the tcOffset array
    tcSlope[memEntry-8] = readout;
  }
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


