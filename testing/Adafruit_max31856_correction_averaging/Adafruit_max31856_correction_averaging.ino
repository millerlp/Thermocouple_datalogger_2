/* Test program to make sure calibration corrections and
 *  averaging are carried out correctly.
 */

#include <Adafruit_MAX31856.h> // Use my fork: https://github.com/millerlp/Adafruit_MAX31856

// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 max = Adafruit_MAX31856(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 max = Adafruit_MAX31856(A1);

#define AVG_WINDOW 4
#define NUM_MAX31856 1
// Create the TempSensor object, defining the pins used for communication
Adafruit_MAX31856 *TempSensor[NUM_MAX31856] = {
  new Adafruit_MAX31856(A0),
};


// Test correction values
double tcOffset = -0.2441172;
double tcSlope = 1.0018771;
double tempArray[AVG_WINDOW][NUM_MAX31856];
double tempArray2[AVG_WINDOW][NUM_MAX31856];
double tempAverages[NUM_MAX31856]; 
double tempAverages2[NUM_MAX31856]; 
byte loopCount = 0;

void setup() {
  Serial.begin(57600);
  Serial.println("MAX31856 thermocouple test");

  // Initialize the temperature sensor chips
  for (int i=0; i<NUM_MAX31856;i++){
    TempSensor[i]->begin(); // Initialize SPI and MAX31856 registers
    // Set thermocouple type to K (default)
    TempSensor[i]->setThermocoupleType(MAX31856_TCTYPE_K);
  }
 

//  Serial.print("Thermocouple type: ");
//  switch ( max.getThermocoupleType() ) {
//    case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
//    case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
//    case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
//    case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
//    case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
//    case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
//    case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
//    case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
//    case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
//    case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
//    default: Serial.println("Unknown"); break;
//  }


}

void loop() {
    
    for (byte Channel = 0; Channel < NUM_MAX31856; Channel++){
      // Read the temperature from the sensor
      tempArray[loopCount][Channel] = TempSensor[Channel]->readThermocoupleTemperature();
      // Now make a version with the calibration applied
      tempArray2[loopCount][Channel] = ((tempArray[loopCount][Channel]) * tcSlope) + tcOffset;
    }
    if (loopCount == 3){
        // Calculate the average of the readings for each sensor i
        for (byte Channel = 0; Channel < NUM_MAX31856; Channel++){
          double tempsum = 0;
          double tempsum2 = 0;
          for (byte j = 0; j < AVG_WINDOW; j++){
            // Add up j measurements for sensor 
            tempsum = tempsum + tempArray[j][Channel];
            // Do the same for the calibrated values
            tempsum2 = tempsum2 + tempArray2[j][Channel];
          }
          // Calculate average temperature for sensor i
          tempAverages[Channel] = tempsum / double(AVG_WINDOW); // cast denominator as double
          tempAverages2[Channel] = tempsum2 / double(AVG_WINDOW); // cast denominator as double
        }
    
      Serial.print("Raw temps: ");
      for (int i = 0; i<AVG_WINDOW; i++){
        Serial.print(tempArray[i][0]);
        Serial.print("\t");
      }
      Serial.print("Cal temps: ");
      for (int i = 0; i<AVG_WINDOW; i++){
        Serial.print(tempArray2[i][0]);
        Serial.print("\t");
      }
      Serial.print("Avg raw: ");
      Serial.print(tempAverages[0]);
      Serial.print("\t Avg cal: ");
      Serial.println(tempAverages2[0]);
    }
//  Serial.print("Cold Junction Temp: "); Serial.println(max.readCJTemperature());

//  Serial.print("Thermocouple Temp: "); Serial.println(max.readThermocoupleTemperature());
  // Check and print any faults
//  uint8_t fault = max.readFault();
//  if (fault) {
//    if (fault & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
//    if (fault & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
//    if (fault & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
//    if (fault & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
//    if (fault & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
//    if (fault & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
//    if (fault & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
//    if (fault & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
//  }
  delay(250);

  loopCount++;
  if (loopCount >= AVG_WINDOW){
    loopCount = 0; // Reset after 4 loops
  }
}
