/*  Calibration_routine_OLED.ino
 *   Luke Miller June 2017
 *   
 *   For Thermocouple_datalogger_RevB hardware, this 
 *   program will allow the user to calibrate the 
 *   board's individual MAX31865 thermocouple converter
 *   chips. This requires an external thermocouple
 *   calibrator (see Omega Engineering CL3515R for example)
 *   that can produce a known millivolt signal corresponding
 *   to a certain temperature. 
 *   
 *   The user will use buttons 1 and 2 to select
 *   a thermocouple channel to calibrate. Output will be
 *   written to csv files, including the known temperature,
 *   recorded temperature, and channel number.
 * 
 * 
 */


#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>  // built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>  // built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include <Adafruit_MAX31856.h> // https://github.com/millerlp/Adafruit_MAX31856

#include "TClib2.h" // My utility library for this project https://github.com/millerlp/TClib2/

#define ERRLED A2    // Red error LED pin
#define GREENLED A3   // Green LED pin
#define BUTTON1 2     // BUTTON1 on INT0, pin PD2
#define BUTTON2 3     // BUTTON2 on INT1, pin PD3
//*********************
#define SAMPLES 30    // number of samples per temperature
//*********************
// Create real time clock object
RTC_DS3231 rtc;
char buf[20]; // declare a string buffer to hold the time result
//******************************************
// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C
#define I2C_ADDRESS2 0x3D

SSD1306AsciiWire oled1; // create OLED display object, using I2C Wire
SSD1306AsciiWire oled2; // create OLED display object, using I2C Wire
//*************
// Create sd card objects
SdFat sd;
//SdFile logfile;  // for sd card, this is the file object to be written to
SdFile calibfile; // for sd card, this is the calibration file to write
const byte chipSelect = 10; // define the Chip Select pin for SD card

// Define MAX31855 chip select pins, need 8 of them for the 8 separate chips
#define CS_MAX0 A1 // Arduino pin PC1, analog 1, Chip Select for MAX31855 #0
#define CS_MAX1 A0 // Arduino pin PC0, analog 0, Chip Select for MAX31855 #1
#define CS_MAX2 7 // Arduino pin PD7, digital 7, Chip Select for MAX31855 #2
#define CS_MAX3 6 // Arduino pin PD6, digital 6, Chip Select for MAX31855 #3
#define CS_MAX4 5 // Arduino pin PD5, digital 5, Chip Select for MAX31855 #4
#define CS_MAX5 4 // Arduino pin PD4, digital 4, Chip Select for MAX31855 #5
#define CS_MAX6 9 // Arduino pin PB1, digital 9, Chip Select for MAX31855 #6
#define CS_MAX7 8 // Arduino pin PB0, digital 8, Chip Select for MAX31855 #7
//#define SCK    13
//#define SDI    11
//#define SDO    12
//#define CR0_INIT  (CR0_AUTOMATIC_CONVERSION + CR0_OPEN_CIRCUIT_FAULT_TYPE_K /* + CR0_NOISE_FILTER_50HZ */)
//#define CR1_INIT  (CR1_AVERAGE_2_SAMPLES + CR1_THERMOCOUPLE_TYPE_K)
//#define MASK_INIT (~(MASK_VOLTAGE_UNDER_OVER_FAULT + MASK_THERMOCOUPLE_OPEN_FAULT))
#define NUM_MAX31856  8 // 8 thermocouple channels

//Adafruit_MAX31856 max0 = Adafruit_MAX31856(CS_MAX0);

// Create the temperature object, defining the pins used for communication
Adafruit_MAX31856 *TempSensor[NUM_MAX31856] = {
  new Adafruit_MAX31856(CS_MAX0),
  new Adafruit_MAX31856(CS_MAX1),
  new Adafruit_MAX31856(CS_MAX2),
  new Adafruit_MAX31856(CS_MAX3),
  new Adafruit_MAX31856(CS_MAX4),
  new Adafruit_MAX31856(CS_MAX5),
  new Adafruit_MAX31856(CS_MAX6),
  new Adafruit_MAX31856(CS_MAX7)
};

double temp[SAMPLES]; // temperature array

// Define the set of simulated temperatures that will be provided
// by the thermocouple calibrator
byte testTemps[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};
byte tempCounter = 0;

// Declare initial name for output files written to SD card
char calibFilename[] = "YYYYMMDD_HHMM_00_ChX_calib.csv";

byte loopCount = 0; // counter to keep track of data sampling loops
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
int Channel;
float targetTemp;
bool continueCalib = true;
char choice;
int debounceTime = 20; // milliseconds

//********************************************************* 
void setup() {
  // Set button1 as an input
  pinMode(BUTTON1, INPUT_PULLUP);
  // Set button2 as an input
  pinMode(BUTTON2, INPUT_PULLUP);
    // Set up the LEDs as output
  pinMode(ERRLED,OUTPUT);
  digitalWrite(ERRLED, LOW);
  pinMode(GREENLED,OUTPUT);
  digitalWrite(GREENLED, LOW);
  // Set the SPI bus chip select pins as outputs
  // for the MAX31856 thermocouple chips
  pinMode(CS_MAX0, OUTPUT);
  digitalWrite(CS_MAX0, HIGH);
  pinMode(CS_MAX1, OUTPUT);
  digitalWrite(CS_MAX1, HIGH);
  pinMode(CS_MAX2, OUTPUT);
  digitalWrite(CS_MAX2, HIGH);
  pinMode(CS_MAX3, OUTPUT);
  digitalWrite(CS_MAX3, HIGH);
  pinMode(CS_MAX4, OUTPUT);
  digitalWrite(CS_MAX4, HIGH);
  pinMode(CS_MAX5, OUTPUT);
  digitalWrite(CS_MAX5, HIGH);
  pinMode(CS_MAX6, OUTPUT);
  digitalWrite(CS_MAX6, HIGH);
  pinMode(CS_MAX7, OUTPUT);
  digitalWrite(CS_MAX7, HIGH);
  // Flash green LED to show that we just booted up
  for (byte i = 0; i < 3; i++){
    digitalWrite(GREENLED, HIGH);
    delay(100);
    digitalWrite(GREENLED, LOW);
    delay(100);
  } 
  Serial.begin(57600);
  Serial.println(F("Hello")); 

//-----------------------------------------------------
// Initialize the MAX31856 thermocouple chips  

//  // Initializing the MAX31855's registers
//  for (int i=0; i<NUM_MAX31856; i++) {
//    TempSensor[i]->writeRegister(REGISTER_CR0, CR0_INIT);
//    TempSensor[i]->writeRegister(REGISTER_CR1, CR1_INIT);
//    TempSensor[i]->writeRegister(REGISTER_MASK, MASK_INIT);
//  }

  for (int i=0; i<NUM_MAX31856;i++){
    TempSensor[i]->begin();
    TempSensor[i]->setThermocoupleType(MAX31856_TCTYPE_K);
  }

  // Read back the thermocouple type to make sure it was set successfully
  Serial.print("Thermocouple type: "); 
  for (int i=0; i<NUM_MAX31856;i++){
    switch ( TempSensor[i]->getThermocoupleType() ) {  // Read TC type
      case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
      case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
      case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
      case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
      case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
      case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
      case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
      case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
      case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
      case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
      default: Serial.println("Unknown"); break;
    }
  }

// Initialize the real time clock DS3231M
  Wire.begin(); // Start the I2C library with default options
  rtc.begin();  // Start the rtc object with default options
  newtime = rtc.now(); // read a time from the real time clock
  newtime.toString(buf, 20); 
  // Now extract the time by making another character pointer that
  // is advanced 10 places into buf to skip over the date. 
  char *timebuf = buf + 10;
  printTimeSerial(rtc.now()); // print time to serial monitor
  Serial.println();

  //----------------------------------
  // Start up the oled displays
  oled1.begin(&Adafruit128x64, I2C_ADDRESS1);
  oled1.set400kHz();  
  oled1.setFont(Adafruit5x7);    
  oled1.clear(); 
  oled1.print(F("Hello"));
  // Start up the 2nd oled display
  oled2.begin(&Adafruit128x64, I2C_ADDRESS2);
  oled2.set400kHz();  
  oled2.setFont(Adafruit5x7);    
  oled2.clear();
  //----------------------------------
  // Display Date on OLED1
  oled1.println();
  oled1.set2X();
  for (int i = 0; i<11; i++){
    oled1.print(buf[i]);
  }
  // Display Time on OLED2
  oled2.println();
  oled2.set2X();
  oled2.println(timebuf);
  delay(1000);
  //*************************************************************
  // SD card setup and read (assumes Serial output is functional already)
  pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
  // Initialize the SD card object
  // Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
  // errors on a breadboard setup. 
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
  // If the above statement returns FALSE after trying to 
  // initialize the card, enter into this section and
  // hold in an infinite loop.
        // There is an error with the SD card, halt everything
    // Consider removing this while loop and allowing user to plow
    // ahead without card (use button input?)
    while(1){ // infinite loop due to SD card initialization error
                        digitalWrite(ERRLED, HIGH);
                        delay(100);
                        digitalWrite(ERRLED, LOW);
                        digitalWrite(GREENLED, HIGH);
                        delay(100);
                        digitalWrite(GREENLED, LOW);
                        oled1.println();
                        oled1.print(F("SD CARD"));
                        oled2.print(F("ERROR"));
    }
  }
  
  bool channelSelectFlag = true;
  byte currentChannel = 0;
  oled1.home();
  oled1.clear();
  oled1.println(F("Choose"));
  oled1.println(F("Channel:"));
  oled1.print(currentChannel);
  oled2.home();
  oled2.clear();
  oled2.set1X();
  oled2.println(F("Press BUTTON1"));
  oled2.println(F("to select"));
  oled2.println(F("Press BUTTON2"));
  oled2.println(F("to start"));

  while (channelSelectFlag){
    if(digitalRead(BUTTON1) == LOW){
      // Button is pressed
      delay(debounceTime);
      if (digitalRead(BUTTON1) == LOW){
        // If button is still pressed, now wait for a high signal
        while (digitalRead(BUTTON1) == LOW){delay(debounceTime);};
        // Now that button in released, increment channel
        ++currentChannel;
        if (currentChannel > 7) currentChannel = 0; // reset to 0
        oled1.setCursor(0,4);
        oled1.clearToEOL();
        oled1.print(currentChannel);
      }
    } else {
      delay(50);
    }
    if (digitalRead(BUTTON2) == LOW){
      // Button 2 is pressed
      delay(debounceTime);
      if (digitalRead(BUTTON2) == LOW){
        // If button is still pressed, wait for a high signal
        while (digitalRead(BUTTON2) == LOW) {delay(debounceTime);};
        channelSelectFlag = false;
        Channel = currentChannel;
      }
    }
  }

  oled1.home();
  oled1.clear();
  oled1.println(F("Using"));
  oled1.println(F("channel"));
  oled1.print(currentChannel);
  delay(1000);

  oled2.home();
  oled2.clear();
  oled2.set2X();
  oled2.println(F("Set calib"));
  oled2.println(F("temp to"));
  oled2.println(F("0C"));
  delay(2000);
  

  // Initialize a new output file
  newtime = rtc.now();
  initCalibFile(newtime, Channel);
  Serial.print(F("Writing to file: "));
  Serial.println(calibFilename);
  

    
} // end of setup() loop

//************************************************
void loop() {
  // Loop through each of the calibration temperatures stored in testTemps
  // Initially the routine will pause and wait for the user to hit 
  // Button1 before storing a set of measurements. The while loop will cycle
  // until the continueCalib flag is set false. 
  while (continueCalib & (tempCounter < (sizeof(testTemps)/sizeof(byte)))){
      oled1.home();
      oled1.clear();
      oled1.set2X();
      oled1.println(F("Set calib"));
      oled1.print(F("temp to "));
      oled1.print(testTemps[tempCounter]);
      oled1.println(F("C"));
      oled1.println(); // add empty line
      oled1.print(F("Ch"));
      oled1.print(Channel);
      oled1.print(F(" "));
      
      // Switch to screen 2
      oled2.home();
      oled2.clear();
      oled2.set2X();
      oled2.println(F("Press"));
      oled2.println(F("BUTTON1"));
      oled2.println(F("to start"));
      oled2.set1X();
      oled2.setCursor(0,9);
      oled2.print(F("Internal temp: "));

      delay(200);      
      
      bool buttonFlag = false;
      double currTemp = 0;
      double internalTemp = 0;
      unsigned long oldMillis = millis();
      // Wait for start signal
      while (!buttonFlag){
           
        // Check the button
        if(digitalRead(BUTTON1) == LOW){
          // Button is pressed
          delay(debounceTime);
          if (digitalRead(BUTTON1) == LOW){
            // If button is still pressed, now wait for a high signal
            while (digitalRead(BUTTON1) == LOW){delay(debounceTime);};
            // Now that button in released, set buttonFlag true to kill loop
            buttonFlag = true;
          }
        } else {
          // Button is not pressed, update OLED screens
          // every 500 ms. 
          if ( (millis() - oldMillis) > 500){
            oldMillis = millis(); // update oldMillis
             // Take a reading to show on the screen
            currTemp = TempSensor[Channel]->readThermocoupleTemperature();
            internalTemp = TempSensor[Channel]->readCJTemperature(); 
//            switch(Channel){
//              case 0:
//                currTemp = max0.readThermocoupleTemperature();
//                internalTemp = max0.readCJTemperature();
////                currTemp = TemperatureSensor[0]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[0]->readJunction(CELSIUS);
//              break;
//              case 1:
////                currTemp = TemperatureSensor[1]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[1]->readJunction(CELSIUS);
//              break;
//              case 2:
////                currTemp = TemperatureSensor[2]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[2]->readJunction(CELSIUS);
//              break;
//              case 3:
////                currTemp = TemperatureSensor[3]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[3]->readJunction(CELSIUS);
//              break;
//              case 4:
////                currTemp = TemperatureSensor[4]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[4]->readJunction(CELSIUS);
//              break;
//              case 5:
////                currTemp = TemperatureSensor[5]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[5]->readJunction(CELSIUS);
//              break;
//              case 6:
////                currTemp = TemperatureSensor[6]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[6]->readJunction(CELSIUS);
//              break;            
//              case 7:
////                currTemp = TemperatureSensor[7]->readThermocouple(CELSIUS);
////                internalTemp = TemperatureSensor[7]->readJunction(CELSIUS);
//              break;
//            }
            // Apply the temperature correction. Function is in 
            // the library TClib2
//            double correctedTemp = correctTemp(currTemp, internalTemp);
            
            // Update the screen
            oled1.set2X();
            oled1.clear(48,128,6,7);
//            oled1.print(correctedTemp);
            oled1.print(currTemp);
            oled1.print(F(" C"));

            oled2.set1X();
            oled2.setCursor(0,7);
            oled2.clearToEOL();
            oled2.print(internalTemp,4);
            oled2.print(F(" C"));
            oled2.set2X();

            Serial.print("currTemp: ");
            Serial.print(currTemp);
            Serial.print("  Junction: ");
            Serial.println(internalTemp);
          }
//          digitalWrite(GREENLED, !digitalRead(GREENLED)); // Flash to notify user
          delay(50);
        }
    } // end of while (!buttonFlag)
    
    targetTemp = testTemps[tempCounter];

    // Update screens
    oled1.home();
    oled1.clear();
    oled1.println(F("Target:"));
    oled1.print(targetTemp);
    oled1.println(F(" C"));
    oled2.home();
    oled2.clear();
    oled2.set2X();
    oled2.println(F("Reading:"));

    Serial.println(F("Target\ttemperature"));

    // Now read appropriate channel
    for (int i = 0; i < SAMPLES; i++){
      temp[i] = TempSensor[Channel]->readThermocoupleTemperature();
      oled2.setCursor(0,2);
      oled2.clearToEOL();
      oled2.print(temp[i]);
      Serial.print(targetTemp);
      Serial.print(F("\t"));
      Serial.println(temp[i]);
      delay(500);  
    }
//    switch(Channel){
//      case 0:
//        for (int i = 0; i < SAMPLES; i++){
//          
//          temp[i] = max0.readThermocoupleTemperature();
////          temp[i] = TemperatureSensor[0]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple0.readCelsius(),thermocouple0.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 1:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[1]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple1.readCelsius(),thermocouple1.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 2:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[2]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple2.readCelsius(),thermocouple2.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break; 
//      case 3:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[3]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple3.readCelsius(),thermocouple3.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 4:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[4]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple4.readCelsius(),thermocouple4.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 5:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[5]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple5.readCelsius(),thermocouple5.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 6:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[6]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple6.readCelsius(),thermocouple6.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;
//      case 7:
//        for (int i = 0; i < SAMPLES; i++){
////          temp[i] = TemperatureSensor[7]->readThermocouple(CELSIUS);
////          temp[i] = correctTemp(thermocouple7.readCelsius(),thermocouple7.readInternal());
//          oled2.setCursor(0,2);
//          oled2.clearToEOL();
//          oled2.print(temp[i]);          
//          Serial.print(targetTemp);
//          Serial.print(F("\t"));
//          Serial.println(temp[i]);
//          delay(500);         
//        }
//      break;                                       
//    }
    // Write temp array to SD card
    writeCalibSD(targetTemp,temp);
    // Increment tempCounter
    ++tempCounter;
    Serial.println(F("Moving to next temperature"));

    
  } // end of while (continueCalib)
  
  // The while loop above should end when the last
  // temperature has been run, so finish by closing 
  // the SD card file. 

  // Clean up and go into suspension
  calibfile.close();
  Serial.print(F("Saved file: "));
  Serial.println(calibFilename);
  Serial.println(F("Reset to calibrate again"));

  oled1.home();
  oled1.clear();
  oled1.println(F("Finished"));
  oled1.set1X();
  oled1.println(F("output file:"));
  oled1.print(calibFilename);
  oled2.home();
  oled2.clear();
  oled2.println(F("Reset to"));
  oled2.println(F("calibrate"));
  oled2.println(F("again"));
  
  while(1); // infinite loop  

  

} // end of main loop()


//---------writeCalibSD-----------------------
// A function to write the current targetTemp and temperature to the 
// csv file
void writeCalibSD (float targetTemp, double *temp){
    // Reopen logfile. If opening fails, notify the user
  if (!calibfile.isOpen()) {
    if (!calibfile.open(calibFilename, O_RDWR | O_CREAT | O_AT_END)) {
      digitalWrite(ERRLED, HIGH); // turn on error LED
    }
  }
  for (int i = 0; i < SAMPLES; i++){
    calibfile.print(targetTemp); // repeat this for each entry
    calibfile.print(F(","));
    calibfile.print(temp[i],3);  // write 3 sig figs   
    calibfile.print(F(","));
    calibfile.println(Channel);
  }
}

//------------ initCalibFile --------------------------------------------------------
// A function to generate a new filename based on the current date and time
// This function will make sure there isn't already a file of the same name
// in existence, then create a new file. 
// filename format: YYYYMMDD_HHMM_00_ChX_calib.csv
//                  0123456789
void initCalibFile(DateTime time1, byte Channel){
    char buf[5];
    // integer to ascii function itoa(), supplied with numeric year value,
    // a buffer to hold output, and the base for the conversion (base 10 here)
    itoa(time1.year(), buf, 10);
      // copy the ascii year into the filename array

                byte count = 0;
    for (byte i = 0; i < 4; i++){
      calibFilename[i] = buf[count];
                        count++;
    }
    // Insert the month value
    if (time1.month() < 10) {
      calibFilename[4] = '0';
      calibFilename[5] = time1.month() + '0';
    } else if (time1.month() >= 10) {
      calibFilename[4] = (time1.month() / 10) + '0';
      calibFilename[5] = (time1.month() % 10) + '0';
    }
    // Insert the day value
    if (time1.day() < 10) {
      calibFilename[6] = '0';
      calibFilename[7] = time1.day() + '0';
    } else if (time1.day() >= 10) {
      calibFilename[6] = (time1.day() / 10) + '0';
      calibFilename[7] = (time1.day() % 10) + '0';
    }
    // Insert an underscore between date and time
    calibFilename[8] = '_';
    // Insert the hour
    if (time1.hour() < 10) {
      calibFilename[9] = '0';
      calibFilename[10] = time1.hour() + '0';
    } else if (time1.hour() >= 10) {
      calibFilename[9] = (time1.hour() / 10) + '0';
      calibFilename[10] = (time1.hour() % 10) + '0';
    }
    // Insert minutes
      if (time1.minute() < 10) {
      calibFilename[11] = '0';
      calibFilename[12] = time1.minute() + '0';
    } else if (time1.minute() >= 10) {
      calibFilename[11] = (time1.minute() / 10) + '0';
      calibFilename[12] = (time1.minute() % 10) + '0';
    }
    // Insert another underscore after time
    calibFilename[13] = '_';
    
    // Next change the counter on the end of the filename
    // (digits 18+19) to increment count for files generated on
    // the same day. This shouldn't come into play
    // during a normal data run, but can be useful when 
    // troubleshooting.
    for (uint8_t i = 0; i < 100; i++) {
      calibFilename[14] = i / 10 + '0';
      calibFilename[15] = i % 10 + '0';
      calibFilename[16] = '_';
      calibFilename[17] = 'C';
      calibFilename[18] = 'h';
      calibFilename[19] = Channel + '0';
      calibFilename[20] = '_';
      calibFilename[21] = 'c';
      calibFilename[22] = 'a';
      calibFilename[23] = 'l';
      calibFilename[24] = 'i';
      calibFilename[25] = 'b';
      calibFilename[26] = '.';
      calibFilename[27] = 'c';
      calibFilename[28] = 's';
      calibFilename[29] = 'v';      
      // Now check to see if this filename is already on SD card
      if (!sd.exists(calibFilename)) {
        // when sd.exists() returns false, this block
        // of code will be executed to open the file
        if (!calibfile.open(calibFilename, O_RDWR | O_CREAT | O_AT_END)) {
          // If there is an error opening the file, notify the user
            Serial.print("Couldn't open file ");
            Serial.println(calibFilename);
          
          while (1) {
            // Just idle here flashing the error LED until the
            // user fixes things and restarts the program. 
            digitalWrite(ERRLED, HIGH);
            delay(100);
            digitalWrite(ERRLED, LOW);
            delay(100);
          }
        }
        break; // Break out of the for loop when the
        // statement if(!logfile.exists())
        // is finally false (i.e. you found a new file name to use).
      } // end of if(!sd.exists())
    } // end of file-naming for loop

        calibfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
                                    time1.hour(), time1.minute(), time1.second());
        calibfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
                                    time1.hour(), time1.minute(), time1.second());
        calibfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
                                    time1.hour(), time1.minute(), time1.second());        
        // Write column headers
        calibfile.println(F("TargetTempC,BoardTempC,Channel"));
        calibfile.close();
}

