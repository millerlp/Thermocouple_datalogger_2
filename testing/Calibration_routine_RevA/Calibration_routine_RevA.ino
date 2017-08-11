/*  Calibration_routine.ino
 *   Luke Miller June 2017
 *   
 *   For Thermocouple_datalogger_RevA hardware, this 
 *   program will allow the user to calibrate the 
 *   board's individual MAX31855 thermocouple converter
 *   chips. This requires an external thermocouple
 *   calibrator (see Omega Engineering CL3515R for example)
 *   that can produce a known millivolt signal corresponding
 *   to a certain temperature. 
 *   
 *   The user will use the serial monitor to select
 *   a thermocouple channel to calibrate, and to input
 *   the known temperature value. Output will be
 *   written to csv files, including the known temperature
 *   and recorded temperature.
 * 
 * 
 */

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>  // built in library, for I2C communications
#include <SPI.h>  // built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "TClib2.h" // My utility library for this project

#define ERRLED A2    // Red error LED pin
#define GREENLED A3   // Green LED pin
#define BUTTON1 2     // BUTTON1 on INT0, pin PD2
#define BUTTON2 3     // BUTTON2 on INT1, pin PD3
//*********************
#define SAMPLES 5    // number of samples per temperature
//*********************
// Create real time clock object
RTC_DS3231 rtc;
char buf[20]; // declare a string buffer to hold the time result

//*************
// Create sd card objects
SdFat sd;
//SdFile logfile;  // for sd card, this is the file object to be written to
SdFile calibfile; // for sd card, this is the calibration file to write
const byte chipSelect = 10; // define the Chip Select pin for SD card

// Define MAX31855 objects, need 8 of them for the 8 separate chips
#define CS_MAX0 A1 // Arduino pin PC1, analog 1, Chip Select for MAX31855 #0
#define CS_MAX1 A0 // Arduino pin PC0, analog 0, Chip Select for MAX31855 #1
#define CS_MAX2 7 // Arduino pin PD7, digital 7, Chip Select for MAX31855 #2
#define CS_MAX3 6 // Arduino pin PD6, digital 6, Chip Select for MAX31855 #3
#define CS_MAX4 5 // Arduino pin PD5, digital 5, Chip Select for MAX31855 #4
#define CS_MAX5 4 // Arduino pin PD4, digital 4, Chip Select for MAX31855 #5
#define CS_MAX6 9 // Arduino pin PB1, digital 9, Chip Select for MAX31855 #6
#define CS_MAX7 8 // Arduino pin PB0, digital 8, Chip Select for MAX31855 #7

Adafruit_MAX31855 thermocouple0(CS_MAX0);
Adafruit_MAX31855 thermocouple1(CS_MAX1);
Adafruit_MAX31855 thermocouple2(CS_MAX2);
Adafruit_MAX31855 thermocouple3(CS_MAX3);
Adafruit_MAX31855 thermocouple4(CS_MAX4);
Adafruit_MAX31855 thermocouple5(CS_MAX5);
Adafruit_MAX31855 thermocouple6(CS_MAX6);
Adafruit_MAX31855 thermocouple7(CS_MAX7);
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
  // for the MAX31855 thermocouple chips
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
//  Serial.println(F("Make sure your terminal is set to No line ending"));

// Initialize the real time clock DS3231M
  Wire.begin(); // Start the I2C library with default options
  rtc.begin();  // Start the rtc object with default options
  newtime = rtc.now(); // read a time from the real time clock
  newtime.toString(buf, 20); 
  printTimeSerial(rtc.now()); // print time to serial monitor
  Serial.println();
  
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
    }
  }
  
  bool channelSelectFlag = false;
  
  while (channelSelectFlag == false){
    Serial.println("Please enter a channel to calibrate (0 to 7):");
    while (!Serial.available());
      if (Serial.available() > 0) {
          // Convert 1st input ascii character to a numeric value (0 to 7)
          Channel = Serial.parseInt();;
          if ( (Channel >= 0) | (Channel <= 7) ){
            Serial.print("Channel ");
            Serial.println(Channel);
            channelSelectFlag = true; // kills while loop
          } else {
            Serial.println("Whoops, please enter value 0 to 7.");
          }
          // Empty the serial buffer
          while (Serial.available() > 0 ){
            char junk = Serial.read();
            delay(5);
          }
      } // end of if (Serial.available() > 0) {
  } // end of while (channelSelectFlag == false)

  // Initialize a new output file
  newtime = rtc.now();
  initCalibFile(newtime, Channel);
  Serial.print(F("Writing to file: "));
  Serial.println(calibFilename);
  
  
    
} // end of setup() loop

//************************************************
void loop() {
  
  while (continueCalib & (tempCounter < (sizeof(testTemps)/sizeof(byte)))){
    Serial.print(F("Set calibrator to "));
    Serial.print(testTemps[tempCounter]);
    Serial.println(F(" C, and hit enter to start"));
    
    while  (Serial.available()<=0) {}; // wait for user response
    if (Serial.available() > 0){
      // Empty the serial buffer
      while (Serial.available() > 0 ){
        char junk = Serial.read();
        Serial.println(junk);
        delay(5);
      }
      // Get next test temperature
      targetTemp = testTemps[tempCounter];
      Serial.print(F("Target temperature: \t"));
      Serial.print(targetTemp);
      Serial.println(F(" C"));
    }
    // Now read appropriate channel
    switch(Channel){
      case 0:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple0.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 1:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple1.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 2:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple2.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break; 
      case 3:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple3.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 4:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple4.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 5:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple5.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 6:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple6.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;
      case 7:
        for (int i = 0; i < SAMPLES; i++){
          temp[i] = thermocouple7.readCelsius();
          Serial.print(targetTemp);
          Serial.print(F("\t"));
          Serial.println(temp[i]);
          delay(500);         
        }
      break;                                       
    }
    // Write temp array to SD card
    writeCalibSD(targetTemp,temp);
    // Increment tempCounter
    ++tempCounter;

//    while (Serial.read() >= 0) {};
//    // Finished taking data, ask user to continue or quit
//    Serial.println(F("Continue or quit? y or q: "));
//    while (!Serial.available()) ; // wait for response
//    if (Serial.available() > 0 ) {
//        choice = Serial.read();
//       if (choice == 'y'){
//          continueCalib = true;
//       } else if (choice == 'q') {
//          continueCalib = false; // kill this inner loop
//       }
//    }
//    while (Serial.read() >= 0) {}; // flush out buffer
    
  } // end of while (continueCalib)
    // The while loop above should end when the last
    // temperature has been run, so finish by closing 
    // the SD card file. 

//  if (!continueCalib){
    // Clean up and go into suspension
    calibfile.close();
    Serial.print(F("Saved file: "));
    Serial.println(calibFilename);
    Serial.println(F("Reset to calibrate again"));
    while(1); // infinite loop  
//  }
  

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
    calibfile.print(temp[i]);     
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

