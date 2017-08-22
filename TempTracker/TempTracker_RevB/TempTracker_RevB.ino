/* TempTracker_RevB.ino
	Copyright Luke Miller 2017

  TODO: Implement screen power down and button to wake screens if a longer
  sampling interval than 5 seconds is selected. This change would also require
  altering the MAX31856 library to change from autoconvert to one-shot mode.
  
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	***********************************************************************
	
  Code to run a Thermocouple_datalogger_RevB board with
  MAX31856 thermocouple sensors. By default
  this program will start running and saving data to the SD 
  card automatically. 

  To see current data file name while datalogging is going on, press 
  Button1 for >1 second and release. The filename will be displayed
  on the OLED screen for 5 seconds. 
  	
	Closing a data file safely:
		1. Hold button 1 down for at least 7 seconds, then release it.
		2. The green LED should flash rapidly 15 times. The previous data
			file will be closed safely, and a new file will be started in
			normal data collection mode. 
  
	Board error codes:
		Red flashes quickly (10Hz): Real Time Clock not set
		Red + Green alternate rapidly: SD card not found
                Red permanently on: SD card not found after data collection started
		Red flash every ~8 seconds: In brownout mode, data collection has stopped


   If you get a compiler warning (not error) involving
   "static EEPROMClass EEPROM;", you may safely ignore it. 

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>	// built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>	// built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include <Adafruit_MAX31856.h> // https://github.com/millerlp/Adafruit_MAX31856
#include "TClib2.h" // My utility library for this project https://github.com/millerlp/TClib2

// Various additional libraries for access to sleep mode functions
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
//******************************
// Data collection rate, enter a value here of 4, 2, or 1 (samples per second)
#define SAMPLES_PER_SECOND 2 // number of samples taken per second (4, 2, or 1)
//******************************
// Define the number of samples per sensor to average (moving average window)
#define AVG_WINDOW 4  // Just a number, no units
// Define the interval (seconds) between saved values (writing to SD card)
#define SAVE_INTERVAL 5 // units of seconds

//********************************
// tcOffset array contains individual calibration adjustments for each
// of the 8 MAX31856 chips on the board being programmed. tcSlope contains
// calibration slope adjustments in the same manner. These values 
// will have been determined with the help of the Calibration_routine_OLED.ino
// program and an external thermocouple calibrator such as an Omega CL3515R.
// The values were generated from a linear regression fit through the sensor
// temperatures and the known calibrator temperatures. 

// Default values, if there is no calibration done, should be 0 for tcOffset
// and 1.0 for tcSlope. Do not change these, instead if you have calibration 
// values they should be written to the board's EEPROM memory with the program
// Store_calibration.ino, so that each board can have its own calibration data
// onboard without having to update/change this main program.
//                      Ch0         Ch1       Ch2         Ch3       Ch4         Ch5       Ch6         Ch7                  
double tcOffset[] = { 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000, 0.0000000}; 
double tcSlope[] =  { 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000, 1.0000000};

//********************************
#define ERRLED A2		// Red error LED pin
#define GREENLED A3		// Green LED pin
#define BUTTON1 2 		// BUTTON1 on INT0, pin PD2
#define BUTTON2 3     // BUTTON2 on INT1, pin PD3

// Comment out the following line to remove parts of the
// test code from functioning. 
#define ECHO_TO_SERIAL // For testing serial output over FTDI adapter

// Interval to flash green LED during normal data collection
// For every 10 seconds, enter 10, for every 30 seconds, enter 30
#define PULSE_INTERVAL 10


// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_DATA, // collecting data normally
  STATE_CLOSE_FILE, // close data file, start new file
} mainState_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_TIME
} debounceState_t;

// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;

// debounce state machine variable, this takes on the various
// values defined for the DEBOUNCE_STATE typedef above.
volatile debounceState_t debounceState;

//******************************************
// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C // for oled1
#define I2C_ADDRESS2 0x3D // for oled2

SSD1306AsciiWire oled1; // create OLED display object, using I2C Wire
SSD1306AsciiWire oled2; // create OLED display object, using I2C Wire

//*************
// Create real time clock object
RTC_DS3231 rtc;
char buf[20]; // declare a string buffer to hold the time result

//*************
// Create sd card objects
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to
SdFile calibfile; // for sd card, this is the calibration file to write
const byte chipSelect = 10; // define the Chip Select pin for SD card

//************
// Define MAX31856 chip select pins, need 8 of them for the 8 separate chips
#define CS_MAX0 A1 // Arduino pin PC1, analog 1, Chip Select for MAX31856 #0
#define CS_MAX1 A0 // Arduino pin PC0, analog 0, Chip Select for MAX31856 #1
#define CS_MAX2 7 // Arduino pin PD7, digital 7, Chip Select for MAX31856 #2
#define CS_MAX3 6 // Arduino pin PD6, digital 6, Chip Select for MAX31856 #3
#define CS_MAX4 5 // Arduino pin PD5, digital 5, Chip Select for MAX31856 #4
#define CS_MAX5 4 // Arduino pin PD4, digital 4, Chip Select for MAX31856 #5
#define CS_MAX6 9 // Arduino pin PB1, digital 9, Chip Select for MAX31856 #6
#define CS_MAX7 8 // Arduino pin PB0, digital 8, Chip Select for MAX31856 #7
#define NUM_MAX31856  8 // 8 thermocouple channels

// Create the TempSensor object, defining the pins used for communication
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

// Declare data arrays
double tempArray[AVG_WINDOW][8]; // store temperature values 128bytes
double tempAverages[8]; // store average of each sensor's last few readings
double prevAverages[8]; // store previous round of sensor readings

// Declare initial name for output files written to SD card
char filename[] = "YYYYMMDD_HHMM_00.csv";

byte loopCount = 0; // counter to keep track of data sampling loops
byte buttonCounter = 0; // counter to count loops after button press
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
byte SPS = SAMPLES_PER_SECOND; 

DateTime buttonTime; // hold the time since the button was pressed
DateTime chooseTime; // hold the time stamp when a waiting period starts
DateTime checkTime; // hold time stamp when a button press debounce is updated
DateTime calibEnterTime; // hold the time stamp when calibration mode is entered
volatile unsigned long button1Time; // hold the initial button1 press millis() value
volatile unsigned long button2Time; // hold the initial button2 press millis() value
byte debounceTime = 20; // milliseconds to wait for debounce
volatile bool button1Flag = false; // Flag to mark when button1 was pressed
volatile bool button2Flag = false; // Flag to mark when button2 was pressed
byte mediumPressTime = 2; // seconds to hold button1 to register a medium press
byte longPressTime = 5; // seconds to hold button1 to register a long press
byte pressCount = 0; // counter for number of button presses
unsigned long prevMillis;	// counter for faster operations
unsigned long newMillis;	// counter for faster operations

float readout = NAN; // Reading back EEPROM calibration values
bool saveData = false; // Flag to tell whether to carry out write operation on SD card
bool oledScreenOn = true; // Flag to tell whether screens should be on/off
bool writeFlag = false; // Flag to signal time to write data to SD card


//---------------- setup loop ------------------------------------------------
void setup() {
	// Set button1 as an input
	pinMode(BUTTON1, INPUT_PULLUP);
  // Set button2 as an input
  pinMode(BUTTON2, INPUT_PULLUP);
	// Register an interrupt on INT0, attached to button1
	// which will call buttonFunc when the button is pressed.
	attachInterrupt(digitalPinToInterrupt(BUTTON1), button1Func, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON2), button2Func, LOW);
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
  
#ifdef ECHO_TO_SERIAL  
	Serial.println(F("Hello"));
	Serial.println();
#endif


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
  bool rtcErrorFlag = false;

	// Initialize the real time clock DS3231M
	Wire.begin();	// Start the I2C library with default options
	rtc.begin();	// Start the rtc object with default options
	newtime = rtc.now(); // read a time from the real time clock
  newtime.toString(buf, 20); 
  // Now extract the time by making another character pointer that
  // is advanced 10 places into buf to skip over the date. 
  char *timebuf = buf + 10;
  
	printTimeSerial(rtc.now()); // print time to serial monitor
  
  oled1.println();
  oled1.set2X();
  for (int i = 0; i<11; i++){
    oled1.print(buf[i]);
  }
  oled2.println();
  oled2.set2X();
  oled2.print(timebuf);
  
	Serial.println();
	if ( (newtime.year() < 2017) | (newtime.year() > 2035) ) {
		// There is an error with the clock, halt everything
    oled1.home();
    oled1.clear();
    oled1.set1X();
    oled1.println(F("RTC ERROR"));
    oled1.println(buf);
    oled1.println(F("Continue?"));
    oled1.println(F("Press 1"));

    rtcErrorFlag = true;
    bool stallFlag = true;
    // Allow user to plow
    // ahead without rtc (use button input?)
		while(stallFlag){ // loop due to RTC initialization error
      digitalWrite(ERRLED, HIGH);
      delay(50);
      digitalWrite(ERRLED, LOW);
      digitalWrite(GREENLED, HIGH);
      delay(50);
      digitalWrite(GREENLED, LOW);

      if (digitalRead(BUTTON1) == LOW){
        delay(40);  // debounce pause
        if (digitalRead(BUTTON1) == LOW){
          // If button is still low 40ms later, this is a real press
          // Now wait for button to go high again
          while(digitalRead(BUTTON1) == LOW) {;} // do nothing
          stallFlag = false; // break out of while(stallFlag) loop
        } 
      }              
    }	
	}

  //*************************************************************
  // SD card setup and read (assumes Serial output is functional already)
  bool sdErrorFlag = false;
	pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
	// Initialize the SD card object
	// Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
	// errors on a breadboard setup. 
	if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
	// If the above statement returns FALSE after trying to 
	// initialize the card, enter into this section and
	// hold in an infinite loop.
        // There is an error with the SD card, halt everything
    oled1.home();
    oled1.clear();
    oled1.println(F("SD ERROR"));
    oled1.println();
    oled1.println(F("Continue?"));
    oled1.println(F("Press 1"));

    sdErrorFlag = true;
    bool stallFlag = true;
		while(stallFlag){ // loop due to SD card initialization error
      digitalWrite(ERRLED, HIGH);
      delay(100);
      digitalWrite(ERRLED, LOW);
      digitalWrite(GREENLED, HIGH);
      delay(100);
      digitalWrite(GREENLED, LOW);

      if (digitalRead(BUTTON1) == LOW){
        delay(40);  // debounce pause
        if (digitalRead(BUTTON1) == LOW){
          // If button is still low 40ms later, this is a real press
          // Now wait for button to go high again
          while(digitalRead(BUTTON1) == LOW) {;} // do nothing
          stallFlag = false; // break out of while(stallFlag) loop
        } 
      }              
		}
	} // end of (!sd.begin(chipSelect, SPI_FULL_SPEED))

  // If the clock and sd card are both working, we can save data
  if (!sdErrorFlag && !rtcErrorFlag){ 
    saveData = true;
  } else {
    saveData = false;
  }

  if (saveData){
    // If both error flags were false, continue with file generation
    newtime = rtc.now(); // grab the current time
    initFileName(sd, logfile, newtime, filename); // generate a file name
#ifdef ECHO_TO_SERIAL    
    Serial.print(F("Writing to "));
    Serial.println(filename);
    delay(5);
#endif
    oled1.home();
    oled1.set1X();
    oled1.clearToEOL();
    oled1.print(F("Writing to: "));
    oled2.home();
    oled2.set1X();
    oled2.clearToEOL();
    oled2.print(filename);
    delay(1000);          
  } else {
     // If saveData if false
    oled1.home();
    oled1.clear();
    oled1.set2X();
    oled1.println(F("Data will"));
    oled1.println(F("not be"));
    oled1.println(F("saved!"));
    delay(1000);
    oled1.println(F("Starting.."));
    delay(1000);
  }

  //*******************************************************
  // Read calibration coefficients from EEPROM, if present
 for (int memEntry = 0; memEntry < 8; memEntry++){
    // Read eeprom at memEntry address and store in readout
    EEPROM_ReadFloat(&readout, memEntry);   
    // Handle cases where the value coming back is 0xFFFFFFFF
    // which is what happens if EEPROM was never written to.
    if (isnan(readout)){
      // If true, there was no value stored, so keep the 
      // default value in tcOffset
    } else if (!isnan(readout)){
      // Now check if the numeric value that came back is
      // within a reasonable range of values
      if ( abs(readout - 0.00) < 3){
        // If the readout value is within 3degrees C of 0,
        // the readout value is probably usable
        // Stick the value in the tcOffset array
        tcOffset[memEntry] = readout;      
      } else {
        // If the readout value is way off the expected range
        // then assume there's an error in the stored value
        // and proceed with default offset of 0.0
        tcOffset[memEntry] = 0.00000;
      }
    }
 }
 
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
      // Sanity check the slope value
      if ( abs(readout - 1.00) < 0.5) {
        // If the readout value for slope is with 0.5 of
        // the ideal slope of 1.0, use the value from 
        // readout. A difference from 1.0 this large is
        // something to be extremely suspect of though,
        // and you should double-check your calibration
        // methods.
        // Stick the value in the tcSlope array
        tcSlope[memEntry-8] = readout;
      } else {
        // If the slope value returned in readout is very
        // farm from the expected value of 1.0, assume that
        // the value in EEPROM is invalid, and proceed with
        // the default slope of 1.0. 
        tcSlope[memEntry-8] = 1.000;
      }
    }
 }
 Serial.println(F("Using coefficients:"));
 for (int i = 0; i < 8; i++){
  Serial.print(tcOffset[i],4);
  Serial.print(F("\t"));
  Serial.println(tcSlope[i],6);
 }
  
  // Initialize the temperature sensor chips
  for (int i=0; i<NUM_MAX31856;i++){
    TempSensor[i]->begin(); // Initialize SPI and MAX31856 registers
    // Set thermocouple type to K (default)
    TempSensor[i]->setThermocoupleType(MAX31856_TCTYPE_K);
  }
  // Read back the thermocouple type to make sure it was set successfully
  Serial.print("Thermocouple type: "); 
  for (int i=0; i<NUM_MAX31856;i++){
    switch ( TempSensor[i]->getThermocoupleType() ) {  // Read TC type
      case MAX31856_TCTYPE_B: Serial.println(F("B")); break;
      case MAX31856_TCTYPE_E: Serial.println(F("E")); break;
      case MAX31856_TCTYPE_J: Serial.println(F("J")); break;
      case MAX31856_TCTYPE_K: Serial.println(F("K")); break;
      case MAX31856_TCTYPE_N: Serial.println(F("N")); break;
      case MAX31856_TCTYPE_R: Serial.println(F("R")); break;
      case MAX31856_TCTYPE_S: Serial.println(F("S")); break;
      case MAX31856_TCTYPE_T: Serial.println(F("T")); break;
      case MAX31856_VMODE_G8: Serial.println(F("Voltage x8 Gain mode")); break;
      case MAX31856_VMODE_G32: Serial.println(F("Voltage x8 Gain mode")); break;
      default: Serial.println(F("Unknown")); break;
    }
  }
  // Take 4 temperature readings to initialize tempArray
  for (byte i = 0; i < AVG_WINDOW; i++){
      for (byte Channel = 0; Channel < NUM_MAX31856; Channel++){
        tempArray[i][Channel] = ((TempSensor[Channel]->readThermocoupleTemperature()) * tcSlope[Channel]) + tcOffset[Channel];
        delay(15); // Delay a bit so that by the time you get back to 
                   // reading the same thermocouple again, at least 100ms
                   // has elapsed
      }
  }

  // Now calculate the average of the 4 readings for each sensor Channel
  for (byte Channel = 0; Channel<8; Channel++){
    double tempsum = 0;
    for (byte j = 0; j < AVG_WINDOW; j++){
      // Add up j measurements for sensor Channel
      tempsum += tempArray[j][Channel];
    }
    // Calculate average temperature for sensor Channel
    tempAverages[Channel] = tempsum / double(AVG_WINDOW); // cast denominator as double
  }
  // Make a copy of the 1st set of averages for use later in main loop
  for (byte Channel = 0; Channel < 8; Channel++){
    prevAverages[Channel] = tempAverages[Channel];
  }
  // Show the first set of temperature readings:
  oled1.home();
  oled1.set2X();
  oled1.clearToEOL();
  for (byte Channel = 0; Channel < 4; Channel++){
    oled1.clearToEOL();
    oled1.print(F("Ch"));
    oled1.print(Channel);
    oled1.print(F(": "));
    oled1.print(tempAverages[Channel]);
    oled1.println(F("C"));
  }
  oled2.home();
  oled2.set2X();
  for (byte Channel = 4; Channel < 8; Channel++){
    oled2.clearToEOL();
    oled2.print(F("Ch"));
    oled2.print(Channel);
    oled2.print(F(": "));
    oled2.print(tempAverages[Channel]);
    oled2.println(F("C"));
  } 

	// Start 32.768kHz clock signal on TIMER2. 
	// Supply the current time value as the argument, returns 
	// an updated time
	newtime = startTIMER2(rtc.now(), rtc, SPS);

  // Cycle briefly until we reach 9 sec, so that the
  // data collection loop will start on a nice even 0 sec
  // time stamp. 
  while (!( (newtime.second() % 10) == 0)){
    delay(50);
    newtime = rtc.now();
  }
	
	oldtime = newtime; // store the current time value
	
	mainState = STATE_DATA; // Start the main loop in data-taking state
}

//*************************Main loop************************************
//*************************Main loop************************************
void loop() {
	// Always start the loop by checking the time
	newtime = rtc.now(); // Grab the current time

	//-------------------------------------------------------------
	// Begin loop by checking the debounceState to 
	// handle any button presses
 	switch (debounceState) {
		// debounceState should normally start off as 
		// DEBOUNCE_STATE_IDLE until button1 is pressed,
		// which causes the state to be set to 
		// DEBOUNCE_STATE_CHECK
		//************************************
		case DEBOUNCE_STATE_IDLE:
			// Do nothing in this case
		break;
		//************************************
		case DEBOUNCE_STATE_CHECK:
			// If the debounce state has been set to 
			// DEBOUNCE_STATE_CHECK by the buttonFunc interrupt,
			// check if the button is still pressed (LOW)
			if (digitalRead(BUTTON1) == LOW) {
				if ( (millis() - button1Time) >  debounceTime) {
					// If the button has been held long enough to 
					// be a legit button press, switch to 
					// DEBOUNCE_STATE_TIME to keep track of how long 
					// the button is held
					debounceState = DEBOUNCE_STATE_TIME;
					buttonTime = rtc.now();
				} else {
					// If button is still pressed, but the debounce 
					// time hasn't elapsed, remain in this state
					debounceState = DEBOUNCE_STATE_CHECK;
				}
			} else {
				// If BUTTON1 is high again when we hit this
				// case in DEBOUNCE_STATE_CHECK, it was a false trigger
				// Reset the debounceState
				debounceState = DEBOUNCE_STATE_IDLE;
				button1Flag = false;
				// Restart the button1 interrupt
				attachInterrupt(digitalPinToInterrupt(BUTTON1), button1Func, LOW);
			}
		break; // end of case DEBOUNCE_STATE_CHECK
		//*************************************
		case DEBOUNCE_STATE_TIME:
			if (digitalRead(BUTTON1) == HIGH) {
				// If the user released the button, now check how
				// long the button was depressed. This will determine
				// which state the user wants to enter. 

				checkTime = rtc.now(); // get the time
				
				if (checkTime.unixtime() < (buttonTime.unixtime() + mediumPressTime)) {
					// User held BUTTON1 briefly, treat as a normal
					// button press, which will be handled differently
					// depending on which mainState the program is in.
					button1Flag = true;
					
				} else if (checkTime.unixtime() > (buttonTime.unixtime() + longPressTime)){
					// User held BUTTON1 long enough to enter close file mode
					mainState = STATE_CLOSE_FILE;
				}
				
				// Now that the button press has been handled, return
				// to DEBOUNCE_STATE_IDLE and await the next button press
				debounceState = DEBOUNCE_STATE_IDLE;
				// Restart the button1 interrupt now that the button
				// has been released
				attachInterrupt(digitalPinToInterrupt(BUTTON1), button1Func, LOW);
			} else {
				// If button is still low (depressed), remain in 
				// this DEBOUNCE_STATE_TIME
				debounceState = DEBOUNCE_STATE_TIME;
			}
			
		break; // end of case DEBOUNCE_STATE_TIME	
	} // end switch(debounceState)
	
	
	//----------------------------------------------------------
	switch (mainState) {
		//*****************************************************
		case STATE_DATA:
			// Check to see if the current seconds value
			// is equal to oldtime.second(). If so, we
			// are still in the same second. 
     if ( abs(newtime.second() - oldtime.second()) >= SAVE_INTERVAL) {
				oldtime = newtime; // update oldtime
        writeFlag = true; // set flag to write data to SD				
        // This will force a SD card write at the SAVE_INTERVAL (seconds)
		 }

      if (loopCount >= AVG_WINDOW){
        loopCount = 0; // reset to begin writing at start of array
      }

      for (byte Channel = 0; Channel < NUM_MAX31856; Channel++){
        tempArray[loopCount][Channel] = ((TempSensor[Channel]->readThermocoupleTemperature()) * tcSlope[Channel]) + tcOffset[Channel];
      }

      // Calculate the average of the readings for each sensor j
      for (byte Channel = 0; Channel < NUM_MAX31856; Channel++){
        double tempsum = 0;
        for (byte j = 0; j < AVG_WINDOW; j++){
          // Add up j measurements for sensor 
          tempsum = tempsum + tempArray[j][Channel];
        }
        // Calculate average temperature for sensor i
        tempAverages[Channel] = tempsum / double(AVG_WINDOW); // cast denominator as double
      }

      if (writeFlag){
        writeFlag = false; // reset to false
        if (saveData){
          // Call the writeToSD function to output the data array contents
          // to the SD card 
          writeToSD(newtime);
        }
 #ifdef ECHO_TO_SERIAL
        // If ECHO_TO_SERIAL is defined at the start of the 
        // program, then this section will send updates
        printTimeSerial(newtime);
        Serial.print(F("\t"));
        for (byte i = 0; i < 8; i++){
          Serial.print(tempAverages[i]);
          Serial.print(F("\t"));
        }
        Serial.println();
        delay(10);
#endif    
      } // end of if(writeFlag)

      if (button1Flag){
        buttonCounter++;
        // If button1Flag is true, the user requested an action. Update
        // the OLED screens to show the current file name and time
        // The if statements below will either display the filename and
        // current time, or reset the screens to show temperature data
        // depending on how long it's been. 
        if ( (newtime.unixtime() - checkTime.unixtime()) < 5){
          newtime.toString(buf, 20); 
          // Now extract the time by making another character pointer that
          // is advanced 10 places into buf to skip over the date. 
          char *timebuf = buf + 10;
          if (buttonCounter <= 1){
            // Only update screen 1 on the first cycle
            oled1.home();
            oled1.clear();          
            oled1.set2X();
            for (int i = 0; i<11; i++){
              oled1.print(buf[i]);
            }
            oled1.println();
            oled1.set1X();
            // Show the filename (or indicate data aren't being saved)
            if (saveData){
              oled1.println(F("Filename:"));
              oled1.print(filename);              
            } else {
              oled1.println(F("No saved data"));
            }
          } // end of if (buttonCounter <= 1)

          // Switch to screen 2, update each cycle to show time ticking
          oled2.home();
          oled2.set2X();
          oled2.clear();
          oled2.print(timebuf);
        } else if ( (newtime.unixtime() - checkTime.unixtime()) > 5){
          // It has been more than 5 seconds, reset button1Flag and buttonCounter
          // and switch the OLED screens back to temperature ouput
          button1Flag = false; // reset flag
          buttonCounter = 0; // reset counter
          // Re-initialize OLEDs to show temperature info.
          oled1.home();
          oled1.set2X();
          oled1.clearToEOL();
          for (byte Channel = 0; Channel < 4; Channel++){
            oled1.clearToEOL();
            oled1.print(F("Ch"));
            oled1.print(Channel);
            oled1.print(F(": "));
            oled1.print(tempAverages[Channel]);
            oled1.println(F("C"));
          }
          oled2.home();
          oled2.set2X();
          for (byte Channel = 4; Channel < 8; Channel++){
            oled2.clearToEOL();
            oled2.print(F("Ch"));
            oled2.print(Channel);
            oled2.print(F(": "));
            oled2.print(tempAverages[Channel]);
            oled2.println(F("C"));
          } 
        } // end of if (rtc.now() - checkTime > 5){ 
      } else {
        // Update the OLED screens with the current temperature averages
        // once per second. 
  			if (loopCount == (SAMPLES_PER_SECOND - 1)) {
          if (oledScreenOn){
            // Print stuff to screens. Function in TClib2.h
            printTempToOLEDs(oled1,oled2,tempAverages,prevAverages);
          } // end of if (oledScreenOn)
          // Update the old prevAverages with these new tempAverages
          for (byte Channel = 0; Channel < 8; Channel++){
            prevAverages[Channel] = tempAverages[Channel];
          }
        } // end of if (loopCount >= (SAMPLES_PER_SECOND - 1))   
      }  // end of if (button1Flag)              
                        
                        
			// Increment loopCount after writing all the sample data to
			// the arrays
			++loopCount; 
				

			goToSleep(); // function in TClib2.h	

			// After waking, this case should end and the main loop
			// should start again. 
			mainState = STATE_DATA;
		break; // end of case STATE_DATA

		//*****************************************************
 		case STATE_CLOSE_FILE:
			// If user held button 1 down for at least 10 seconds, they want 
			// to close the current data file and open a new one. 
			logfile.close(); // Make sure the data file is closed and saved.
      oled1.home();
      oled1.clear();
      oled1.set2X();
      oled1.println(F("File"));
      oled1.println(F("Closed"));
      oled1.set1X();
      oled1.println(filename);
			// Briefly flash the green led to show that program 
			// has closed the data file and started a new one. 
			for (byte i = 0; i < 15; i++){
				digitalWrite(GREENLED, HIGH);
				delay(100);
				digitalWrite(GREENLED, LOW);
				delay(100);
			}
      // Open a new output file
      initFileName(sd, logfile, rtc.now(), filename ); 
#ifdef ECHO_TO_SERIAL
			Serial.print(F("New file: "));
      Serial.println(filename);
			printTimeSerial(newtime);
			Serial.println();
#endif		
			mainState = STATE_DATA; // Return to normal data collection
      button1Flag = true; // Set true so that OLED screens display new filename briefly
      checkTime = rtc.now();
  break; // end of case STATE_FILE_CLOSE
		
	} // End of switch (mainState) statement
} // end of main loop


//-----------------------------------------------------------------------------
// This Interrupt Service Routine (ISR) is called every time the
// TIMER2_OVF_vect goes high (==1), which happens when TIMER2
// overflows. The ISR doesn't care if the AVR is awake or
// in SLEEP_MODE_PWR_SAVE, it will still roll over and run this
// routine. If the AVR is in SLEEP_MODE_PWR_SAVE, the TIMER2
// interrupt will also reawaken it. This is used for the goToSleep() function
ISR(TIMER2_OVF_vect) {
	// nothing needs to happen here, this interrupt firing should 
	// just awaken the AVR
}

//--------------- button1Func --------------------------------------------------
// button1Func
void button1Func (void){
	detachInterrupt(digitalPinToInterrupt(BUTTON1)); // Turn off the interrupt
	button1Time = millis(); // Grab the current elapsed time
	debounceState = DEBOUNCE_STATE_CHECK; // Switch to new debounce state
	// Execution will now return to wherever it was interrupted, and this
	// interrupt will still be disabled. 
}

void button2Func (void){
  detachInterrupt(digitalPinToInterrupt(BUTTON2)); // Turn off the interrupt
  button2Time = millis(); // Grab the current elapsed time
  debounceState = DEBOUNCE_STATE_CHECK; // Switch to new debounce state
  // Execution will now return to wherever it was interrupted, and this
  // interrupt will still be disabled. 
}




//------------- writeToSD -----------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (DateTime timestamp) {

	// Flash the green LED every 30 seconds to show data is being collected
	if (newtime.second() % PULSE_INTERVAL == 0) {
		digitalWrite(GREENLED,HIGH);
		delay(5);
		digitalWrite(GREENLED,LOW);
	}
	// Reopen logfile. If opening fails, notify the user
	if (!logfile.isOpen()) {
		if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
			digitalWrite(ERRLED, HIGH); // turn on error LED
		}
	}
	// Write the unixtime
	logfile.print(timestamp.unixtime(), DEC); // POSIX time value
	logfile.print(F(","));
  printTimeToSD(logfile, timestamp); // human-readable 
  // Write the 8 temperature values in a loop
  for (byte i = 0; i < 8; i++){
    logfile.print(F(","));
    if (isnan(tempAverages[i])){
      // If the value is nan, replace with NA for easier parsing
      // in R. 
      logfile.print(F("NA"));
    } else {
      logfile.print(tempAverages[i], 3); // truncate to 3 significant digits
    }
  }
  logfile.println();
	// logfile.close(); // force the buffer to empty

  if (timestamp.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, timestamp.year(),timestamp.month(), \
	    timestamp.day(),timestamp.hour(),timestamp.minute(),timestamp.second());
  }
}

//---------freeRam-----------
// A function to estimate the remaining free dynamic memory. On a 328P (Uno), 
// the dynamic memory is 2048 bytes.
int freeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
