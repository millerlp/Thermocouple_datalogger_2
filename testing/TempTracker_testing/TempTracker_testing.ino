/* TempTracker_testing.ino
	Copyright Luke Miller 2017

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
	
  Code to run a Thermocouple_datalogger_RevA board with
  MAX31855 thermocouple sensors. By default
  this program will start running and saving data to the SD 
  card automatically. 
  	
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

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>	// built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>	// built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "TClib2.h" // My utility library for this project
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

// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C
#define I2C_ADDRESS2 0x3D
SSD1306AsciiWire oled1; // create OLED display object
SSD1306AsciiWire oled2; // create OLED display object


// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_DATA, // collecting data normally
  STATE_ENTER_CALIB, // user wants to calibrate
//  STATE_CALIB1, // user chooses to calibrate accel 1
//  STATE_CALIB2, // user chooses to calibrate accel 2
  STATE_CALIB_WAIT, // waiting for user to signal mussel is positioned
  STATE_CALIB_ACTIVE, // taking calibration data, button press ends this
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
double temp0 = 0; // hold output from MAX31855 #0
double temp1 = 0; // hold output from MAX31855 #1
double temp2 = 0; // hold output from MAX31855 #2
double temp3 = 0; // hold output from MAX31855 #3
double temp4 = 0; // hold output from MAX31855 #4
double temp5 = 0; // hold output from MAX31855 #5
double temp6 = 0; // hold output from MAX31855 #6
double temp7 = 0; // hold output from MAX31855 #7


// Declare data arrays
double tempArray[SAMPLES_PER_SECOND][8]; // store temperature values 128bytes
double tempAverages[8]; // store average of each sensor's last few readings

// Declare initial name for output files written to SD card
char filename[] = "YYYYMMDD_HHMM_00.csv";

volatile byte loopCount = 0; // counter to keep track of data sampling loops
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
byte SPS = SAMPLES_PER_SECOND; 
DateTime buttonTime; // hold the time since the button was pressed
DateTime chooseTime; // hold the time stamp when a waiting period starts
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

bool saveData = false; // Flag to tell whether to carry out write operation on SD card
bool oledScreenOn = true; // Flag to tell whether screens should be on/off
bool writeFlag = false; // Flag to signal time to write data to SD card

// Define two temperature limits for the thermocouples, values outside
// this range will trigger the error notification
//float TClowerlimit = 0.0;
//float TCupperlimit = 60.0;

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
#ifdef ECHO_TO_SERIAL  
	Serial.println(F("Hello"));
//	Serial.print(F("MCUSR contents: "));
//	printBits(mcusr);
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
  oled2.print(F("There"));
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
    oled1.println(F("RTC ERROR"));
    oled1.println(F("Continue?"));

    rtcErrorFlag = true;
    // Consider removing this while loop and allowing user to plow
    // ahead without rtc (use button input?)
		while(1){
		// Flash the error led to notify the user
		// This permanently halts execution, no data will be collected
			digitalWrite(ERRLED, !digitalRead(ERRLED));
			delay(100);
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
    oled1.println(F("SD ERROR"));
    oled1.println();
    oled1.println(F("SD ERROR"));
    oled1.println(F("Continue?"));

    sdErrorFlag = true;
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

  // If the clock and sd card are both working, we can save data
  if (!sdErrorFlag | !rtcErrorFlag){ saveData = true;}

  if (saveData){
    // If both error flags were false, continue with file generation
    newtime = rtc.now(); // grab the current time
//    initFileName(newtime); // generate a file name
    initFileName(sd, logfile, newtime, filename); // generate a file name
#ifdef ECHO_TO_SERIAL    
    Serial.print("Writing to ");
    Serial.println(filename);
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
          
  }

//   Take 4 temperature measurements to initialize the array
  for (byte i = 0; i < 4; i++){
    tempArray[i][0] = thermocouple0.readCelsius(); delay(15);
    tempArray[i][1] = thermocouple1.readCelsius(); delay(15);
    tempArray[i][2] = thermocouple2.readCelsius(); delay(15);
    tempArray[i][3] = thermocouple3.readCelsius(); delay(15);
    tempArray[i][4] = thermocouple4.readCelsius(); delay(15);
    tempArray[i][5] = thermocouple5.readCelsius(); delay(15);
    tempArray[i][6] = thermocouple6.readCelsius(); delay(15);
    tempArray[i][7] = thermocouple7.readCelsius(); delay(15);
  }

  // Now calculate the average of the 4 readings for each sensor i
  for (byte i = 0; i<8; i++){
    double tempsum = 0;
    for (byte j = 0; j < 4; j++){
      // Add up j measurements for sensor i
      tempsum += tempArray[j][i];
    }
    // Calculate average temperature for sensor i
    tempAverages[i] = tempsum / double(4); // cast denominator as double
  }

  oled1.home();
  oled1.set2X();
  oled1.clearToEOL();
  for (byte i = 0; i < 4; i++){
    oled1.clearToEOL();
    oled1.print(F("Ch"));
    oled1.print(i);
    oled1.print(F(": "));
    oled1.print(tempAverages[i]);
    oled1.println(F("C"));
  }
  oled2.home();
  oled2.set2X();
  for (byte i = 4; i < 8; i++){
    oled2.clearToEOL();
    oled2.print(F("Ch"));
    oled2.print(i);
    oled2.print(F(": "));
    oled2.print(tempAverages[i]);
    oled2.println(F("C"));
  } 
	delay(2000);
	// Start 32.768kHz clock signal on TIMER2. 
	// Supply the current time value as the argument, returns 
	// an updated time
	newtime = startTIMER2(rtc.now(), rtc, SPS);
	
	oldtime = newtime; // store the current time value
	
	mainState = STATE_DATA; // Start the main loop in data-taking state
}

//*************************Main loop************************************
void loop() {
//	delay(2); // trying this to quell bogus sd writes
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
			// check if the button is still pressed
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

				DateTime checkTime = rtc.now(); // get the time
				
				if (checkTime.unixtime() < (buttonTime.unixtime() + mediumPressTime)) {
					Serial.println(F("Short press registered"));
					// User held BUTTON1 briefly, treat as a normal
					// button press, which will be handled differently
					// depending on which mainState the program is in.
					button1Flag = true;
					
				} else if ( (checkTime.unixtime() > (buttonTime.unixtime() + mediumPressTime)) &
					(checkTime.unixtime() < (buttonTime.unixtime() + longPressTime)) ) {
					// User held BUTTON1 long enough to enter calibration
					// mode, but not long enough to enter close file mode
					mainState = STATE_ENTER_CALIB;
					
					// Flash both LEDs 5 times to let user know we've entered
					// calibration mode
					for (byte i = 0; i < 5; i++){
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(100);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(100);
					}
					
					// Start a timer for entering Calib mode, to be used to give
					// the user time to enter 1 button press or 2 to choose 
					// which unit to calibrate
					chooseTime = rtc.now();
	
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
			// bitSet(PIND, 4); // toggle on, for monitoring on o-scope
		
			// Check to see if the current seconds value
			// is equal to oldtime.second(). If so, we
			// are still in the same second. 
//     if ( abs(newtime.second() - oldtime.second()) >= 1) {
			if (oldtime.second() != newtime.second()) {
				oldtime = newtime; // update oldtime
        writeFlag = true; // set flag to write data to SD				
        // This will force a SD card write once per second
			}

      if (loopCount >= 4){
        loopCount = 0; // reset to beging writing at start of array
      }
      
			// If it is the start of a new minute, flash the 
			// green led each time through the loop. This is
			// used to help the user look for error codes that
			// flash at seconds 1,2,3,4,5, and 6. 
			if (newtime.second() == 0) {
				digitalWrite(GREENLED, HIGH);
				delay(5);
				digitalWrite(GREENLED, LOW);
			}

//      Serial.print(F("Reading "));
      Serial.print(loopCount);
      Serial.print(F(" "));
      printTimeSerial(newtime);
      Serial.println();
      // Read each of the sensors
      tempArray[loopCount][0] = thermocouple0.readCelsius();
      tempArray[loopCount][1] = thermocouple1.readCelsius();
      tempArray[loopCount][2] = thermocouple2.readCelsius();
      tempArray[loopCount][3] = thermocouple3.readCelsius();
      tempArray[loopCount][4] = thermocouple4.readCelsius();
      tempArray[loopCount][5] = thermocouple5.readCelsius();
      tempArray[loopCount][6] = thermocouple6.readCelsius();
      tempArray[loopCount][7] = thermocouple7.readCelsius();


      if (writeFlag){
        // Calculate the average of the 4 readings for each sensor i
        for (byte i = 0; i < 8; i++){
          double tempsum = 0;
          for (byte j = 0; j < 4; j++){
            // Add up j measurements for sensor i
            tempsum = tempsum + tempArray[j][i];
          }
          // Calculate average temperature for sensor i
          tempAverages[i] = tempsum / double(4); // cast denominator as double
        }

        // Call the writeToSD function to output the data array contents
        // to the SD card 
        writeToSD(newtime);
        writeFlag = false; // reset to false
#ifdef ECHO_TO_SERIAL
        // If ECHO_TO_SERIAL is defined at the start of the 
        // program, then this section will send updates of the
        // sensor values once per second.
        printTimeSerial(oldtime);
        Serial.print(F(","));
        for (byte i = 0; i < 8; i++){
          Serial.print(tempAverages[i]);
          Serial.print(F(", "));
        }
        Serial.println();

//        delay(10);
//        Serial.println();A
        delay(10);

#endif          
      } // end of if(writeFlag)
      
//			if (loopCount == (SAMPLES_PER_SECOND - 1)) {
//        if (oledScreenOn){
//          // Print stuff to screens
//          oled1.home();
//          oled1.set2X();
//          oled1.clearToEOL();
//          for (byte i = 0; i < 4; i++){
//            oled1.clearToEOL();
//            oled1.print(F("Ch"));
//            oled1.print(i);
//            oled1.print(F(": "));
//            oled1.print(tempAverages[i]);
//            oled1.println(F("C"));
//          }
//          oled2.home();
//          oled2.set2X();
//          for (byte i = 4; i < 8; i++){
//            oled2.clearToEOL();
//            oled2.print(F("Ch"));
//            oled2.print(i);
//            oled2.print(F(": "));
//            oled2.print(tempAverages[i]);
//            oled2.println(F("C"));
//          } 
//        } // end of if (oledScreenOn)
//      } // end of if (loopCount >= (SAMPLES_PER_SECOND - 1))                   
                        
                        
			// Increment loopCount after writing all the sample data to
			// the arrays
			++loopCount; 
				
			// bitSet(PIND, 4); // toggle off, for monitoring on o-scope
			// delay(1);
			// bitSet(PIND, 3); // toggle on, for monitoring on o-scope
			goToSleep(); // function in MusselTrackerlib.h	
			// bitSet(PIND, 3); // toggle off, for monitoring on o-scope
			// After waking, this case should end and the main loop
			// should start again. 
			mainState = STATE_DATA;
		break; // end of case STATE_DATA
		
		//*****************************************************
		case STATE_ENTER_CALIB:
			// We have arrived at this state because the user held button1 down
			// for the specified amount of time (see the debounce cases earlier)
			// so we now allow the user to enter additional button presses to 
			// choose which accelerometer they want to calibrate. 
			
			// Read a time value
			calibEnterTime = rtc.now();
	
			if (calibEnterTime.unixtime() < (chooseTime.unixtime() + longPressTime)){
				// If longPressTime has not elapsed, add any new button presses to the
				// the pressCount value
				if (button1Flag) {
					pressCount++;
					// If the user pressed the button, reset chooseTime to
					// give them extra time to press the button again. 
					chooseTime = calibEnterTime;
					button1Flag = false; // reset buttonFlag
					// If more than 2 button presses are registered, cycle
					// back around to zero.
					if (pressCount > 2) {
						pressCount = 0;
					}
					Serial.print(F("Button1 press "));
					Serial.println(pressCount);

					// Flash the green led 1 or 2 times to show current count
					if (pressCount > 0) {
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(200);
							digitalWrite(GREENLED, LOW);
							delay(200);
						}
					} else if (pressCount == 0) {
						// Flash red LED to show that we haven't 
						// got a useful choice
						digitalWrite(ERRLED, HIGH);
						delay(250);
						digitalWrite(ERRLED, LOW);
						delay(250);
					}
				}
				mainState = STATE_ENTER_CALIB; // remain in this state
			} else if (calibEnterTime.unixtime() >= (chooseTime.unixtime() + longPressTime)){
				// The wait time for button presses has elapsed, now deal with 
				// the user's choice based on the value in pressCount
				switch (pressCount) {
					case 0:
						// If the user didn't press the button again, return
						// to normal data taking mode
						mainState = STATE_DATA; 
						Serial.println(F("Returning to data state"));
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);	
					break;
					case 1:
						// If the user pressed one time, we'll calibrate accel 1
						mainState = STATE_CALIB_WAIT;
						pressCount = 1;
						Serial.println(F("Calib accel 1"));
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(1300);
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(250);
							digitalWrite(GREENLED, LOW);
							delay(250);
						}
						delay(1000);
					break;
					case 2:
						// If the user pressed two times, we'll calibrate accel 2
						mainState = STATE_CALIB_WAIT;
						pressCount = 2;
						Serial.println(F("Calib accel 2"));
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(1300);
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(200);
							digitalWrite(GREENLED, LOW);
							delay(200);
						}		
						delay(1000);
					break;
				} 
			}
		break; // end of STATE_CALIB_ENTER
		//*****************************************************
 		case STATE_CALIB_WAIT:
			// If the user entered either 1 or 2 button presses in 
			// STATE_CALIB_ENTER, then we should arrive here. 
			// Now we wait for the user to get the mussel situated and 
			// press the button one more time to begin taking data.
			// The green led will pulse on and off at 1Hz while waiting
			if (newtime.second() != calibEnterTime.second() ) {
				Serial.println(F("Waiting..."));
				calibEnterTime = newtime; // reset calibEnterTime
				digitalWrite(GREENLED, !digitalRead(GREENLED));
			}
			
			// If the user finally presses the button, enter the active
			// calibration state
			if (button1Flag) {
				mainState = STATE_CALIB_ACTIVE;
				button1Flag = false; // reset button1Flag
				// Create a data output file
//				initCalibFile(sd, logfile, newtime, filename);
//				Serial.print(F("Writing to "));
//				Serial.println(filenameCalib);
				if (pressCount == 1){
					delay(5);
				} else if (pressCount == 2) {
					delay(5);
				}
			}
			
		break;
		//*****************************************************
		case STATE_CALIB_ACTIVE:
//			// Write accelerometer/compass data to the calibration file
//			newMillis = millis(); // get current millis value
//			// If 10 or more milliseconds have elapsed, take a new
//			// reading from the accel/compass
//			if (newMillis >= prevMillis + 10) {
//				prevMillis = newMillis; // update millis
//				// Reopen logfile. If opening fails, notify the user
//				if (!calibfile.isOpen()) {
//					if (!calibfile.open(filenameCalib, O_RDWR | O_CREAT | O_AT_END)) {
//						digitalWrite(ERRLED, HIGH); // turn on error LED
//					}
//				}
//				// Choose which accel to sample based on pressCount value
//				switch (pressCount) {
//					case 1:
//						digitalWrite(GREENLED, HIGH);
//
//						digitalWrite(GREENLED, LOW);
//					break;
//					
//					case 2:
//						digitalWrite(GREENLED, HIGH);
//						
//						digitalWrite(GREENLED, LOW);
//					break;
//				} // end of switch (pressCount)
//			}
//			
//			// The user can press button1 again to end calibration mode
//			// This would set buttonFlag true, and cause the if statement
//			// below to execute
// 			if (button1Flag) {
//				button1Flag = false;
//				calibfile.close(); // close and save the calib file
//				Serial.println(F("Saving calib file"));
//				// Set the accel/magnetometer back to normal slow
//				// mode (50Hz accel with antialias filter, 6.25Hz magnetometer)
//				if (pressCount == 1){
//
//					delay(5);
//
//				} else if (pressCount == 2) {
//
//					delay(5);
//
//				}
////				initFileName(newtime); // open a new data file
//        initFileName(sd, logfile, newtime, filename); // open a new data file
//				mainState = STATE_DATA; // return to STATE_DATA
//				// Flash both LEDs 5 times to let user know we've exited
//				// calibration mode
//				for (byte i = 0; i < 5; i++){
//					digitalWrite(ERRLED, HIGH);
//					digitalWrite(GREENLED, HIGH);
//					delay(100);
//					digitalWrite(ERRLED, LOW);
//					digitalWrite(GREENLED, LOW);
//					delay(100);
//				}
//			} // end of if(button1Flag) statement
			
		break; 
		
		//*****************************************************
 		case STATE_CLOSE_FILE:
			// If user held button 1 down for at least 6 seconds, they want 
			// to close the current data file and open a new one. 
			logfile.close(); // Make sure the data file is closed and saved.
			
			// Briefly flash the green led to show that program 
			// has closed the data file and started a new one. 
			for (byte i = 0; i < 15; i++){
				digitalWrite(GREENLED, HIGH);
				delay(100);
				digitalWrite(GREENLED, LOW);
				delay(100);
			}

      initFileName(sd, logfile, rtc.now(), filename ); // Open a new output file
#ifdef ECHO_TO_SERIAL
			Serial.print(F("Writing to "));
			printTimeSerial(newtime);
			Serial.println();
#endif		
			mainState = STATE_DATA; // Return to normal data collection
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
    logfile.print(tempAverages[i]);
  }
  logfile.println();
	// logfile.close(); // force the buffer to empty

  if (timestamp.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, timestamp.year(),timestamp.month(), \
	    timestamp.day(),timestamp.hour(),timestamp.minute(),timestamp.second());
  }
}



