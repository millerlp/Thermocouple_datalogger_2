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
  
	Entering calibration mode:
		1. Hold button 1 down for at least 3 seconds, but less than 5 seconds
		2. Both red and green leds will flash 5 times when you enable 
			calibration mode. 
		3. You then have ~3 seconds to press button1 again
			to choose whether to calibrate mussel 1 or 2. 
		4. Pressing button1 repeatedly will cycle through 1 flash 
			(mussel 1), 2 flashes (mussel 2) or a red flash (no mussel chosen). 
		5. Once you've chosen the mussel, wait a few seconds for both red and
			green LEDs to flash once. 
		6. The green led will then slowly cycle on and off (1Hz) while it waits 
			for you to get the mussel situated.
		7. With the mussel situated facing north, press button1 briefly to 
			enter the calibration data collection mode. 
		8. When you have finished taking calibration data, press button1 again
			to exit calibration mode and return to normal data collection mode. 
			The red and green LEDs will flash 5 times to denote the end of 
			calibration mode. 
		
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

	
	Sensor error codes:
		At the top of every minute, the green LED will flash quickly 5
		times. This lets the user know that error codes will follow 
		during the following 6 seconds, one signal per second. 
		As each second ticks by (1-6), the red error LED will light once 
		if a sensor error code is set for that sensor. The ordering of the 6 potential flashes is:
			1. Accelerometer/magnetometer 1
			2. Accelerometer/magnetometer 2
			3. Thermocouple 1
			4. Thermocouple 2
			5. Hall effect sensor 1
			6. Hall effect sensor 2
		You will need to count off the seconds in your head after you see 
		the 5 quick green flashes to determine which error is being 
		signaled. If there are no errors, the red error LED will not light 
		during this time. 

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>	// built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>	// built in library, for SPI communications
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
// Various additional libraries for access to sleep mode functions
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
//******************************
// Data collection rate, enter a value here of 4, 2, or 1 (samples per second)
#define SAMPLES_PER_SECOND 1 // number of samples taken per second (4, 2, or 1)
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
uint32_t unixtimeArray[SAMPLES_PER_SECOND]; // store unixtime values temporarily
//byte fracSecArray[SAMPLES_PER_SECOND]; // store fracSec values temporarily
double tempArray[SAMPLES_PER_SECOND][8]; // store temperature values 128bytes
//int accelcompass1Array[SAMPLES_PER_SECOND][6]; // store accel/compass1 values
//int accelcompass2Array[SAMPLES_PER_SECOND][6]; // store accel/compass2 values

// Declare initial name for output files written to SD card
char filename[] = "YYYYMMDD_HHMM_00.csv";
// Define initial name of calibration file for accelerometers
char filenameCalib[] = "CAL0_YYYYMMDD_HHMM_00.csv";
//// Placeholder serialNumber
//char serialNumber[] = "SN00";
// Define a flag to show whether the serialNumber value is real or just zeros
//bool serialValid = false;
byte mcusr = 0; 	// variable to hold MCU status register codes
byte loopCount = 0; // counter to keep track of data sampling loops
byte fracSec = 0; // counter to keep track of fractional seconds
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
byte oldday; 		// used to keep track of midnight transition
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
// Flags to mark when sensors go bad
bool tc0fail = false;
bool tc1fail = false;
bool tc2fail = false;
bool tc3fail = false;
bool tc4fail = false;
bool tc5fail = false;
bool tc6fail = false;
bool tc7fail = false;

bool saveData = false; // Flag to tell whether to carry out write operation on SD card
bool oledScreenOn = true; // Flag to tell whether screens should be on/off

// Define two temperature limits for the thermocouples, values outside
// this range will trigger the error notification
float TClowerlimit = 0.0;
float TCupperlimit = 60.0;

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
	
	// Brownout detection process.
	mcusr = MCUSR; // Grab the contents of the MCUSR
	MCUSR = 0; // Reset MCUSR to 0 so it is ready for the next go-around.
	// Call the checkMCUSR function to check reason for this restart.
	// If only a brownout is detected, this function will put the board
	// into a permanent sleep that can only be reset with the reset 
	// button or a complete power-down.
//	checkMCUSR(mcusr, ERRLED);  // In the MusselTrackerlib library
								
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
    initFileName(newtime); // generate a file name
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

  // Take 4 temperature measurements to initialize the array
//  for (int i = 0; i < 4; i++){
//    tempArray[i][0] = thermocouple0.readCelsius();
//    tempArray[i][1] = thermocouple1.readCelsius();
//    tempArray[i][2] = thermocouple2.readCelsius();
//    tempArray[i][3] = thermocouple3.readCelsius();
//    tempArray[i][4] = thermocouple4.readCelsius();
//    tempArray[i][5] = thermocouple5.readCelsius();
//    tempArray[i][6] = thermocouple6.readCelsius();
//    tempArray[i][7] = thermocouple7.readCelsius();
//  }
	
	// Start 32.768kHz clock signal on TIMER2. 
	// Supply the current time value as the argument, returns 
	// an updated time
	newtime = startTIMER2(rtc.now());
	
	oldtime = newtime; // store the current time value
	oldday = oldtime.day(); // store the current day value
	
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
			// are still in the same second. If not,
			// the fracSec value should be reset to 0
			// and oldtime updated to equal newtime.
			if (oldtime.second() != newtime.second()) {
				fracSec = 0; // reset fracSec
				oldtime = newtime; // update oldtime
				loopCount = 0; // reset loopCount				
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
			
			// Save current time to unixtimeArray
			unixtimeArray[loopCount] = newtime.unixtime();
//			fracSecArray[loopCount] = fracSec;

			if (fracSec == 0) {
				// We only read the thermocouples and hall effect sensors
				// once per second regardless of the main sampling rate,
				// since these won't change fast enough to warrant reading
				// them (and waking them) more than once per second.
				temp0 = thermocouple0.readCelsius();
				temp1 = thermocouple1.readCelsius();
        temp2 = thermocouple2.readCelsius();
        temp3 = thermocouple3.readCelsius();
        temp4 = thermocouple4.readCelsius();
        temp5 = thermocouple5.readCelsius();
        temp6 = thermocouple6.readCelsius();
        temp7 = thermocouple7.readCelsius();
        
				// Sanity check the thermocouple values
				if ( (temp0 < TClowerlimit) | isnan(temp0) | (temp0 > TCupperlimit) ){
				   tc0fail = true; 
				} else { tc0fail = false;}
				if ( (temp1 < TClowerlimit) | isnan(temp1) | (temp1 > TCupperlimit) ) {
				   tc1fail = true; 
				} else { tc1fail = false;}
        if ( (temp2 < TClowerlimit) | isnan(temp2) | (temp2 > TCupperlimit) ){
           tc2fail = true; 
        } else { tc2fail = false;}
        if ( (temp3 < TClowerlimit) | isnan(temp3) | (temp3 > TCupperlimit) ) {
           tc3fail = true; 
        } else { tc3fail = false;}
        if ( (temp4 < TClowerlimit) | isnan(temp4) | (temp4 > TCupperlimit) ){
           tc4fail = true; 
        } else { tc4fail = false;}
        if ( (temp5 < TClowerlimit) | isnan(temp5) | (temp5 > TCupperlimit) ) {
           tc5fail = true; 
        } else { tc5fail = false;}
        if ( (temp6 < TClowerlimit) | isnan(temp6) | (temp6 > TCupperlimit) ){
           tc6fail = true; 
        } else { tc6fail = false;}
        if ( (temp7 < TClowerlimit) | isnan(temp7) | (temp7 > TCupperlimit) ) {
           tc7fail = true; 
        } else { tc7fail = false;}                        

			}  // end of if (fracSec == 0)
		
			// Now if loopCount is equal to the value in SAMPLES_PER_SECOND
			// (minus 1 for zero-based counting), then write out the contents
			// of the sample data arrays to the SD card. This should write data
			// every second.
			if (loopCount == (SAMPLES_PER_SECOND - 1)) {
				// Call the writeToSD function to output the data array contents
				// to the SD card
				writeToSD();

        // Add if(oledScreenOn) statement here and print current temps
        // to screens if they are on. 
        if (oledScreenOn){
          // Print stuff to screens
          oled1.home();
          oled1.set2X();
          oled1.clearToEOL();
          oled1.print(F("Ch0: "));
          oled1.print(temp0);
          oled1.println(F("C"));
          oled1.clearToEOL();
          oled1.print(F("Ch1: "));
          oled1.print(temp1);
          oled1.println(F("C"));
          oled1.clearToEOL();
          oled1.print(F("Ch2: "));
          oled1.print(temp2);
          oled1.println(F("C"));
          oled1.clearToEOL();
          oled1.print(F("Ch3: "));
          oled1.print(temp3);
          oled1.println(F("C"));

          oled2.home();
          oled2.set2X();
          oled2.clearToEOL();
          oled2.print(F("Ch4: "));
          oled2.print(temp4);
          oled2.println(F("C"));
          oled2.clearToEOL();
          oled2.print(F("Ch5: "));
          oled2.print(temp5);
          oled2.println(F("C"));
          oled2.clearToEOL();
          oled2.print(F("Ch6: "));
          oled2.print(temp6);
          oled2.println(F("C"));
          oled2.clearToEOL();
          oled2.print(F("Ch7: "));
          oled2.print(temp7);
          oled2.println(F("C"));
          
        }
				
#ifdef ECHO_TO_SERIAL
				// If ECHO_TO_SERIAL is defined at the start of the 
				// program, then this section will send updates of the
				// sensor values once per second.
				printTimeSerial(oldtime);
				Serial.print(F(" Temp0: "));
				Serial.print(temp0);

				Serial.print(F(" Temp1: "));
				Serial.print(temp1);

				delay(10);
				Serial.println();
				delay(5);

#endif			
      } // end of if (loopCount >= (SAMPLES_PER_SECOND - 1))                   
                        
                        
			// Increment loopCount after writing all the sample data to
			// the arrays
			loopCount++; 
				
			// Increment the fractional seconds count
	#if SAMPLES_PER_SECOND == 4
			fracSec = fracSec + 25;
	#endif

	#if SAMPLES_PER_SECOND == 2
			fracSec = fracSec + 50;
	#endif
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
				initCalibFile(newtime);
				Serial.print(F("Writing to "));
				Serial.println(filenameCalib);
				if (pressCount == 1){
					delay(5);
				} else if (pressCount == 2) {
					delay(5);
				}
			}
			
		break;
		//*****************************************************
		case STATE_CALIB_ACTIVE:
			// Write accelerometer/compass data to the calibration file
			newMillis = millis(); // get current millis value
			// If 10 or more milliseconds have elapsed, take a new
			// reading from the accel/compass
			if (newMillis >= prevMillis + 10) {
				prevMillis = newMillis; // update millis
				// Reopen logfile. If opening fails, notify the user
				if (!calibfile.isOpen()) {
					if (!calibfile.open(filenameCalib, O_RDWR | O_CREAT | O_AT_END)) {
						digitalWrite(ERRLED, HIGH); // turn on error LED
					}
				}
				// Choose which accel to sample based on pressCount value
				switch (pressCount) {
					case 1:
						digitalWrite(GREENLED, HIGH);

						digitalWrite(GREENLED, LOW);
					break;
					
					case 2:
						digitalWrite(GREENLED, HIGH);
						
						digitalWrite(GREENLED, LOW);
					break;
				} // end of switch (pressCount)
			}
			
			// The user can press button1 again to end calibration mode
			// This would set buttonFlag true, and cause the if statement
			// below to execute
 			if (button1Flag) {
				button1Flag = false;
				calibfile.close(); // close and save the calib file
				Serial.println(F("Saving calib file"));
				// Set the accel/magnetometer back to normal slow
				// mode (50Hz accel with antialias filter, 6.25Hz magnetometer)
				if (pressCount == 1){

					delay(5);

				} else if (pressCount == 2) {

					delay(5);

				}
				initFileName(newtime); // open a new data file
				mainState = STATE_DATA; // return to STATE_DATA
				// Flash both LEDs 5 times to let user know we've exited
				// calibration mode
				for (byte i = 0; i < 5; i++){
					digitalWrite(ERRLED, HIGH);
					digitalWrite(GREENLED, HIGH);
					delay(100);
					digitalWrite(ERRLED, LOW);
					digitalWrite(GREENLED, LOW);
					delay(100);
				}
			} // end of if(button1Flag) statement
			
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
			initFileName( rtc.now() ); // Open a new output file
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

//-------------- initFileName --------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter. 
// The character array 'filename' was defined as a global array 
// at the top of the sketch in the form "YYYYMMDD_HHMM_00.csv"
void initFileName(DateTime time1) {
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filename array
	for (byte i = 0; i < 4; i++){
		filename[i] = buf[i];
	}
	// Insert the month value
	if (time1.month() < 10) {
		filename[4] = '0';
		filename[5] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filename[4] = (time1.month() / 10) + '0';
		filename[5] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filename[6] = '0';
		filename[7] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filename[6] = (time1.day() / 10) + '0';
		filename[7] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filename[8] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filename[9] = '0';
		filename[10] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filename[9] = (time1.hour() / 10) + '0';
		filename[10] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filename[11] = '0';
		filename[12] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filename[11] = (time1.minute() / 10) + '0';
		filename[12] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filename[13] = '_';

	// Next change the counter on the end of the filename
	// (digits 14+15) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[14] = i / 10 + '0';
		filename[15] = i % 10 + '0';
		
		if (!sd.exists(filename)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(5);
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	//------------------------------------------------------------
  // Write 1st header line
  logfile.println(F("POSIXt,DateTime,TC0,TC1,TC2,TC3,TC4,TC5,TC6,TC7"));
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close(); // force the data to be written to the file by closing it
} // end of initFileName function


//-------------- initCalibFile ---------------------------------------------------
// initCalibFile - a function to create a filename for the SD card based
// The character array 'filenameCalib' was defined as a global array 
// at the top of the sketch in the form: 
// filenameCalib[] = "CAL0_YYYYMMDD_HHMM_00.csv";
void initCalibFile(DateTime time1) {
	if (pressCount == 1) {
		filenameCalib[3] = '1';
	} else if (pressCount == 2) {
		filenameCalib[3] = '2';
	}
	char buf[5];
	// Integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filenameCalib array
	byte count = 0;
	for (byte i = 5; i < 9; i++){
		filenameCalib[i] = buf[count];
		count++;
	}
	// Insert the month value
	if (time1.month() < 10) {
		filenameCalib[9] = '0';
		filenameCalib[10] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filenameCalib[9] = (time1.month() / 10) + '0';
		filenameCalib[10] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filenameCalib[11] = '0';
		filenameCalib[12] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filenameCalib[11] = (time1.day() / 10) + '0';
		filenameCalib[12] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filenameCalib[13] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filenameCalib[14] = '0';
		filenameCalib[15] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filenameCalib[14] = (time1.hour() / 10) + '0';
		filenameCalib[15] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filenameCalib[16] = '0';
		filenameCalib[17] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filenameCalib[16] = (time1.minute() / 10) + '0';
		filenameCalib[17] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filenameCalib[18] = '_';	
	// If there is a valid serialnumber, insert it into 
	// the file name in positions 17-20. 
//	if (serialValid) {
//		byte serCount = 0;
//		for (byte i = 22; i < 26; i++){
//			filenameCalib[i] = serialNumber[serCount];
//			serCount++;
//		}
//	}	
	// Next change the counter on the end of the filenameCalib
	// (digits 19+20) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filenameCalib[19] = i / 10 + '0';
		filenameCalib[20] = i % 10 + '0';
		
		if (!sd.exists(filenameCalib)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!calibfile.open(filenameCalib, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(5);
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	
	// Write aheader line to the SD file
	calibfile.println(F("millis,a.x,a.y,a.z,m.x,m.y,m.z"));
	// Update the file's creation date, modify date, and access date.
	calibfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.close(); // Force the data to be written to the file by closing it
} // end of initCalibFile function 



//------------- writeToSD -----------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (void) {

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

	// Step through each element of the sample data arrays
	// and write them to the SD card
	for (byte i = 0; i < SAMPLES_PER_SECOND; i++) {
		// Write the unixtime
		logfile.print(unixtimeArray[i], DEC); // POSIX time value
		logfile.print(F(","));
    printTimeToSD(logfile, unixtimeArray[i]); // human-readable 
		logfile.print(F(","));

		// Print thermocouple 0 value
		logfile.print(temp0); 
		logfile.print(F(","));
		logfile.print(temp1); 
    logfile.print(F(","));
    logfile.print(temp2); 
    logfile.print(F(","));
    logfile.print(temp3); 
    logfile.print(F(","));
    logfile.print(temp4); 
    logfile.print(F(","));
    logfile.print(temp5); 
    logfile.print(F(","));
    logfile.print(temp6); 
    logfile.print(F(","));
    logfile.print(temp7); 
    logfile.println();
	}
	// logfile.close(); // force the buffer to empty
	  DateTime t1 = DateTime(unixtimeArray[0]);
	  // If the seconds value is 30, update the file modified timestamp
	  if (t1.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, t1.year(),t1.month(),t1.day(),t1.hour(),t1.minute(),t1.second());
	  }
}






//---------- startTIMER2 ----------------------------------------------------
// startTIMER2 function
// Starts the 32.768kHz clock signal being fed into XTAL1 from the
// real time clock to drive the
// quarter-second interrupts used during data-collecting periods. 
// Supply a current DateTime time value. 
// This function returns a DateTime value that can be used to show the 
// current time when returning from this function. 
DateTime startTIMER2(DateTime currTime){
	TIMSK2 = 0; // stop timer 2 interrupts

	rtc.enable32kHz(true);
	ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
	// The EXCLK bit should only be set if you're trying to feed the
	// 32.768 clock signal from the Chronodot into XTAL1. 

	ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid
	// clobbering the EXCLK bit that might already be set. This tells 
	// TIMER2 to take its clock signal from XTAL1/2
	TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
	
	// Set up TCCR2B register (Timer Counter Control Register 2 B) to use the 
	// desired prescaler on the external 32.768kHz clock signal. Depending on 
	// which bits you set high among CS22, CS21, and CS20, different 
	// prescalers will be used. See Table 18-9 on page 158 of the AVR 328P 
	// datasheet.
	//  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
	// no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)
	//  TCCR2B = _BV(CS20) ; 
	// prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
	//  TCCR2B = _BV(CS21) ; 
#if SAMPLES_PER_SECOND == 4
	// prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
	TCCR2B = _BV(CS21) | _BV(CS20); 
#endif

#if SAMPLES_PER_SECOND == 2
	TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
#endif

#if SAMPLES_PER_SECOND == 1
    TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 seconds
#endif

	// Pause briefly to let the RTC roll over a new second
	DateTime starttime = currTime;
	// Cycle in a while loop until the RTC's seconds value updates
	while (starttime.second() == currTime.second()) {
		delay(1);
		currTime = rtc.now(); // check time again
	}

	TCNT2 = 0; // start the timer at zero
	// wait for the registers to be updated
	while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} 
	TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
	TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
	// TIMER2 will now create an interrupt every time it rolls over,
	// which should be every 0.25, 0.5 or 1 seconds (depending on value 
	// of SAMPLES_PER_SECOND) regardless of whether the AVR is awake or asleep.
	return currTime;
}


//------------------printTimeSerial------------------------------------------
void printTimeSerial(DateTime now){
// printTime function takes a DateTime object from
// the real time clock and prints the date and time 
// to the serial monitor. 
  Serial.print(now.year(), DEC);
    Serial.print('-');
  if (now.month() < 10) {
    Serial.print(F("0"));
  }
    Serial.print(now.month(), DEC);
    Serial.print('-');
    if (now.day() < 10) {
    Serial.print(F("0"));
  }
  Serial.print(now.day(), DEC);
    Serial.print(' ');
  if (now.hour() < 10){
    Serial.print(F("0"));
  }
    Serial.print(now.hour(), DEC);
    Serial.print(':');
  if (now.minute() < 10) {
    Serial.print("0");
  }
    Serial.print(now.minute(), DEC);
    Serial.print(':');
  if (now.second() < 10) {
    Serial.print(F("0"));
  }
    Serial.print(now.second(), DEC);
  // You may want to print a newline character
  // after calling this function i.e. Serial.println();

}


//--------------------goToSleep--------------------------------------------------
// goToSleep function. When called, this puts the AVR to
// sleep until it is awakened by an interrupt (TIMER2 in this case)
// This is a higher power sleep mode than the lowPowerSleep function uses.
void goToSleep() {
  // Create three variables to hold the current status register contents
  byte adcsra, mcucr1, mcucr2;
  // Cannot re-enter sleep mode within one TOSC cycle. 
  // This provides the needed delay.
  OCR2A = 0; // write to OCR2A, we're not using it, but no matter
  while (ASSR & _BV(OCR2AUB)) {} // wait for OCR2A to be updated
  // Set the sleep mode to PWR_SAVE, which allows TIMER2 to wake the AVR
  set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  adcsra = ADCSRA; // save the ADC Control and Status Register A
  ADCSRA = 0; // disable ADC by zeroing out the ADC status register
  sleep_enable();
  // Do not interrupt before we go to sleep, or the
  // ISR will detach interrupts and we won't wake.
  noInterrupts ();
  
  wdt_disable(); // turn off the watchdog timer
  
  //ATOMIC_FORCEON ensures interrupts are enabled so we can wake up again
  ATOMIC_BLOCK(ATOMIC_FORCEON) { 
    // Turn off the brown-out detector
    mcucr1 = MCUCR | _BV(BODS) | _BV(BODSE); 
    mcucr2 = mcucr1 & ~_BV(BODSE);
    MCUCR = mcucr1; //timed sequence
    // BODS stays active for 3 cycles, sleep instruction must be executed 
    // while it's active
    MCUCR = mcucr2; 
  }
  // We are guaranteed that the sleep_cpu call will be done
  // as the processor executes the next instruction after
  // interrupts are turned on.
  interrupts ();  // one cycle, re-enables interrupts
  sleep_cpu(); //go to sleep
  //wake up here
  sleep_disable(); // upon wakeup (due to interrupt), AVR resumes here
  ADCSRA = adcsra; // re-apply the previous settings to the ADC status register

}


//---------------printTimeToSD----------------------------------------
// printTimeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void printTimeToSD (SdFile& mylogfile, DateTime tempTime) {
    // Write the date and time in a human-readable format
    // to the file on the SD card. 
    mylogfile.print(tempTime.year(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.month() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.month(), DEC);
    mylogfile.print(F("-"));
    if (tempTime.day() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.day(), DEC);
    mylogfile.print(F(" "));
    if (tempTime.hour() < 10){
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.hour(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.minute() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.minute(), DEC);
    mylogfile.print(F(":"));
    if (tempTime.second() < 10) {
      mylogfile.print("0");
    }
    mylogfile.print(tempTime.second(), DEC);
}
