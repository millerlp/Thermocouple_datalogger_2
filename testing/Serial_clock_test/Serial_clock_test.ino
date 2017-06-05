/* Serial_clock_test
 * 
 * For Thermocouple_datalogger_RevA hardware
 * Luke Miller June 2017
 * 
 * Test functionality of the serial monitor, buttons,
 * real time clock etc. 
 * 
 */

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib

#define REDLED A2
#define GRNLED A3
#define BUTTON1 2
#define BUTTON2 3

// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C
#define I2C_ADDRESS2 0x3D
SSD1306AsciiWire oled1; // create OLED display object
SSD1306AsciiWire oled2; // create OLED display object

//*************
// Create real time clock object
RTC_DS3231 rtc;
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
char buf[20]; // declare a string buffer to hold the time result

unsigned long debounceTime = 30; // debounce time, milliseconds
volatile unsigned long debounceButton1;
volatile unsigned long debounceButton2;


volatile bool BUTTON1flag = false;
volatile bool BUTTON2flag = false;

void setup() {
  // Set button1 as an input
  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  // Set up the LEDs as output
  pinMode(REDLED,OUTPUT);
  digitalWrite(REDLED, LOW);
  pinMode(GRNLED,OUTPUT);
  digitalWrite(GRNLED, LOW);
  // Initialize the real time clock DS3231M
  Wire.begin(); // Start the I2C library with default options
  rtc.begin();  // Start the rtc object with default options
  Serial.begin(57600);
  Serial.println(F("Hello"));
  newtime = rtc.now(); // read a time from the real time clock
  printTimeSerial(rtc.now()); // print time to serial monitor
  Serial.println();
  if ( (newtime.year() < 2017) | (newtime.year() > 2035) ) {
    // There is an error with the clock, halt everything
    while(1){
    // Flash the error led to notify the user
    // This permanently halts execution, no data will be collected
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(100);
    } 
  }

  
      // Create a string representation of the date and time, 
      // which will be put into 'buf'. 
  newtime.toString(buf, 20); 

  // Start up the oled display
  oled1.begin(&Adafruit128x64, I2C_ADDRESS1);
  oled1.set400kHz();  
  oled1.setFont(Adafruit5x7);    
  oled1.clear(); 
  oled1.home();
  oled1.print(buf);
  oled1.println();
  oled1.set2X();
  oled1.print(F("OLED 1"));

  // Start up the 2nd oled display
  oled2.begin(&Adafruit128x64, I2C_ADDRESS2);
  oled2.set400kHz();  
  oled2.setFont(Adafruit5x7);    
  oled2.clear(); 
  oled2.home();
  oled2.print(buf);
  oled2.println();
  oled2.set2X();
  oled2.print(F("OLED 2"));

  // Flash green LED to show that we just booted up
  for (byte i = 0; i < 3; i++){
    digitalWrite(GRNLED, HIGH);
    delay(100);
    digitalWrite(GRNLED, LOW);
    delay(100);
  } 

  // start the button1 interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON1), buttonFunc1, LOW);
  attachInterrupt(digitalPinToInterrupt(BUTTON2), buttonFunc2, LOW);

  oldtime = newtime;
}
//*****************************************************************
void loop() {
  
  if (BUTTON1flag){
    if ( (millis() - debounceButton1) > debounceTime){
      // button has been low for longer than debounceTime
      // so count this as a real press. Now determine when
      // the button is released to count as a completed press

      if (digitalRead(BUTTON1)) {
        // This section only triggers once the button goes
        // high again. This should mark a release of the
        // button. 
        digitalWrite(REDLED, !digitalRead(REDLED));
        Serial.println(F("BUTTON1"));
        BUTTON1flag = false;
        // If the button continues to be held low, 
        // this loop should be ignored because 
        // BUTTON1flag has been reset to false.
        // When the button is released (high), it 
        // should not trigger an interrupt, so we
        // can re-attach the interrupt now 
        attachInterrupt(digitalPinToInterrupt(BUTTON1), buttonFunc1, LOW);
      }
    }
  }  // end of if (BUTTON1flag)
  //-------------------------------------
  if (BUTTON2flag){
    if ( (millis() - debounceButton2) > debounceTime){
      // button has been low for longer than debounceTime
      // so count this as a real press. Now determine when
      // the button is released to count as a completed press

      if (digitalRead(BUTTON2)) {
        // This section only triggers once the button goes
        // high again. This should mark a release of the
        // button. 
        digitalWrite(GRNLED, !digitalRead(GRNLED));
        Serial.println(F("BUTTON2"));
        BUTTON2flag = false;
        // If the button continues to be held low, 
        // this loop should be ignored because 
        // BUTTON1flag has been reset to false.
        // When the button is released (high), it 
        // should not trigger an interrupt, so we
        // can re-attach the interrupt now 
        attachInterrupt(digitalPinToInterrupt(BUTTON2), buttonFunc2, LOW);
      }
    }
  }
  //-------------------------
  newtime = rtc.now();
  if ( (newtime.unixtime() - oldtime.unixtime()) >= 1 ){
   // If enough time has elapsed, update the OLED displays of the time
    oldtime = newtime;
    newtime.toString(buf, 20); // update date/time string in 'buf'
      // Now extract the time by making another character pointer that
      // is advanced 10 places into buf to skip over the date. 
      char *subbuf = buf + 10;

    oled1.home();
    oled1.set1X();
    oled1.clear(60,128,0,6); // clear time string section of display
    // the above will clear from column 60 to 128, and rows 0 to 6, which
    // should blank out the default 5x7 font used to display the time
    oled1.print(subbuf); // print time string

    oled2.home();
    oled2.set1X();    
    oled2.clear(60,128,0,6); // clear time string section of display
    oled2.print(subbuf); // print time string    
  }
  
}

//----------------------------------------------------------------------
void buttonFunc1(void){
  detachInterrupt(digitalPinToInterrupt(BUTTON1));
  debounceButton1 = millis();  
  BUTTON1flag = true; // set true to trigger debounce check in main loop
}

void buttonFunc2(void){
  detachInterrupt(digitalPinToInterrupt(BUTTON2));
  debounceButton2 = millis();
  BUTTON2flag = true; // set true to trigger debounce check in main loop
}



//----------------------------------------------------------------------
void printTimeSerial(DateTime now){
//------------------------------------------------
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
