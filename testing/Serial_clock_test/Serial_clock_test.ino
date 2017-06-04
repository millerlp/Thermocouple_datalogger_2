/* Serial_clock_test
 * Need to refine debounce routine
 * For Thermocouple_datalogger_RevA hardware
 * Luke Miller June 2017
 * 
 * Test functionality of the serial monitor, buttons,
 * real time clock etc. 
 * 
 */

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib

#define REDLED A2
#define GRNLED A3
#define BUTTON1 2
#define BUTTON2 3

//*************
// Create real time clock object
RTC_DS3231 rtc;
DateTime newtime; // used to track time in main loop

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
  if (newtime.year() < 2017 | newtime.year() > 2035) {
    // There is an error with the clock, halt everything
    while(1){
    // Flash the error led to notify the user
    // This permanently halts execution, no data will be collected
      digitalWrite(REDLED, !digitalRead(REDLED));
      delay(100);
    } 
  }



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

}

void loop() {
  // put your main code here, to run repeatedly:
  if (BUTTON1flag){
    digitalWrite(REDLED, !digitalRead(REDLED));
    Serial.println(F("BUTTON1"));
    BUTTON1flag = false;
    attachInterrupt(0, buttonFunc1, LOW);
  }
  if (BUTTON2flag){
      digitalWrite(GRNLED, !digitalRead(GRNLED));
      Serial.println(F("BUTTON2"));
      BUTTON2flag = false; // reset the flag
      attachInterrupt(1, buttonFunc2, LOW); // restart the interrupt
  }
  
}

//----------------------------------------------------------------------
void buttonFunc1(void){
  detachInterrupt(digitalPinToInterrupt(BUTTON1));
  delay(20);
  if (digitalRead(BUTTON1) == LOW){
    BUTTON1flag = true;
  } else {
    BUTTON1flag = false;
  }
  
}

void buttonFunc2(void){
  detachInterrupt(digitalPinToInterrupt(BUTTON2));
  delay(20);
  if (digitalRead(BUTTON2) == LOW){
    BUTTON2flag = true;
  } else {
    BUTTON2flag = false;
  }
  
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
