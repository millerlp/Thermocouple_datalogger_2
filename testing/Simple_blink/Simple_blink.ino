/* Quick blink test
 *  
 *  For Thermocouple_datalogger_RevA and RevB hardware
 *  Luke Miller June 2017 
 *  
 *  
 */

#define REDLED A2 // Using the arduino pin names (A2, A3)
#define GRNLED A3 // Using the arduino pin names (A2, A3)
 
void setup() {
  pinMode(REDLED, OUTPUT);
  pinMode(GRNLED, OUTPUT);
  digitalWrite(GRNLED, LOW);
  digitalWrite(REDLED, LOW);

}

void loop() {
  
  digitalWrite(REDLED, !digitalRead(REDLED));
  digitalWrite(GRNLED, !digitalRead(GRNLED));
  delay(1000);

}
