#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii

// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C
#define I2C_ADDRESS2 0x3D
SSD1306AsciiWire oled1; // create OLED display object
SSD1306AsciiWire oled2; // create OLED display object


void setup() {
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

  //---------------------------------------
  oled1.home();
  oled1.clear();
  oled1.set2X();
  for (int i = 0; i < 4; i++){
      oled1.print(F("Ch"));
      oled1.print(i);
      oled1.print(": ");
      oled1.print(23.45+i);
      oled1.println(F("C"));
  }
  oled1.home();
  // Clear columns 60 to 128 (end) of screen 
  // for the current row + the row beneath it. At
  // 2x font, this will clear a single row of large text
  oled1.clear(60, 128, oled1.row(), oled1.row()+1);

  oled1.setRow(4);
  oled1.clear(60,128,oled1.row(),oled1.row()+1);
  oled1.print("TEST");

}

void loop() {
  // put your main code here, to run repeatedly:

}
