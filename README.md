## Thermocouple_datalogger_2
### Luke Miller 2017

This repository contains code to run a redesigned 8-channel thermocouple
datalogger based on Arduino-type hardware. Currently there are 2 revisions,
A and B, which differ in the type of thermocouple-to-digital chip they use.
Revision A uses the older MAX31855K chip, while Revision B uses the more
recent MAX31856 that effectively superseded the MAX31855 series. 

The directory `TempTracker` contains the primary files used to run a 
datalogger board, while the `testing` directory contains various utilities
used to test the basic functions of a datalogger board or to help calibrate
a board. 

## Required libraries:
https://github.com/millerlp/TClib2
https://github.com/millerlp/Adafruit_MAX31856 - fork of the original Adafruit_MAX31856 library, mainly changed to return NAN temperature values automatically if there is an error. 
https://github.com/millerlp/RTClib - to run the onboard real time clock chip DS3231M
https://github.com/greiman/SdFat - for reading/writing SD cards
https://github.com/greiman/SSD1306Ascii - for writing to OLED displays

## Calibrating
The MAX3185x thermocouple chips are relatively accurate from the factory, and for
most purposes this will be sufficient, if you don't need absolute accuracy less than
1-2C. If you require sub-degree accurracy, calibration will be necessary, and this means
calibrating both the datalogger board and the individual thermocouple leads you use. 

Calibrating the datalogger board actually requires calibrating each of the 8 onboard 
MAX31856 chips separately, using a separate thermocouple calibrator such as the
Omega Engineering CL3515R. These devices are capable of outputting a millivolt signal
equivalent to what the relevant thermocouple type (K, T, J etc) would put out at a
specified temperature. The calibrator comes with a thermocouple cable that plugs 
directly into the thermocouple plugs on the datalogger board, mimicing a real 
thermocouple. The program `Calibration_routine_OLED_RevB` (or the RevA version) is meant
to be used with the external calibrator. After generating a set of calibration data for
a set of known temperatures, you can use the observed temperatures (recorded by the 
datalogger) and the known temperatures (from the calibrator) to fit a straight-line 
regression, and get the intercept (aka offset) and slope of the relationship between 
observed and expected readings. 

The program `Store_calibrations.ino` can then be used 
to upload the full set of 8 offset values and 8 slope values for the 8 MAX3185x 
chips to the datalogger's EEPROM onboard memory. The datalogger board will then have these
calibration values available to use every time it boots up. Typically I find that an 
individual MAX31855K or MAX31856 chip has a slope very near to 1.0 (ideal), but the 
intercept (aka offset) may often be +/- 0.5 to 1C off from ideal (0C offset). Therefore, calibration
is useful if you want the 8 individual channels on a board to be directly comparable. 

