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
* https://github.com/millerlp/TClib2
* https://github.com/millerlp/Adafruit_MAX31856 - fork of the original Adafruit_MAX31856 library, mainly changed to return NAN temperature values automatically if there is an error. 
* https://github.com/millerlp/RTClib - to run the onboard real time clock chip DS3231M
* https://github.com/greiman/SdFat - for reading/writing SD cards
* https://github.com/greiman/SSD1306Ascii - for writing to OLED displays

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
is useful if you want the 8 individual channels on a board to be directly comparable to 
each other at the sub-degree level. The nominal temperature resolution of a MAX31855K chip is 0.25C (14-bit A-D converter) and the MAX31856 is 0.0078125 (19-bit A-D converter), but 
chip-to-chip variation from the factory can easily swamp that and only get you 
within +/- 2C of the true temperature in the normal biological range of temperatures. Careful calibration and housing the datalogger board in a thermally stable container 
should get you within 0.1C across channels. 

## A word about the housing and thermal stability

The thermocouple-to-digital chips rely on an internal temperature sensor in the chip
itself to measure the "cold junction" temperature. This temperature is ideally equivalent
to the temperature at the point where the thermocouple plugs into the board and makes 
the transition from special thermocouple wire to the plain copper of the circuit board
(which is a thermocouple junction by definition). An accurate measurement of the 
distant ("hot") end of the thermocouple probe relies on the MAX3185x chip also knowing 
what the cold junction temperature is quite accurately. If the MAX3185x chip internal
temperature is a few degrees different than the thermocouple plug on the board, the
registered thermocouple temperature will also be off. Ideally, a block of material
that is thermally conductive (an "isothermal block") would be used to connect all 8 
of the MAX3185x chips and their respective thermocouple plugs, and thus keep all of 
those points at the same temperature. A bar of aluminum, copper, or brass, mounted so that 
it sits across the top of all the MAX3185x chips and also rests against the 
thermocouple plug terminals would be best, although these should not all be in 
electrical contact. To thermally couple the various parts without creating short 
circuits, adhesive thermal gap filler (often used to attach heat sinks to circuit 
boards and chips) can be used. This material is electrically non-conductive, but does
a reasonable job of allowing heat to transfer across it, and has the added benefit of
filling small gaps that might exist between an isothermal block and the various parts
due to height differences or variation in how flat the chips sit on the board. The RevB
board has two holes on the edges of the board positioned to help bolt an isothermal 
block across the tops of the MAX31856 chips. 

After provisioning an isothermal block, it would also be useful to then enclose the 
datalogger board in a housing that will stay relatively temperature-stable, or at 
least won't change temperature rapidly at different places. Surrounding the board in
insulating styrofoam may be desirable, and then placing that inside a plastic or metal housing that only exposes the front of the thermocouple connectors to the outside air. 
The RevB board is designed so that panel-mount buttons can be attached to wire leads and  
plugged into the board (via standard 0.1" headers), so that the buttons can be
mounted to a housing. The 2 OLED displays can be attached via 4-wire cables in the 
same manner. The rear/bottom edge of the board (opposite the thermocouple connectors) will 
need to be accessible through a housing wall so that the serial port, micro SD card slot, 
and 2.1mm DC power plug are accessible. 

