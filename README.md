## AD4112-ESP32-SPI-MAIN ##
Driver and interface code for the Analog Devices AD411x 24-bit ADC unit. This code was written in VS Code using the ESP IDF. The target unit is the ESP32-C3 MINI. This code was written around the AD4112 unit but should work with the other models in the same series with a few adjustments as necessary.

### Credits ###
Michael Dawson 

michaelshanedawson@gmail.com

### About ###
The device is interfaced via SPI, Mode 3, MSB first. The registers are typically 2 or 3 bytes in length with the exception of the COMMS and the Status registers which are only a single byte.

### Changelog ###
* v0.01 1/26/2024 - Initial code creation. Can query and serial print the chip ID. Had the reverse the byte order on the SPI RX data, not sure why at the moment.
* v0.02 1/28/2024 - Added ADC configurations. Need to update and make easier to follow but so far seems functional. The ADC data output is not accurate or even close, need to evaluate and solve. Could be a configuration issue.
