# Character_Colour_Clock_SK6812_1284-MKR1010_V044
This Arduino code controls the ATMEGA328 (Nano) ATMEGA1284 chip and Arduino MKR1010 that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05 and HM10 BLE
- DCF77 module DCF-2
- LCD
- WIFI on MKR 1010 to get NTP time

Load and install in IDE:
http://arduino.esp8266.com/stable/package_esp8266com_index.json  (ESP-12)
https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json ATMEGA1260 and 644
https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json ATTINY
Arduino SAMD for MKR1010

The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module

Arduino Uno, Nano with SK6812 LEDs: Program size must stay below approx 23572, 1129 bytes bytes with 144 LEDs.
************************************************************************************
To set the time with the rotary button:
One press on rotary: UUR is flashing -> turn to set hour
Second press on rotary: HT IS WAS flashes --> turn to set minute
the following presses are various display modes with the DIGITAL choice as second last 
Last press on rotary: many letters turns on --> turn to set intensity level of the LEDs
Press again of wait for one minute to return to normal operation
Displaymodes (Qn):
DEFAULTCOLOUR = 0; Yellow text.  HET Green to Red hour IS WAS Green to Red Minute  
HOURLYCOLOUR  = 1; Every hour other colour text. HET Green to Red hour IS WAS Green to Red Minute         
WHITECOLOR    = 2; All white text
OWNCOLOUR     = 3; All own colour
OWNHETISCLR   = 4; All own colour. HET Green to Red hour IS WAS Green to Red Minute
WHEELCOLOR    = 5; Every second another colour of rainbow
DIGITAL       = 6; Digital display
************************************************************************************
 Author .: Ed Nieuwenhuys
 
 Changes.: 0.25 0.26 Optimised and ready for SK6812 RGBW
 
 Changes.: 0.27 Colour changes per minute. New Function HSVtoRGB
 
 Changes.: 0.28b 144 LED EJN_SK6821 clock with Digits in display BLUETOOTH not possible bug with Softwareserial & Adafruit_neopixel. connect BT to pin 0 & 1
 
 Changes.: 0.29 FAB_LED library added. Doet het redelijk. Kost meer global variabel space. Serial en BT werkt ook niet met Bluetooth enabled
 
 Changes.: 0.30 Digits in matrix.
 
 Changes.: 0.31 Bluetooth to pin 0 and 1. Removed software serial. Added EJN to defines
 
 Changes.: 0.32 3D array in PROGMEM to avoid memory problem with NEOpixel library. 
 
 Changes.: 0.33 Program size must stay below approx 23572, 1129 bytes with 144 LEDs. BLUETOOTH on pin 0 & 1, No software serial
                Program compatible with ATMEGA 1284
                
 Changes.: 0.34-c Added FR, DE en UK
 
 Changes.: 0.35 Ready for testing in four languages clock.
 
 Changes.: 0.36 Added MKR1010.  #if defined(ARDUINO_SAMD_MKRWIFI1010)
 
 Changes.: 0.37 Added webserver for MKR1010
 
 Changes.: 0.38 ATMEGA 1280 gave problems with Bluetooth. Solved by adding Softwareserial.h at the top.  Probably the double #ifdef
 
 Changes.: 0.39 NB. Bluetooth with 1280 boards  on serial port pins  0 en 1 Softwareserial.h not needed anymore
 
 Changes.: 0.39 DCF HIGH LOW in definition, introduced shorter #define ATmega644_1284
 
 Changes.: 0.40 Repaired 'press rotary' to set time. Moved SetSecondColour() and SetMinuteColour() occurences to DisplayTime()
 
 Changes.: 0.41 Stores chosen display mode in EEPROM. Reset function added.
 
 Changes.: 0.42 Changed time settings in rotary one press change hour, 2 presses change minutes, then display choices and last light intensity
 
 Changes.: 0.43 Stable version Removed HET IF WAS turning off in display. Store Own colour in EEPROM. Cleaned up code. Software for ATMEGA 1280 25cm Klok No36 en No37
 
 Changes.: 0.44 #define WS2812 and #define SK6812 changed to #define LED2812 #define LED6812 remove WS1812 library and uses Adafruit_NeoPixel for both LED types
                Zet_Pixel() changed
