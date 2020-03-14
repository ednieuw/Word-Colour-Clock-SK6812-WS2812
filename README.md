# Character Colour Clock WS2812/SK6812 ATMEGA 328/1284 MKR1010
This Arduino code controls the ATMEGA328 (Nano) ATMEGA1284 chip and Arduino MKR1010 that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05 and HM10 BLE
- DCF77 module DCF-2
- LCD
- WIFI on MKR 1010 to get NTP time

Load and install in IDE:<br>
http://arduino.esp8266.com/stable/package_esp8266com_index.json  (ESP-12)<br>
https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json ATMEGA1260 and 644<br>
https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json ATTINY
Arduino SAMD for MKR1010<br>

<p>This word clock program is suitable and tested for use with Arduino UNO, Nano, Nano Every, ATMEGA328, ATMEGA1284 chip, Arduino MKR 1010. <br>
Other Arduino variant probably also work.<br>
The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock. <br>
With a rotary encoder time and several display modes can also be changed.<br>
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module Arduino Uno, Nano with WS2812 or SK6812 LEDs: <br>
Program size must stay below approx 23572 bytes  when 144 LEDs are used. Due to a Adafruit Neopixel bug the usage of memory by the LEDs is not subtracted from the avaiable memory. <br>
With the Nano Every, Mega or chips with larger memory program size is no issue.<br>
In an Arduino MKR1010 Wifi can be used to receive the time from a timeserver<br>
  <br>
  To set the time with the rotary button: <br>
  One press on rotary: UUR is flashing -&gt; turn to set hour.<br>
  Second press on rotary: HET IS WAS flashes --&gt; turn to set minute the following presses are various display modes with the DIGITAL choice as second last.<br>
  Last press on rotary: many letters turns on --&gt; turn to set intensity level of the LEDs Press again of wait for one minute to return to normal operation.<br>
  Displaymodes (Qn): <br>
  DEFAULTCOLOUR = 0; 
Yellow text. HET Green to Red hour IS WAS Green to Red Minute<br>
HOURLYCOLOUR = 1; Every hour other colour text. HET Green to Red hour IS WAS Green to Red Minute <br>
WHITECOLOR = 2; All white text <br>
OWNCOLOUR = 3; All own colour<br>
OWNHETISCLR = 4; All own colour. HET Green to Red hour IS WAS Green to Red Minute <br>
WHEELCOLOR = 5; Every second another colour of rainbow<br>
DIGITAL = 6; Digital display </p>
************************************************************************************<br>
Changes.: 0.24 Built in Clock Michelle en JoDi klok. <br>
Changes.: 0.25 0.26 Optimised and ready for SK6812 RGBW<br>
Changes.: 0.27 Colour changes per minute. New Function HSVtoRGB<br>
Changes.: 0.28b 144 LED EJN_SK6821 clock with Digits in display BLUETOOTH not possible bug with Softwareserial &amp; Adafruit_neopixel. connect BT to pin 0 &amp; 1<br>
Changes.: 0.29 FAB_LED library added. Doet het redelijk. Kost meer global variabel space. Serial en BT werkt ook niet met Bluetooth enabled<br>
Changes.: 0.30 Digits in matrix.<br>
Changes.: 0.31 Bluetooth to pin 0 and 1. Removed software serial. Added EJN to defines<br>
Changes.: 0.32 3D array in PROGMEM to avoid memory problem with NEOpixel library. <br>
Changes.: 0.33 Program size must stay below approx 23572, 1129 bytes with 144 LEDs. BLUETOOTH on pin 0 &amp; 1, No software serial<br>
Program compatible with ATMEGA 1284<br>
Changes.: 0.34-c Added FR, DE en UK<br>
Changes.: 0.35 Ready for testing in four languages clock. <br>
Changes.: 0.36 Added MKR1010. #if defined(ARDUINO_SAMD_MKRWIFI1010)<br>
Changes.: 0.37 Added webserver for MKR1010<br>
Changes.: 0.38 ATMEGA 1280 gave problems with Bluetooth. Solved by adding Softwareserial.h at the top. Probably the double #ifdef <br>
Changes.: 0.39 NB. Bluetooth with 1280 boards on serial port pins 0 en 1 Softwareserial.h not needed anymore<br>
Changes.: 0.39 DCF HIGH LOW in definition, introduced shorter #define ATmega644_1284<br>
Changes.: 0.40 Repaired 'press rotary' to set time. Moved SetSecondColour() and SetMinuteColour() occurences to DisplayTime()<br>
Changes.: 0.41 Stores chosen display mode in EEPROM. Reset function added. <br>
Changes.: 0.42 Changed time settings in rotary one press change hour, 2 presses change minutes, then display choices and last light intensity<br>
Changes.: 0.43 Stable version Removed HET IF WAS turning off in display. Store Own colour in EEPROM. Cleaned up code. Software for ATMEGA 1280 25cm Klok No36 en No37<br>
Changes.: 0.44 #define WS2812 and #define SK6812 changed to #define LED2812 #define LED6812 remove WS1812 library and uses Adafruit_NeoPixel for both LED types<br>
Zet_Pixel() aangepast<br>
Changes.: 0.45 Adapted for WS2812 96LEDS Massaranduba clock No7<br>
Changes.: 0.46 Removed DCF77 library and used own DCF77decoding. Receive DCF77 after turning off display of colour LEDs at night <br>
Changes.: 0.47 Stripped down for use with WS2812 92 LEDs clock with Arduino Nano.<br>
Changes.: 0.48 Maintenance<br>
Changes.: 0.49 Changed variable name error: SumMinuteSignal --&gt;SumSecondSignal. Added &quot;A DCF Debug&quot; to menu<br>
Changes.: 0.50 TinyDCF identical with improved code in DCF_HC12TransmitterV28
