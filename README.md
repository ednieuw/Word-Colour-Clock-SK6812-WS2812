# Character Word Colour Clock<br> WS2812/SK6812 for Arduino Nano Every, ATMEGA 328/1284 , MKR1010
This software drives the display with RGB(W) LEDs for a word clock.<br>
When using the RGBW SK6812 LEDs the display of characters can show the whole spectrum of colours<br>
but also pure white. With WS2812 RGB LEDs white has a shadows because it made by the combination of R, G en B.<br> 
The software makes intensive use of defines to select the several options and modules attached.<br>
This makes the program very large. Remove the code between and #ifdef en #endif of parts you do not use.<br>
<br>
The clock can be used for the languages Dutch, French, German and English.<br>
For a four-languages word clock with Version V083 see here:<br> https://ednieuw.home.xs4all.nl/Woordklok/Bouwpakket/4LanguagesWordClock.htm <br>
<br>
The Arduino code version Character_Clock_WS2812_Dec2020 is designed to run with the <br>
ATMEGA328 (Uno, Nano, Mini), ATMEGA1284 chip and only controls the LED strips of type WS2812.<br>
Later versions uses more memory and are suited for SK6812 and WS2812 LED strips.<br>
<br>
The last version is V083. This is free of compiler warnings and the code is optimized.<br>
Copy the libraries from librariesV083.zip in your libraries folder.<br>
With DEFINES in the source code options can be turned On and Off. 
Compile and upload. <br>
Open the serial monitor or the terminal app on your phone when you have installed a Bluetooth module<br> 
![image](https://user-images.githubusercontent.com/12166816/119348087-aee38b00-bc9c-11eb-8c76-687d7cb20d97.png)
<br><br>

When a DCF77 module is attached time is adjusted to the second with a German longwave time signal.<br> 
Version V070 and higher uses two methods to receive the DCF77 time.<br>
One method uses the DCF77 Arduino library that uses interrupts.<br>
The other method samples over 25000 signals a second in the loop and is therefore less prone for spikes in the signal.<br>
But ... the code may not be delayed to much by other processes.<br>
In the source you can find methods keep delays to a minimum.<br>
Combining both methods results in a 50% improvement of the received time.<br>
In the source one of both methods can be selected to be used. <br>
<br>
#define DCFMOD     // Use the Arduino DCF77 library with interrupts. <br> 
#define DCFNOINT   // Use the Tiny DCF algorithm in this program.<br> 
<br>
More here: https://github.com/ednieuw/DCF77_NoInterrupt<br>
The DCF77 signal is disturbed by the colour LEDs.<br>
Turn off the power to the LEDs during the night or place the receiver at least 10 cm away from the LEDs.<br>
Also cheap power supplies, PC's, magnetrons and other high frequency apparatus can disturb the signal.<br>
If the signal falls away or is erratic, turn off all electrical devices and power supplies<br>
and find the culprit.<br>

More about the construction of the word clock see this page in Dutch:<br>
The page is in Dutch but can be translated with Google translate. If needed mail me for a translation.<br>
<br>
https://ednieuw.home.xs4all.nl/Woordklok/Bouwpakket/WoordklokSK6812.htm
<img alt="Two SK6812 clocks" src="TwoSK6812Clocks.jpg" width="450" /><br>

The source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05 and HM10 BLE
- DCF77 module DCF-2
- LCD
- HC12 wireless transceiver
- 4x3 and 3x1 Keypads
- WIFI on MKR 1010 to get NTP time

For the ATMEGA1284 Load and install in IDE:<br>
https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json<br>

The HC05, HM-10 (CC254x) and nRF52 Bluetooth modules can be used  to read and write information or instructions to the clock with a IOS or android mobile phone.<br>
To use it a serial terminal app must be installed on your phone.<br>
With a rotary encoder time and several display modes can also be selected.<br>

<br>Note for Character_Clock_WS2812_Dec2020 for Arduino Uno, Nano with WS2812 or SK6812 LEDs: <br>
Program size with an Arduino UNO or Nano must stay below approx 23572 bytes when 144 LEDs are used. <br>
Due to a Adafruit Neopixel bug the usage of memory by the LEDs is not subtracted from the avaiable memory.<br>
With the Nano Every, Mega or chips with larger memory program size this is (still) no issue.<br>
<br>With an Arduino MKR1010, WIFI can be used to receive the time from a timeserver<br>
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

<img alt="PCB" src="ColourWordClockPCBV15.jpg" width="450" /><br>
PCB layout 

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
Changes.: 0.50 TinyDCF identical with improved code in DCF_HC12TransmitterV28<br>
Changes.: 0.51 Added VERSION. Better 'installed mods' texts<br>
Changes.: 0.52 For wordclock No38 with 144 SK6812 LEDs. HC-12 transceiver module added<br>
Changes.: 0.53 Shortened filename. improved menu. Replaced DCF decoding. Identical with DCF77-NoIntV01.ino <br>
Changes.: 0.54 Added every hour DCFlocked = false;  Identical to DCF77-NoIntV02c<br>
Changes.: 0.55 Stable in WS2812 96 LEDs Massaranduba clock No7. Optimized time decoding from DCF. Identical to DCF77-NoIntV03. <br>
Changes.: 0.56 SignalFaults=0 every hour. MinutesSinceStart > 0X0FFFFFF0.  <br>   
Changes.: 0.57 Changed the DebugInfo line. Changed updating time when TimeOK = 1 and 2. Introduced use of Daylight saving flag BOnehourchange.<br>
Changes.: 0.58 Made for 144 LEDs, rotary, DCF and Bluetooth SK6812<br>
Changes.: 0.59 Added Thhmmss in menu. Removed Secpulse and simplified Brightness calculation. Cleaned coding. <br>
                DCF77 and DCFtiny can be used seperately or together<br>
Changes.: 0.60 Four languages clock. Werkende versie  New name --> Four-language Clock_V001<br>
Changes.: 0.61 Corrected errors in English lay-out. Added colours as text instead of HEX. <br>
                Serialcheck in Demomode() and Selftest(). Changed own color entry to Pwwrrggbb instead of Prrggbb<br>
Changes.: 0.62 Counters for DCF Wrong times reported added. LedsOnOff added. sorage in Mem. EEPROM added <br>
Changes.: 0.63 Removed SetMinuteColour() and CheckColourStatus() and fused them in SetSecondColour(). Optimized DCFtiny code TimeMinutesDiff <2 --> <10 <br>
Changes.: 0.64 SIX->SIXUK Added EEPROM.put in setup  Added || Dday>31in pdateDCFclock<br>
Changes.: 0.65 sizeof(menu) / sizeof(menu[0]). Added PrintLine sub routine<br>
Changes.: 0.66 Minor changes<br>
Changes.: 0.67 Copied ReworkInputString() from FibonacciKlok_V021. Optimised loop() Version not tested !!!!<br>
               Added KEYPADs. Changed rotary processing to combine it with the KEYPADs<br>
Changes.: 0.68 Solved compiler warnings. Changed constrain(mem,0,250) -->= min(mem,250). Added in Reset() EEPROM.put(0,Mem);<br>
Changes.: 0.69 Moved sprintf(sptext,"--> EdT:%ld Ed:%ld Th:%ld EdW:%ld ThW:%ld.... <br>
                case  59:  if(MinOK && HourOK) TimeMinutes = Dhour * 60 + Dminute; else TimeMinutes++;<br>
                if(Dminute>59) MinOK=0; Sometimes MinOK HourOK etc. parity was OK but the time was definitely wrong<br>
                Corrected error BOnehourchange and COnehourchange at BitPos 16<br>
                refined DCFNoInt with: if((millis() - DCFmsTick) >1010) break;<br>
                Removed TimeOK=5<br>
                Dsecond, ..., Dyear --> tmElements_t D time struct<br>
                Isecond, ..., Iyear --> tmElements_t I time struct  V069.17<br>
                Tekstprintln() changed coding<br>
                Added two "One wire keypad" routines<br>
                Added more fault dates control in DCFNoInt <br>
                Made GetSecondsSince2000DCFNoInt() more robust when no signal is received<br>
                Main source code comparable with Four-language_Clock_V004<br>
                Added LIBSK6812 to be used instead of NEOPIXEL<br>
Changes.: 0.70 Added self written SK6812 library EdSoft_SK6812.h<br>
Changes: 0.71 Renamed DCFMOD -> DCFTH <br>
Changes: 0.72 Added HT16K33tijd 4-digit display <br>
               Optimized for Arduino Nano Every<br>
Changes: 0.73 RP2040 #if defined(ARDUINO_ARCH_RP2040). <br>
Changes: 0.74 BluetoothSerial function identical with Fibonacciklok_V036<br>
Changes: 0.75 Added MAXBRIGHTNESS and Mem.UpperBrightness to regulate maximum luminosity. Changed MAXBRIGHTNESS to SLOPEBRIGHTNESS<br>
               Mem.LightReducer is the slope.<br>
Changes: 0.76 Checked and corrected DCF77 functionality  <br>
Changes: 0.77 Version fitted for 4-language clock<br>
Changes: 0.78 Renamed SetSecondColour() in SetMinuteColour()<br>
Changes: V079 Added Mem.Checksum in EEPROM storage. Bug resoved in void ProcessKeyPressTurn --> if (encoderPos == 0) <br>
Changes: V080 Changing source code to be suitable for RP2040.  Added DCF_counts. Added in setup() while (Serial.available())  Serial.read();  <br>                                                // Flush the serial input buffer. When a Bluetooth module is connected to RX/TX pin 0&1 garbage can set the time to 00:00:00<br>
Changes: V081 Added nRF52 BLE protocol. <br>
               Added several ARCHitectures   #if defined(__AVR_ATmega328P__) || defined(ARDUINO_ARCH_RP2040) || defined (ARDUINO_AVR_NANO_EVERY)|| defined<br> (ARDUINO_AVR_ATmega4809) || defined(ARDUINO_ARCH_SAMD)<br>
               Added WIFIMOD and remove MKR1010 defines<br>
Changes: V082  Bugs in menu corrected. Deze versie compileert niet meer goed.<br>  
Changes: V083 Used for 4-language clock No1 with Nano Every and compiled with MegaCore X, ATMega4809, JTAG2UDPI<br>
              Updated menu. Added toggle On/Off LEDs <br>
