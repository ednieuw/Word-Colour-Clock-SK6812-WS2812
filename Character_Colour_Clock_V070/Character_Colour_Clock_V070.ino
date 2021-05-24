// =============================================================================================================================
/* 
This Arduino code controls the Nano Every, ATMEGA1284 chip and Arduino MKR1010 that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05 and HM10 BLE
- DCF77 module DCF-2
- LCD
- WIFI on MKR 1010 to get NTP time
- HC12 transceiver

Load and install in IDE:
http://arduino.esp8266.com/stable/package_esp8266com_index.json  (ESP-12)
https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json ATMEGA1260 and 644
https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json ATTINY
Arduino SAMD for MKR1010

The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module.
The DCFNOINT subroutine is  a non-interrupt driven DCF77 decoding routine. Using both together improves decoding efficiency

Arduino Uno, Nano with SK6812 LEDs: Program size must stay below approx 23572, 1129 bytes bytes with 144 LEDs in ATMEGA 32KB 
With a Arduino Nano Every with 64kb memory this is not a problem
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
 Changes.: 0.24 Built in Clock Michelle en JoDi klok. 
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
                Zet_Pixel() aangepast
 Changes.: 0.45 Adapted for WS2812 96LEDS Massaranduba clock No7
 Changes.: 0.46 Removed DCF77 library and used own DCF77decoding. Receive DCF77 after turning off display of colour LEDs at night  
 Changes.: 0.47 Stripped down for use with WS2812 92 LEDs clock with Arduino Nano.
 Changes.: 0.48 Maintenance
 Changes.: 0.49 Changed variable name error: SumMinuteSignal -->SumSecondSignal. Added "A DCF Debug" to menu
 Changes.: 0.50 TinyDCF identical with improved code in DCF_HC12TransmitterV28 
 Changes.: 0.51 Added VERSION. Better 'installed mods' texts
 Changes.: 0.52 For wordclock No38 with 144 SK6812 LEDs. HC-12 transceiver module added
 Changes.: 0.53 Shortened filename. improved menu. Replaced DCF decoding. Identical with DCF77-NoIntV01.ino 
 Changes.: 0.54 Changed  if(DCF_signal>60 && abs((((Dyear*1  in if(DCFlocked && abs((((Dyear*12+Dmonth)*. Added every hour DCFlocked = false;
                Identical to DCF77-NoIntV02c
 Changes.: 0.55 Stable in WS2812 96 LEDs Massaranduba clock No7 with Nano Every. Optimized time decoding from DCF. Identical to DCF77-NoIntV03. 
 Changes.: 0.56 SignalFaults=0 every hour. MinutesSinceStart > 0X0FFFFFF0 
 Changes.: 0.57 Changed the DebugInfo line. Changed updating time when TimeOK = 1 and 2. Introduced use of Daylight saving flag COnehourchange.
 Changes.: 0.58 Made for 144 LEDs, rotary, DCF and Bluetooth SK6812
 Changes.: 0.59 Added Thhmmss in menu. Removed Secpulse and simplified Brightness calculation. Cleaned coding. 
                DCF77 and DCFNOINT can be used seperately or together
 Changes.: 0.60 Four languages clock. Werkende versie  New name --> Four-language Clock_V001
 Changes.: 0.61 Corrected errors in English lay-out. Added colours as text instead of HEX. 
                Serialcheck in Demomode() and Selftest(). Changed own color entry to Pwwrrggbb instead of Prrggbb
 Changes.: 0.62 Counters for DCF Wrong times reported added. LedsOnOff added. sorage in Mem. EEPROM added 
 Changes.: 0.63 Removed SetMinuteColour() and CheckColourStatus() and fused them in SetSecondColour(). Optimized DCFNOINT code TimeMinutesDiff <2 --> <10 
 Changes.: 0.64 SIX->SIXUK Added EEPROM.put in setup  Added || Dday>31in pdateDCFclock
 Changes.: 0.65 sizeof(menu) / sizeof(menu[0]). Added PrintLine sub routine
 Changes.: 0.66 Minor changes
 Changes.: 0.67 Copied ReworkInputString() from FibonacciKlok_V021. Optimised loop()  **** Version not tested !!!!
                Added KEYPADs. Changed rotary processing to combine it with the KEYPADs
 Changes.: 0.68 Solved compiler warnings. Changed constrain(mem,0,250) -->= min(mem,250). Added in Reset() EEPROM.put(0,Mem); **** Version not tested !!!!
 Changes.: 0.69 Moved sprintf(sptext,"--> EdT:%ld Ed:%ld Th:%ld EdW:%ld ThW:%ld.... It was printed many times when reception of DCF77 is erratic
                case  59:  if(MinOK && HourOK) TimeMinutes = Dhour*60 + Dminute; else TimeMinutes++;
                if(Dminute>59) MinOK=0; Sometimes MinOK HourOK etc. parity was OK but the time was definitely wrong
                Corrected error BOnehourchange and COnehourchange at BitPos 16
                refined DCFNoInt with: if((millis() - DCFmsTick) >1010) break;
                Removed TimeOK=5
                Dsecond, ..., Dyear --> tmElements_t D time struct
                Isecond, ..., Iyear --> tmElements_t I time struct  V069.17
                Tekstprintln() changed coding
                Added two "One wire keypad" routines
                Added more fault dates control in DCFNoInt 
                Made GetSecondsSince2000DCFNoInt() more robust when no signal is received
                Main source code comparable with Four-language_Clock_V004
                Added LIBSK6812 to be used instead of NEOPIXEL
Changes.: 0.70 Added EdSoft_SK6812.h
todo  Rename EdSoft_SK6812 --> EdPix
      Make function calls identical to NeoPix


 */
// ===============================================================================================================================
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Definition of installed modules
//------------------------------------------------------------------------------
// ------------------>   Define only one type of LED strip 
//#define LED2812
#define LED6812                                    // Choose between LED type RGB=LED2812 or RGBW=LED6812
// ------------------>   Define only one library
#define NEOPIXEL                                 // Adafruit Neopixel
//#define FABLED                                   // FABLED alleen UNO en Nano
//#define LIBSK6812                                  // SK6812 library. Only with SK6812 LEDs

// ------------------>  Define which module is present.  
#define BLUETOOTHMOD                               // Use  this define if Bluetooth needs other pins than pin 0 and pin 1
//#define HC12MOD                                  // Use HC12 time transreceiver Long Range Wireless Communication Module in Bluetooth slot
#define DCF77MOD                                   // DCF77 receiver installed
#define MOD_DS3231                                 // The DS3231 module is installed, if not the Arduino internal clock is used
//#define LCDMOD                                   // For LCD support

#define ROTARYMOD                                  // Rotary encoder installed
//#define KEYPAD3x4                                // Use a 3x4 keypad with 7 wires
//#define KEYPAD3x1                                // Use a 3x1 keypad with 4 wires   
//#define ONEWIREKEYPAD3x1                         // Use a 3x1 keypad with one wire   
//#define ONEWIREKEYPAD3x4                         // Use a 3x4 keypad with one wire

char VERSION[] = "Character_Colour_Clock_V070";
//------------------------------------------------------------------------------
// ARDUINO Definition of installed language word clock
//------------------------------------------------------------------------------
 #define NL
// #define UK
// #define DE
// #define FR        
//------------------------------------------------------------------------------
// ARDUINO Includes defines and initialisations
//------------------------------------------------------------------------------
//                                                                                            //
                     #if defined(ROTARYMOD) || defined(ONEWIREKEYPAD3x1) || defined(ONEWIREKEYPAD3x4)
#define ROTKEYMOD                                  // not Implemented yet ************ Make this #ifdef easier to read
                     #endif
                     #ifdef DCF77MOD
#define DCFMOD                                     // Use the Arduino DCF77 library with interrupts.  
#define DCFNOINT                                   // Use the Tiny DCF algorithm in this program. 
                     #endif
                     #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
#define ATmega644_1284                             // Make this #ifdef easier to read
                     #endif                     
#include <TimeLib.h>                               // https://github.com/PaulStoffregen/Time 
#include <Wire.h>                                  // Arduino standard library
#include "RTClib.h"                                // https://github.com/adafruit/RTClib 

//#include <avr/wdt.h>                             // for reset function
#include <avr/pgmspace.h>                          // Arduino
                     #ifdef ARDUINO_SAMD_MKRWIFI1010
#include <FlashAsEEPROM.h>                         // url=https://github.com/cmaglie/FlashStorage
#include <SPI.h>                                   // for web server
#include <WiFiNINA.h>                              // url=http://www.arduino.cc/en/Reference/WiFiNINA 
//#include "arduino_secrets.h"                     // for web server
#define SECRET_SSID "PWDis_klok"
#define SECRET_PASS "klok"
                     #else
#include <EEPROM.h>
                     #endif
                     #ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel   for LED strip WS2812 or SK6812
                     #endif  // NEOPIXEL
                     #ifdef LIBSK6812
//#include <EdSoft_SK6812.h>             //experimental
#include <SK6812.h>            //   https://github.com/Electry/Arduino_SK6812  
                     #endif // LIBSK6812
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>                     // https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
                     #endif  //LCDMOD                      // https://github.com/adafruit/Adafruit_NeoPixel 
                     #if defined(KEYPAD3x4) || defined(KEYPAD3x1)
#include "Adafruit_Keypad.h"                       // https://github.com/adafruit/Adafruit_Keypad//                 
                     #endif //KEYPAD 
                     #ifdef HC12MOD
#include <SoftwareSerial.h>        // For HC12 transceiver
                     #endif  //HC12MOD 
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>                        // Arduino for Bluetooth communication
                     #endif //BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>                               // http://www.pjrc.com/teensy/td_libs_Encoder.html 
                     #ifdef ATmega644_1284
  #define CORE_NUM_INTERRUPT 3                     // use other interupt voor these chipsetsa
  #define CORE_INT0_PIN   10
  #define CORE_INT1_PIN   11
  #define CORE_INT2_PIN   2
                     #endif
                     #endif //ROTARYMOD
                     #ifdef DCFMOD
#include "DCF77.h"                                 // http://playground.arduino.cc/Code/DCF77 

//                                                                                            //
                     #endif  //DCFMOD
                     #ifdef NL
#define HET     ColorLeds("Het",     0,   2, MINColor);   
#define IS      ColorLeds("is",      4,   5, SECColor);    Is = true; 
#define WAS     ColorLeds("was",     8,  10, SECColor);    Is = false;
#define PRECIES ColorLeds("precies", 17, 23, LetterColor);
#define MTIEN   ColorLeds("tien",    12, 15, LetterColor); 
#define MVIJF   ColorLeds("vijf",    25, 28, LetterColor); 
#define KWART   ColorLeds("kwart",   30, 34, LetterColor);
#define VOOR    ColorLeds("voor",    44, 47, LetterColor);
#define OVER    ColorLeds("over",    38, 41, LetterColor);
#define HALF    ColorLeds("half",    48, 51, LetterColor);
#define MIDDER  ColorLeds("midder",  53, 58, LetterColor);
#define VIJF    ColorLeds("vijf",    66, 69, LetterColor);
#define TWEE    ColorLeds("twee",    60, 63, LetterColor);
#define EEN     ColorLeds("een",     72, 74, LetterColor);
#define VIER    ColorLeds("vier",    77, 80, LetterColor);
#define TIEN    ColorLeds("tien",    91, 94, LetterColor);
#define TWAALF  ColorLeds("twaalf",  84, 89, LetterColor);
#define DRIE    ColorLeds("drie",    96, 99, LetterColor);
#define NEGEN   ColorLeds("negen",  102,106, LetterColor);
#define ACHT    ColorLeds("acht",   114,117, LetterColor);
#define NACHT   ColorLeds("nacht",  114,118, LetterColor);
#define ZES     ColorLeds("zes",    110,112, LetterColor);
#define ZEVEN   ColorLeds("zeven",  121,125, LetterColor);
#define ELF     ColorLeds("elf",    128,130, LetterColor);
#define NOEN    ColorLeds("noen",   139,142, LetterColor);
#define UUR     ColorLeds("uur",    134,136, LetterColor);
#define EDSOFT  ColorLeds("EdSoft", 132,132, LetterColor);
#define X_OFF   ColorLeds("",         0,  2, 0);
#define X_ON    ColorLeds("",         0,  2, LetterColor);
                     #endif //NL
                     #ifdef UK
#define IT      ColorLeds("\nIt",   145, 145, MINColor);   
#define ISUK    ColorLeds("is",     145, 145, SECColor);    Is = true;
#define WASUK   ColorLeds("was",    145, 145, SECColor);    Is = false;
#define EXACTUK ColorLeds("exact",  145, 145, LetterColor);
#define HALFUK  ColorLeds("half",   145, 145, LetterColor); 
#define TWENTY  ColorLeds("twenty", 145, 145, LetterColor); 
#define MFIVE   ColorLeds("five",   145, 145, LetterColor);
#define QUARTER ColorLeds("quarter",145, 145, LetterColor);
#define MTEN    ColorLeds("ten",    145, 145, LetterColor);
#define PAST    ColorLeds("past",   145, 145, LetterColor);
#define TO      ColorLeds("to",     145, 145, LetterColor);
#define SIXUK   ColorLeds("six",    145, 145, LetterColor);
#define TWO     ColorLeds("two",    145, 145, LetterColor);
#define FIVE    ColorLeds("five",   145, 145, LetterColor);
#define TWELVE  ColorLeds("twelve", 145, 145, LetterColor);
#define TEN     ColorLeds("ten",    145, 145, LetterColor);
#define ELEVEN  ColorLeds("eleven", 145, 145, LetterColor);
#define FOUR    ColorLeds("four",   145, 145, LetterColor);
#define NINE    ColorLeds("nine",   145, 145, LetterColor);
#define THREE   ColorLeds("three",  145, 145, LetterColor);
#define EIGHT   ColorLeds("eight",  145, 145, LetterColor);
#define ONE     ColorLeds("one",    145, 145, LetterColor);
#define SEVEN   ColorLeds("seven",  145, 145, LetterColor);
#define OCLOCK  ColorLeds("O'clock",145, 145, LetterColor);
                      #endif //UK
                      #ifdef DE
#define ES      ColorLeds("\nEs",   145, 145, MINColor);   
#define IST     ColorLeds("ist",    145, 145, SECColor);    Is = true;
#define WAR     ColorLeds("war",    145, 145, SECColor);    Is = false;
#define GENAU   ColorLeds("genau",  145, 145, LetterColor);
#define MZEHN   ColorLeds("zehn",   145, 145, LetterColor);
#define MFUNF   ColorLeds("funf",   145, 145, LetterColor);
#define VIERTEL ColorLeds("viertel",145, 145, LetterColor);
#define ZWANZIG ColorLeds("zwanzig",145, 145, LetterColor);
#define KURZ    ColorLeds("kurz",   145, 145, LetterColor);
#define VOR     ColorLeds("vor",    145, 145, LetterColor);
#define NACH    ColorLeds("nach",   145, 145, LetterColor);
#define HALB    ColorLeds("halb",   145, 145, LetterColor);
#define FUNF    ColorLeds("funf",   145, 145, LetterColor);
#define EINS    ColorLeds("eins",   145, 145, LetterColor);
#define VIERDE  ColorLeds("vier",   145, 145, LetterColor);
#define ZEHN    ColorLeds("zehn",   145, 145, LetterColor);
#define ZWOLF   ColorLeds("zwolf",  145, 145, LetterColor);
#define DREI    ColorLeds("drei",   145, 145, LetterColor);
#define NEUN    ColorLeds("neun",   145, 145, LetterColor);
#define ACHTDE  ColorLeds("acht",   145, 145, LetterColor);
#define SECHS   ColorLeds("sechs",  145, 145, LetterColor);
#define SIEBEN  ColorLeds("sieben", 145, 145, LetterColor);
#define ZWEI    ColorLeds("zwei",   145, 145, LetterColor);
#define ELFDE   ColorLeds("elf",    145, 145, LetterColor);
#define UHR     ColorLeds("uhr",    145, 145, LetterColor);
                      #endif //DE
                      #ifdef FR
#define IL      ColorLeds("\nIl",   145, 145, MINColor);   
#define EST     ColorLeds("est",    145, 145, SECColor);    Is = true;
#define ETAIT   ColorLeds("etait",  145, 145, SECColor);    Is = false;
#define EXACT   ColorLeds("exact",  145, 145, LetterColor);
#define SIX     ColorLeds("six",    145, 145, LetterColor); 
#define DEUX    ColorLeds("deux",   145, 145, LetterColor); 
#define TROIS   ColorLeds("trois",  145, 145, LetterColor);
#define ONZE    ColorLeds("onze",   145, 145, LetterColor);
#define QUATRE  ColorLeds("quatre", 145, 145, LetterColor);
#define MINUIT  ColorLeds("minuit", 145, 145, LetterColor);
#define DIX     ColorLeds("dix",    145, 145, LetterColor);
#define CINQ    ColorLeds("cinq",   145, 145, LetterColor);
#define NEUF    ColorLeds("neuf",   145, 145, LetterColor);
#define MIDI    ColorLeds("midi",   145, 145, LetterColor);
#define HUIT    ColorLeds("huit",   145, 145, LetterColor);
#define SEPT    ColorLeds("sept",   145, 145, LetterColor);
#define UNE     ColorLeds("une",    145, 145, LetterColor);
#define HEURE   ColorLeds("heure",  145, 145, LetterColor);
#define HEURES  ColorLeds("heures", 145, 145, LetterColor);
#define ET      ColorLeds("et",     145, 145, LetterColor);
#define MOINS   ColorLeds("moins",  145, 145, LetterColor);
#define LE      ColorLeds("le",     145, 145, LetterColor);
#define DEMI    ColorLeds("demie",  145, 145, LetterColor);
#define QUART   ColorLeds("quart",  145, 145, LetterColor);
#define MDIX    ColorLeds("dix",    145, 145, LetterColor);
#define VINGT   ColorLeds("vingt",  145, 145, LetterColor);
#define MCINQ   ColorLeds("cinq",   145, 145, LetterColor);
#define DITLEHEURE DitLeHeure();
                      #endif //FR
//                                                                                            //
//------------------------------------------------------------------------------
// PIN Assigments
//------------------------------------------------------------------------------ 
//             #if defined(__AVR_ATmega328P__) || defined(ARDUINO_SAMD_MKRWIFI1010) || defined (ARDUINO_AVR_NANO_EVERY)|| defined (ARDUINO_AVR_ATmega4809)
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----
 RX           = 0,                // Connects to Bluetooth TX
 TX           = 1,                // Connects to Bluetooth RX
 DCF_PIN      = 2,                // DCFPulse on interrupt  pin
 encoderPinA  = 3,                // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 4,                // switch (labeled SW on decoder)
 LED_PIN      = 5,                // Pin to control colour SK6812/WS2812 LEDs
 BT_TX        = 6,                // Connects to Bluetooth RX
 BT_RX        = 7,                // Connects to Bluetooth TX
 HC_12TX      = 6,                // HC-12 TX Pin  // note RX and TX are reversed compared with a BT Module
 HC_12RX      = 7,                // HC-12 RX Pin 
 encoderPinB  = 8,                // left (labeled CLK on decoder)no interrupt pin  
 secondsPin   = 9,
 HeartbeatLED = 10,               // PIN10        = 10,                        // PIN 10 
 DCFgood      = 11,               // DCF-signal > 50    //PIN11        = 11,   // PIN 11
 PIN12        = 12,               // PIN 12             // led = 12,          // MKR1010
 DCF_LED_Pin  = 13,                // DCF signal
 };
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 EmptyA0      = 0,                // Empty
 EmptyA1      = 1,                // Empty
 PhotoCellPin = 2,                // LDR pin
 OneWirePin   = 3,                // OneWirePin
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5,                // SCL pin
 EmptyA6     =  6,                // Empty
 EmptyA7     =  7};               // Empty
//                  #endif

                             #ifdef ATmega644_1284  
//------------------------------------------------------------------------------//------------------------------------------------------------------------------
/*                     +---\/---+
           (D 0) PB0 1|        |40 PA0 (AI 0 / D24)
           (D 1) PB1 2|        |39 PA1 (AI 1 / D25)
      INT2 (D 2) PB2 3|        |38 PA2 (AI 2 / D26)
       PWM (D 3) PB3 4|        |37 PA3 (AI 3 / D27)
    PWM/SS (D 4) PB4 5|        |36 PA4 (AI 4 / D28)
      MOSI (D 5) PB5 6|        |35 PA5 (AI 5 / D29)
  PWM/MISO (D 6) PB6 7|        |34 PA6 (AI 6 / D30)
   PWM/SCK (D 7) PB7 8|        |33 PA7 (AI 7 / D31)
                 RST 9|        |32 AREF
                VCC 10|        |31 GND
                GND 11|        |30 AVCC
              XTAL2 12|        |29 PC7 (D 23)
              XTAL1 13|        |28 PC6 (D 22)
      RX0 (D 8) PD0 14|        |27 PC5 (D 21) TDI
      TX0 (D 9) PD1 15|        |26 PC4 (D 20) TDO
RX1/INT0 (D 10) PD2 16|        |25 PC3 (D 19) TMS
TX1/INT1 (D 11) PD3 17|        |24 PC2 (D 18) TCK
     PWM (D 12) PD4 18|        |23 PC1 (D 17) SDA
     PWM (D 13) PD5 19|        |22 PC0 (D 16) SCL
     PWM (D 14) PD6 20|        |21 PD7 (D 15) PWM
                      +--------+*/         
//                                                                                            //
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 1284P ----
 BT_RX        = 0,                // Bluetooth RX Connects to Bluetooth TX
 BT_TX        = 1,                // Bluetooth TX Connects to Bluetooth RX
 DCF_PIN      = 2,                // DCFPulse on interrupt pin
 LED_PIN      = 3,                // Pin to control colour 2811/2812 leds PB3 digital
 PIN04        = 4,                // Pin 4         PB4 PWM
 secondsPin   = 5,                // Seconden
 PIN06        = 6,                // PIN 6         PB6 PWM  
 HeartbeatLED = 7,                // PIN 7         PB7 PWM 
 RX1          = 8,                // RX1           PD0 digital
 TX1          = 9,                // TX1           PD1 digital
 DCF_LED_Pin  = 10,               // LED10         PD2 digital
 encoderPinB  = 11,               // left (labeled CLK on decoder)no interrupt pin
 encoderPinA  = 12,               // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 13,               // switch (labeled SW on decoder)  
 LED14        = 14,               //               PD6 digital
 LED15        = 15,               // LED15         PD7 PWM
 SCL_pin      = 16,               // SCL pin       PC0 interrupt
 SDA_pin      = 17,               // SDA pin       PC1 interrupt
 DCF_TX       = 19,               // TX2         4800,SERIAL_7E2);  //connect TX to D18 and RX to D19 PC2 digital
 DCF_RX       = 18,               // RX2 LED19     PC3 digital
 LEDDataPin   = 20,               // blauw HC595
 LEDStrobePin = 21,               // groen HC595
 LEDClockPin  = 22,               // geel  HC595
 PIN23        = 23 
 };
                                  
enum AnaloguePinAssignments {     // Analogue hardware constants ----
  EmptyA0     = 24,               // Empty
  EmptyA1     = 25,               // Empty
  PhotoCellPin= 26,               // LDR pin  AI 2
  EmptyA3     = 27,               // Empty
  EmptyA4     = 28,               // Empty
  EmptyA5     = 29,               // Empty
  EmptyA6     = 30};              // Empty
                     #endif //ATmega644_1284

//------------------------------------------------------------------------------
// COLOURS
//------------------------------------------------------------------------------   
const byte DEFAULTCOLOUR = 0;
const byte HOURLYCOLOUR  = 1;          
const byte WHITECOLOR    = 2;
const byte OWNCOLOUR     = 3;
const byte OWNHETISCLR   = 4;
const byte WHEELCOLOR    = 5;
const byte DIGITAL       = 6;
byte ResetDisplayChoice  = DEFAULTCOLOUR; 
                         #ifdef LED2812
const uint32_t white  = 0xFFFFFF, lgray  = 0x666666;               // R, G and B on together gives white light
const uint32_t gray   = 0x333333, dgray  = 0x222222;                
                         #endif  //LED2812
                         #ifdef LED6812    
const uint32_t white  = 0xFF000000, lgray  = 0x66000000;           // The SK6812 LED has a white LED that is pure white
const uint32_t dgray  = 0x22000000, gray   = 0x33000000;
                         #endif  //LED6812  
//------------------------------------------------------------------------------
const uint32_t black    = 0x000000, darkorange    = 0xFF8C00, red    = 0xFF0000, chartreuse = 0x7FFF00;
const uint32_t brown    = 0x503000, cyberyellow   = 0xFFD300, orange = 0xFF8000; 
const uint32_t yellow   = 0xFFFF00, cadmiumyellow = 0xFFF600, chromeyellow = 0xFFA700;
const uint32_t green    = 0x00FF00, brightgreen   = 0x66FF00, apple  = 0x80FF00, grass  = 0x00FF80;  
const uint32_t amaranth = 0xE52B50, edamaranth    = 0xFF0050, amber  = 0xFF7E00;
const uint32_t marine   = 0x0080FF, darkviolet    = 0x800080, pink   = 0xFF0080, purple = 0xFF00FF; 
const uint32_t blue     = 0x0000FF, cerulean      = 0x007BA7, sky    = 0x00FFFF, capri  = 0x00BFFF;
const uint32_t edviolet = 0X7500BC, frenchviolet  = 0X8806CE, coquelicot = 0xFF3800;
const uint32_t greenblue= 0x00F2A0, hotmagenta    = 0xFF00BF, dodgerblue = 0x0073FF, screamingreen= 0x70FF70;
//------------------------------------------------------------------------------
// LED
//------------------------------------------------------------------------------
//                                                                                            //
const byte NUM_LEDS      = 144;                                    // How many leds in  strip?
const byte MATRIX_WIDTH  = 12;
const byte MATRIX_HEIGHT = 12;
const byte BRIGHTNESS    = 32;                                     // BRIGHTNESS 0 - 255

                     #ifdef LED6812    
                             #ifdef NEOPIXEL   
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                             #endif //NEOPIXEL
                                              #ifdef LIBSK6812
EdSoft_SK6812 LED(NUM_LEDS, LED_PIN);   
//RGBW   Strippixel[NUM_LEDS+1] ;//= {0};                                   // Array holding the original pixel data
//RGBW   StrippixelDimmed[NUM_LEDS+1];// = {0};                             // Array holding the pixel send to the pixel after dimming 
                                           // Initialyse SK6812 library
                                             #endif  //LIBSK6812  
                      #endif  //LED6812  
                                            #ifdef LED2812
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);  //NEO_RGB NEO_GRB
                                            #endif  //LED2812
bool     LEDsAreOff            = false;                             // If true LEDs are off except time display
bool     NoTextInColorLeds     = false;                             // Flag to control printing of the text in function ColorLeds()
byte     BrightnessCalcFromLDR = BRIGHTNESS;                        // Initial brightness value The intensity send to the LEDs (0-255)
int      Previous_LDR_read     = 512;                               // The actual reading from the LDR + 4x this value /5
uint32_t MINColor      = chromeyellow;
uint32_t SECColor      = chromeyellow;
uint32_t LetterColor   = chromeyellow;       
uint32_t DefaultColor  = chromeyellow;      
uint32_t OwnColour     = 0X002345DD;        // Blueish
uint32_t WhiteColour   = white;
uint32_t WheelColor    = blue;
uint32_t HourColor[] ={  white,      darkviolet, cyberyellow, capri,         amber,         apple,
                         darkorange, cerulean,   edviolet,    cadmiumyellow, green,         edamaranth,
                         red,        yellow,     coquelicot,  pink,          apple,         hotmagenta,
                         green,      greenblue,  brightgreen, dodgerblue,    screamingreen, blue,
                         white,      darkviolet, cyberyellow};     

// uint32_t HourColor[24] ={white   ,0X800080,0XFFD500,0X00B8FF,0XFF6200,0X80FF00,
//                          0XFF8000,0X00C8FF,0X7500BC,0XFFD000,0X00FF00,0XFF0050,
//                          0XFF0000,0XFFFF00,0XFF4000,0XFF0088,0XF7FF00,0XFF00BF,
//                          0X00FF00,0X00F2A0,0X6FFF00,0X0073FF,0X70ff70,0X0000FF };     

// Definition of the digits 0 - 9, 3 wide, 5 high. 
const byte PROGMEM Getal[10][3][5]  = { 
                     { {1, 1, 1, 1, 1}, {1, 0, 0, 0, 1}, {1, 1, 1, 1, 1} },  //0
                     { {1, 0, 0, 0, 1}, {1, 1, 1, 1, 1}, {0, 0, 0, 0, 1} },  //1
                     { {1, 0, 1, 1, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 0, 1} },  //2
                     { {1, 0, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} },  //3
                     { {1, 1, 1, 0, 0}, {0, 0, 1, 0, 0}, {1, 1, 1, 1, 1} },  //4
                     { {1, 1, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 0, 1, 1, 1} },  //5
//                   { {1, 1, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 0, 0, 1, 1} },  //5 An other 5
                     { {1, 1, 1, 1, 1}, {0, 0, 1, 0, 1}, {0, 0, 1, 1, 1} },  //6
                     { {1, 1, 0, 0, 0}, {1, 0, 0, 0, 0}, {1, 1, 1, 1, 1} },  //7
                     { {1, 1, 1, 1, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} },  //8
                     { {1, 1, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} }   //9
                     }; 
     
//------------------------------------------------------------------------------
// KY-040 ROTARY
//------------------------------------------------------------------------------ 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);                         // Use digital pin  for encoder
                          #endif  //ROTARYMOD      
long     Looptime          = 0;
byte     RotaryPress       = 0;                                  // Keeps track display choice and how often the rotary is pressed.
uint32_t RotaryPressTimer  = 0;
byte     NoofRotaryPressed = 0;

//------------------------------------------------------------------------------
// LDR PHOTOCELL
//------------------------------------------------------------------------------
//                                                                                            //
byte     TestLDR            = 0;                                  // If true LDR inf0 is printed every second in serial monitor
byte     NVRAMmem[24];   
const    byte MAXBRIGHTNESS = 80;                                 // Factor (%) to dim LED intensity with. Between 1% and 250%
const    byte LOWBRIGHTNESS = 5;                                  // Lower limit of Brightness ( 0 - 255)   
int      OutPhotocell;                                            // stores reading of photocell;
int      MinPhotocell       = 999;                                // stores minimum reading of photocell;
int      MaxPhotocell       = 1;                                  // stores maximum reading of photocell;
uint32_t SumLDRreadshour  = 0;
uint32_t NoofLDRreadshour = 0;

//------------------------------------------------------------------------------
// CLOCK
//------------------------------------------------------------------------------                                 
static  uint32_t msTick;                                          // the number of millisecond ticks since we last incremented the second counter
int     count; 
int     Delaytime = 200;
//byte    Isecond, Iminute, Ihour, Iday, Imonth, Iyear, I.Wday; 
byte    lastday = 0, lastminute = 0, lasthour = 0, sayhour = 0;
bool    ChangeTime           = false;
bool    ChangeLightIntensity = false;
bool    Demo                 = false;
bool    Zelftest             = false;
bool    Is                   = true;                              // Toggle of displaying Is or Was
bool    ZegUur               = true;                              // Say or not say Uur in NL clock
tmElements_t I;                                                   // Initialyse a time struct I for internal times used to display the time in the clock
//------------------------------------------------------------------------------
// DS3231 CLOCK MODULE
//------------------------------------------------------------------------------
//                                                                                            //
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
        #ifdef MOD_DS3231
RTC_DS3231 RTCklok;    //RTC_DS1307 RTC; 
        #else
RTC_Millis RTCklok;   
        #endif  //
DateTime Inow;

//------------------------------------------------------------------------------
// BLUETOOTH
//------------------------------------------------------------------------------                                     
                           #ifdef BLUETOOTHMOD                    // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);                           // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
String  BluetoothString;
                           #endif  //BLUETOOTHMOD 
//------------------------------------------------------------------------------
// HC-12 Long Range Wireless Communication Module
//------------------------------------------------------------------------------
                     #ifdef HC12MOD
SoftwareSerial HC12(HC_12TX, HC_12RX);                            // HC-12 TX Pin, HC-12 RX Pin
                     #endif  //HC12MOD 
                     #ifdef DCF77MOD                                              
//------------------------------------------------------------------------------
// DCF-2 DCF77 MODULE
//------------------------------------------------------------------------------
//                                                                                            //
byte   DCF_signal            = 40;                                 // is a proper time received?

bool   DCFEd                 = false;                              // Flag to detect if DCFNOINT library received a good time
bool   DCFTh                 = false;                              // Flag to detect if DCF77 library received a good time
bool   DCFlocked             = false;                              // Time received from DCF77.
                   #endif // DCF77MOD
                    #ifdef DCFMOD 
                    #if defined ATmega644_1284 ||  defined ARDUINO_AVR_NANO_EVERY || defined ARDUINO_AVR_ATmega4809
#define DCF_INTERRUPT 2                                           // DCF Interrupt number associated with DCF_PIN
                    #else
#define DCF_INTERRUPT 0                                           // Check for other boards !!! this works for Uno, nano etc
                    #endif
time_t   TimeDCF77Seconds   = 0;                                  // Time since 1-1-2000 from DCF
time_t   LastDCF77Seconds   = 0;                                  // Time since 1-1-2000 from DCF
const byte DCF_INVERTED     = LOW;                                // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
DCF77 DCF = DCF77(DCF_PIN, DCF_INTERRUPT,DCF_INVERTED);
                    #endif  //DCFMOD                   
                    #ifdef DCFNOINT
//------------------------------------------------------------------------------
// DCF77 DCFNOINT MODULE
//------------------------------------------------------------------------------
//                                                                                            //

byte     StartOfEncodedTime = 0;                                  // Should be always one. Start if time signal
int      CSummertime        = 0;                                  // Counter. If >30 times CSummertime is received Summertime = true
int      CWintertime        = 0;                                  // Counter. If >30 times CWintertime is received Wintertime = true
int      COnehourchange     = 0;                                  // Counter. If >10 times COnehourchange is received Onehourchange = true
uint32_t DCFmsTick          = 0;                                  // the number of millisecond ticks since we last incremented the second counter
uint32_t SumSecondSignal    = 0;                                  // sum of digital signals ( 0 or 1) in 1 second
uint32_t SumSignalCounts    = 0;                                  // Noof of counted signals
uint32_t SignalFaults       = 0;                                  // Counter for SignalFaults per hour  
time_t   TimeDCFNoIntSecs   = 0;                                  // Time since 1-1-2000 from DCF
time_t   LastDCFNoIntSecs   = 0;                                  // Time since 1-1-2000 from DCF
//uint32_t SignalBin[20];                                         // For debugging to see in which time interval reception takes place
tmElements_t D;                                                   // Initialyse a time struct for times used in DCFNoInt
                    #endif  //DCFNOINT                
                    #ifdef LCDMOD
//------------------------------------------------------------------------------
// LCD Module
//------------------------------------------------------------------------------
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);// 0x27 is the I2C bus address for an unmodified backpack
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7);
                    #endif 
//------------------------------------------------------------------------------
// Webserver
//------------------------------------------------------------------------------
//                                                                                            //
                     #ifdef ARDUINO_SAMD_MKRWIFI1010
char ssid[]  = SECRET_SSID;                                        // your network SSID (name)
char pass[]  = SECRET_PASS;                                        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                                                  // your network key Index number (needed only for WEP)
//int led    = LED_BUILTIN;
int status   = WL_IDLE_STATUS;
WiFiServer     server(80);
                     #endif
                     #ifdef KEYPAD3x1
//------------------------------------------------------------------------------
// KEYPAD 3x1
//          -------- GND
//  R Y G   -------- Pin 2
//          -------- Pin 3
//          -------- Pin 4
//------------------------------------------------------------------------------
String        KeypadString;
unsigned long KeyLooptime;
const byte ROWS = 3;                                                // No of rows
const byte COLS = 1;                                                // No of columns
char Keys[ROWS][COLS] = {  {'G'},  {'Y'},  {'R'} };                 // Define the symbols on the buttons of the keypads
byte rowPins[ROWS] = {2,3,4};                                       // Connect to the row pinouts of the keypad
byte colPins[COLS] = {15};                                          // Connect to a nonsense pin
Adafruit_Keypad Keypad3x1 = Adafruit_Keypad(makeKeymap(Keys), rowPins, colPins, ROWS, COLS); 
                     #endif //KEYPAD3x1                       
                     #ifdef KEYPAD3x4
//------------------------------------------------------------------------------
// KEYPAD3*4
//------------------------------------------------------------------------------
String        KeypadString;
unsigned long KeyLooptime;
const byte    ROWS = 4;                                              // No of rows 
const byte    COLS = 3;                                              // No of columns 
byte          rowPins[ROWS] = {12,11,10,9};                          // Connect to the row pinouts of the keypad 
byte          colPins[COLS] = {8,7,6};                               // Connect to the column pinouts of the keypad
                                                                     // Wire under * to pin 6. Wire under # to pin 12
char   hexaKeys[ROWS][COLS] = {                                      // Define the symbols on the buttons of the keypads
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'} };
Adafruit_Keypad Keypad3x4 = Adafruit_Keypad( makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
                      #endif //KEYPAD3x4
byte KeyInputactivated = false;                                      // if pressing the keys stop other events like EveryMinuteUpdate()   

//----------------------------------------
// Common
//----------------------------------------
// 
bool   PrintDebugInfo        = false;                              // for showing debug info for DCFNOINT
bool   SeeDCFsignalInDisplay = false;                              // if ON then the display line HET IS WAS will show the DCF77-signal received
bool   UseDCF                = true;                               // Use the DCF-receiver or not
char   sptext[100];                                                // For common print use
uint16_t  MilliSecondValue  = 1000;                                // The duration of a second  minus 1 ms. Used in Demo mode
static uint32_t last_time = 0;                                     // Heartbeat defines
struct EEPROMstorage {                                             // Data storage in EEPROM to maintain them after power loss
  byte LightReducer;
  byte LowerBrightness;
  byte DisplayChoice;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
  uint32_t OwnColour;                                              // Self defined colour for clock display
  uint32_t EdMinTijd;                                              // Counter of valid time but invalid date
  uint32_t EdMin;                                                  // Counter of total valid date/times since start for DCFNOINT 
  uint32_t ThMin;                                                  // Counter of totalvalid date/times since start for DCF77
  uint32_t EdTh;                                                   // Valid date/times since start counters for DCFNOINT and DCF77
  uint32_t EdThMin;                                                // Valid times per minute counters for DCFNOINT and DCF77
  uint32_t ThWrong;                                                // Different times reported DCFNOINT and DCF77
  uint32_t EdWrong;                                                // Different times reported DCFNOINT and DCF77
  uint32_t ValidTimes;                                             // Counter of total recorded valid times
  uint32_t MinutesSinceStart;                                      // Counter of recorded valid and not valid times
} Mem ; // = {0};
//------------------------------------------------------------------------------
// Menu
//------------------------------------------------------------------------------  
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890  
 char menu[][42] = {                       // menu[][nn]  nn is largest length of sentence in the menu 
 "Woordklok RGBW",
 "Enter time as: hhmmss (132145)",
 "A Debug DCF-signal",
 "D D15122017 is date 15 December 2017",   //
 "G DCF-signalinfo in display",
 "H Use DCF-receiver",
 "L (L5) Min lightintensity(0-255 bits)",  //
 "M (M90)Max lightintensity(1%-250%)",
 "N (N2208)Turn OFF LEDs between Nhhhh",   //
 "P (P00234F8A) own colour (n=0-F)",
 "Q Display Choice (Q0-6) Q=Choices)",
 "I For this info",
 "R Reset to default settings",
 "S Self test",
 "W Test LDR reading every second",       //
 "X (X50) Demo mode. ms delay (0-9999)",
 "Ed Nieuwenhuys May 2021" };
//  -------------------------------------   End Definitions  ---------------------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Loop
//------------------------------------------------------------------------------
void loop()
{
                      InputDevicesCheck();                                                    // Check for input from input devices
 if (Demo)            Demomode();
 else if (Zelftest)   Selftest();   
 else                 EverySecondCheck();  
}  
//------------------------------------------------------------------------------
// ARDUINO Setup
//------------------------------------------------------------------------------
//                                                                                            //
void setup()
{                                                            
 Serial.begin(9600);                                                                          // Setup the serial port to 9600 baud       
//  int32_t Tick = millis();                                                                  // Start the timer 
//  while (!Serial)                                                                           // Wait until serial port is started 
//     {if ((millis() - Tick) >1000) break;}                                                  // Prevents hanging if serial monitor/port is not connected  Tekstprintln("*********\nSerial started"); 
 Tekstprintln("\n*********\nSerial started"); 
 Wire.begin();                                                                                // Start communication with I2C / TWI devices     
 pinMode(DCF_LED_Pin,  OUTPUT);                                                               // For showing DCF-pulse with a LED
 pinMode(DCF_PIN,      INPUT_PULLUP);                                                         // Use Pin 2 for DCF input
 pinMode(DCFgood,      OUTPUT ); 
 pinMode(secondsPin,   OUTPUT );  
 pinMode(HeartbeatLED, OUTPUT );      
                          #ifdef MOD_DS3231
 RTCklok.begin();
 Tekstprintln("RTC DS3231 enabled");// start the RTC-module
                          # elif 
 RTCklok.begin(DateTime(F(__DATE__), F(__TIME__)));                                           // If no RTC module is installed use the ATMEGAchip clock
 Tekstprintln("Internal clock enabled");   
                          #endif  //MOD_DS3231 
                          #ifdef BLUETOOTHMOD 
                          #ifdef ARDUINO_SAMD_MKRWIFI1010 
 Serial1.begin(9600);                                                                         // Bluetooth connected to Serial1
                          #else
 Bluetooth.begin(9600);  
                          #endif  //ARDUINO_SAMD_MKRWIFI1010
 Tekstprintln("Bluetooth enabled");
                          #endif  //BLUETOOTHMOD
                          #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary enabled"); 
 myEnc.write(0);                                                                              // Clear Rotary encode buffer
                          #endif  //ROTARYMOD 
                          #ifdef KEYPAD3x1
 Keypad3x1.begin();                                                                           // Start the 3x4 keypad
 Tekstprint("3*1 keypad enabled");  
                          #endif  //KEYPAD3x1
                          #ifdef KEYPAD3x4
 Keypad3x4.begin();                                                                           // Start the 3x4 keypad
 Tekstprint("4*3 keypad enabled");  

                          #endif  //KEYPAD3x4
                          #ifdef ONEWIREKEYPAD  
 Tekstprint("4*3keypad onewire enabled");                                                     // The one wire keypad is enabled
                          #endif  //ONEWIREKEYPAD
                            #ifdef NEOPIXEL 
 strip.begin();                                                                              // Start communication to LED strip
 strip.setBrightness(BRIGHTNESS);                                                            // Set brightness of LEDs
 ShowLeds(); 
 Tekstprintln("LIB NEOPIXEL");   
                          #endif //NEOPIXEL 
                          #ifdef LIBSK6812
 //LED.set_output(LED_PIN);
 LED.setBrightness(BRIGHTNESS);                                           // Set brightness of LEDs
 Tekstprintln("LIBSK6812");                                          // Initialyse SK6812 library
                          #endif //LIBSK6812                                                                                                     // Initialize all pixels to 'off' 
                          #ifdef LED6812    
 Tekstprintln("LEDs SK6812 enabled");
                          #endif  //LED6812  
                          #ifdef LED2812
 Tekstprintln("LEDs WS2812 enabled");
                          #endif  //LED2812 
                          #ifdef HC12MOD
 HC12.begin(9600);               // Serial port to HC12
 Tekstprintln("HC-12 time receiver enabled");
                          #endif  //HC12MOD                           
//                                                                                            //
                          #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                                                   // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                          #endif  //LCDMOD
                          #ifdef DCFMOD
 DCF.Start();                                                                                 // Start the DCF-module
 Tekstprintln("DCF77 enabled");
                          #endif  //DCFMOD
                          #ifdef DCFNOINT
 DCFmsTick = millis();
 Tekstprintln("DCFNOINT enabled");
                          #endif  //DCFNOINT
                          #ifdef ARDUINO_SAMD_MKRWIFI1010
 Tekstprintln("MKR-1010 WIFI starting");
 Setup_MKRWIFI1010();                                                                         // Start the MKR1010
 Tekstprintln("MKR-1010 WIFI enabled");
                          #endif  //ARDUINO_SAMD_MKRWIFI1010
                          #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("Compiled for ARDUINO AVR NANO EVERY");
                          #endif
                          #if defined(__AVR_ATmega328P__) 
 Tekstprintln("Compiled for ARDUINO AVR ATmega328P");
                          #endif
                          #ifdef ATmega644_1284
 Tekstprintln("Compiled for ARDUINO AVR ATmega644_1284");
                          #endif
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));                             // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 EEPROM.get(0,Mem);                                                                           // Get the data from EEPROM
 Mem.LightReducer    = constrain(Mem.LightReducer,1,250);                                     // 
 Mem.LowerBrightness = min(Mem.LowerBrightness, 250);                                         // 
 Mem.DisplayChoice   = min(Mem.DisplayChoice, DIGITAL);                                       // 
 if ( Mem.OwnColour == 0 ) Mem.OwnColour = 0X002345DD;                                        // If memory is empty then store default value, blue  
 EEPROM.put(0,Mem);                                                                           // update EEPROM if some data are out of the constrains     
 DS3231NVRAMRead(0,NVRAMmem);                                                                  // Read the LDR reading from NVRAM DS3231 clock module if present
 Previous_LDR_read = analogRead(PhotoCellPin);                                                // to have a start value
 MinPhotocell      = Previous_LDR_read;                                                       // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                                                       // Stores maximum reading of photocell;                                            
// Selftest();                                                                                // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();                                                                               // Turn on the LEDs with proper time and display choice
 msTick = Looptime = millis();                                                                // Used in KY-040 rotary for debouncing and seconds check, Start of DCF 1 second loop
 } 
//                                                                                            //
//------------------------------------------------------------------------------
// ARDUINO Reset to default settings
//------------------------------------------------------------------------------
void Reset(void)
{
 Mem.LightReducer     = MAXBRIGHTNESS;                                                        // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.LowerBrightness  = LOWBRIGHTNESS;                                                        // Lower limit of Brightness ( 0 - 255)
 Mem.DisplayChoice    = ResetDisplayChoice; 
 Mem.TurnOffLEDsAtHH  = Mem.TurnOnLEDsAtHH = 0;
 Mem.OwnColour        = 0X002345DD;                                                           // Blue 
 Mem.EdMinTijd        = 0;                                                                    // Counter of total valid times since start for DCFNOINT 
 Mem.EdMin            = 0;                                                                    // Counter of total valid date/times since start for DCFNOINT 
 Mem.ThMin            = 0;                                                                    // Counter of totalvalid date/times since start for DCF77
 Mem.EdTh             = 0;                                                                    // Valid date/times since start counters for DCFNOINT and DCF77
 Mem.EdThMin          = 0;                                                                    // Valid times per minute counters for DCFNOINT and DCF77
 Mem.ThWrong          = 0;                                                                    // Different times reported DCFNOINT and DCF77
 Mem.EdWrong          = 0;                                                                    // Different times reported DCFNOINT and DCF77
 Mem.ValidTimes       = 0;                                                                    // Counter of total recorded valid times
 Mem.MinutesSinceStart= 0;                                                                    // Counter of recorded valid and not valid times
 PrintDebugInfo       = 0;
 Previous_LDR_read    = analogRead(PhotoCellPin);                                             // to have a start value
 MinPhotocell         = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
 MaxPhotocell         = Previous_LDR_read;                                                    // Stores maximum reading of photocell;                                            
 TestLDR              = 0;                                                                    // If true LDR display is printed every second
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Demo                 = false;
 Zelftest             = false;
 Is                   = true;                                                                 // toggle of displaying Is or Was
 ZegUur               = true;                                                                 // Say or not Uur in NL clock
 EEPROM.put(0,Mem);                                                                           // Update EEPROM       
 for (int i=0;i<24;i++) NVRAMmem[i] = 3;                                                      // Reset LDR readings 
 DS3231NVRAMWrite(0,NVRAMmem);                                                                // And write it of NVRAM of DS3231
// Selftest();                                                                                  // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();
}
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Version info
//------------------------------------------------------------------------------
void SWversion(void) 
{ 
 unsigned int i;
 PrintLine(40);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); Tekstprintln(menu[i++]));
 PrintLine(48);
 //sprintf(sptext,"LDR average bits measured per hour");                                           Tekstprintln(sptext); 
 sprintf(sptext,"DCF reception per hour");                                                      Tekstprintln(sptext); 
 for (i=0;i<12;i++) { sprintf(sptext," %02d ",i );          Tekstprint(sptext); }               Tekstprintln("");
 for (i=0;i<12;i++) { sprintf(sptext,"%03d ",NVRAMmem[i]); Tekstprint(sptext); }                Tekstprintln("");
 for (i=12;i<24;i++){ sprintf(sptext,"%03d ",NVRAMmem[i]); Tekstprint(sptext); }                Tekstprintln("");
 PrintLine(48);
 sprintf(sptext,"Brightness Min: %3d bits  Max: %3d%%",Mem.LowerBrightness, Mem.LightReducer);  Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min:%4d bits  Max: %3d bits",MinPhotocell, MaxPhotocell);           Tekstprintln(sptext);
 sprintf(sptext,"Number of LEDs: %d   Palette: %d", NUM_LEDS, Mem.DisplayChoice);               Tekstprintln(sptext);  
 sprintf(sptext,"LEDs off between: %02d - %02d",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH);       Tekstprintln(sptext);
 sprintf(sptext,"Software: %s",VERSION);                                                        Tekstprintln(sptext); 
 GetTijd(0);  
 PrintLine(40);
}
void PrintLine(byte Lengte)
{
 for (int n=0; n<Lengte; n++) {Serial.print(F("-"));} Serial.println(); 
}

//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Update routine done every second
//------------------------------------------------------------------------------
void EverySecondCheck(void)
{
 uint32_t ms = millis() - msTick;                                                             // A Digitalwrite() is very time consuming. 
// static bool Dpin;                                                                            // Only write once to improve program speed in the loop()
// if (ms > 1 && Dpin) 
//      {Dpin = LOW; digitalWrite(secondsPin,LOW);}                                              // Turn OFF the second on pin 13 
    if(UseDCF) 
       if(ms ==  (ms>>5)<<5) digitalWrite(DCF_LED_Pin, !digitalRead(DCF_PIN));                // Write received signal to the DCF LED every 32 loops (>>5 <<5)
 if (ms > 999)                                                                                // Every second enter the loop
  {
    GetTijd(0);                                                                               // Update I.Second, I.Minute, I.Hour, I.Day, I.Month, I.Year
    msTick = millis();
    SetSecondColour();                                                                         // Set the colour per second of 'IS' and 'WAS' 
 
//  digitalWrite(secondsPin,HIGH);                                                            // turn ON the second on pin 
//    Dpin = HIGH;
                             #ifdef LCDMOD
   Print_tijd_LCD();
                             #endif  //LCDMOD    
//   if(I.Second % 30 == 0) DimLeds(true);                                                    // Text LED intensity control + seconds tick print every 30 seconds    
//   DimLeds(false);                                                                          // Read the LDR and set intensity for the LEDs
   DimLeds(TestLDR);                                                                          // Every second an intensitiy check and update from LDR reading 
   }
  //  heartbeat();                                                                            // Only heartbeat with ATMEGA1284 or Nano Every

  if (I.Minute != lastminute)   EveryMinuteUpdate();                                           // Enter the every minute routine after one minute
 }

//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Update routine done every minute
//------------------------------------------------------------------------------
void EveryMinuteUpdate(void)
 {
  lastminute = I.Minute;  
  GetTijd(0);
//  if(LEDsAreOff) { LedsOff(); ShowLeds();}                                                  // Turn the LEDs off
//   else {                                                                                   // Turn the LEDs on     
  DimLeds(true);                                                                              // measure the LDR and print the results
  Displaytime();
//                }
  Print_RTC_tijd();

                    #ifdef DCFNOINT
  if(Mem.MinutesSinceStart > 0X0FFFFFF0)                                                      // Reset counters before they overflow
    { Mem.EdMinTijd = Mem.EdMin = Mem.ThMin = Mem.EdThMin = Mem.ValidTimes = Mem.MinutesSinceStart = 0; }
                     #endif  //DCFNOINT
 if (I.Hour != lasthour) EveryHourUpdate(); 
 }
//                                                                                            //
 //------------------------------------------------------------------------------
// CLOCK Update routine done every hour
//------------------------------------------------------------------------------
void EveryHourUpdate(void)
{
 if(I.Hour != lasthour) 
   {
    lasthour = min(I.Hour, 23);
    if(I.Hour == Mem.TurnOffLEDsAtHH) {LEDsAreOff = true;  LedsOff(); ShowLeds();}             // is it time to turn off the LEDs?
    if(I.Hour == Mem.TurnOnLEDsAtHH)  LEDsAreOff = false;                                      // or on?
                    #ifdef DCF77    
    NVRAMmem[lasthour] = (byte) DCF_signal; //
                    #else
    NVRAMmem[lasthour] =(byte)((SumLDRreadshour / NoofLDRreadshour)/4);  // Update the average LDR readings per hour
                    # endif  //DCF77
    SumLDRreadshour  = 0;
    NoofLDRreadshour = 0;
    DS3231NVRAMWrite(0,NVRAMmem); 
                    #ifdef DCFNOINT
    COnehourchange = 0;                                                                         // Reset every hour.  
    SignalFaults   = 0;                                                                         // Reset every hour. 
                    #endif  //DCFNOINT
    }
 if (I.Day != lastday) EveryDayUpdate(); 
}

//------------------------------------------------------------------------------
// CLOCK Update routine done every day
//------------------------------------------------------------------------------
void EveryDayUpdate(void)
{
 if(I.Day != lastday) 
   {
    lastday = I.Day; 
    Previous_LDR_read = analogRead(PhotoCellPin);                                             // to have a start value
    MinPhotocell      = Previous_LDR_read;                                                    // Stores minimum reading of photocell;
    MaxPhotocell      = Previous_LDR_read;                                                    // Stores maximum reading of photocell;
    EEPROM.put(0,Mem);                                                                        // This function uses EEPROM.update() to perform the write, so does not rewrites the value if it didn't change
                    #ifdef DCFNOINT
    DCFlocked = false;                                                                        // RTC time can be updated from DCF time  Tekstprintln("DCFlock is unlocked ");
   // Tekstprintln("DCFLock unlocked in EveryDayUpdate");
                    #endif  //DCFNOINT
    }
}
//------------------------------------------------------------------------------
// CLOCK check for input from devices
// This fubction is called from with the loop()
//------------------------------------------------------------------------------
//                                                                                            //
void InputDevicesCheck(void)
{
 SerialCheck();
                           #ifdef DCF77MOD
 if(UseDCF)  DCF77Check();
 //digitalWrite(DCF_LED_Pin, !digitalRead(DCF_PIN));                                            // Write received signal to the DCF LED 
                           #endif  //DCF77MOD
                           #ifdef ROTARYMOD      
 RotaryEncoderCheck(); 
                           #endif  //ROTARYMOD
                           #ifdef KEYPAD3x4   
 Keypad3x4Check(); 
                           #endif  //KEYPAD3x4
                           #ifdef KEYPAD3x1   
 Keypad3x1Check(); 
                           #endif  //KEYPAD3x1
                           #ifdef ONEWIREKEYPAD3x4   
 OnewireKeypad3x4Check(); 
                           #endif  //ONEWIREKEYPAD3x4
                           #ifdef ONEWIREKEYPAD3x1   
 OnewireKeypad3x1Check(); 
                           #endif  //ONEWIREKEYPAD3x1
                           #ifdef HC12MOD
 HC12Check();
                           #endif  //HC12MOD 
                          #ifdef DCFNOINT
 if(UseDCF)  DCF77Check();                                                                    // More reads for DCFNoInt inproves accurary of signal
                           #endif  //DCFNOINT
 
                           #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                           #endif  //BLUETOOTHMOD
                           #ifdef ARDUINO_SAMD_MKRWIFI1010
 WIFIServercheck(); 
                           #endif  // //
                           #ifdef DCFNOINT
 if(UseDCF)  DCF77Check();                                                                    // More reads for DCFNoInt inproves accurary of signal
                           #endif  //DCFNOINT
}
//------------------------------------------------------------------------------
// CLOCK common print routines
//------------------------------------------------------------------------------
//                                                                                            //
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
//                          #ifdef ATmega644_1284
 Bluetooth.print(tekst);  
//                          #endif
                          #ifdef ARDUINO_SAMD_MKRWIFI1010
 Serial1.print(tekst);  
                          #endif
                          #endif  //BLUETOOTHMOD
}

void Tekstprintln(char const *tekst)
{
 strcpy(sptext,tekst);
 strcat(sptext,"\n");          //sprintf(sptext,"%s\n",tekst);
 Tekstprint(sptext);    
}

//------------------------------------------------------------------------------
// CLOCK Heart beat in LED
//------------------------------------------------------------------------------
void Heartbeat() 
{
 static byte hbval   = 128;                                                                   // Heartbeat initial intensity
 static byte hbdelta = 10;                                                                    // Determines how fast heartbeat is
 unsigned long now   = millis();
 if ((now - last_time) < 40)    return;
 last_time = now;
 if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
 hbval += hbdelta;
 analogWrite(HeartbeatLED, hbval);
}
//------------------------------------------------------------------------------
// CLOCK Demo mode
//------------------------------------------------------------------------------
//                                                                                            //
void Demomode(void)
{
 if ( millis() - msTick == 10)   digitalWrite(secondsPin,LOW);                                // Turn OFF the second on pin 13
 if ( millis() - msTick >= MilliSecondValue)                                                  // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                                          // second++; 
  digitalWrite(secondsPin,HIGH);                                                              // Turn ON the second on pin 13
  I.Second = 60-I.Minute;
  if( ++I.Minute >59) { I.Minute = 0; I.Second = 0; I.Hour++;}
  if( I.Hour >12)                                                                              // If hour is after 12 o'clock 
   {    
    I.Hour = 9;                   
    if (++ Mem.DisplayChoice > DIGITAL)  Mem.DisplayChoice = 0;                               // Start with a following DISPLAYCHOICE AT Nine oclock
   }
  DimLeds(false);
  Displaytime();
  Tekstprintln("");
  SerialCheck();
 }
}

//------------------------------------------------------------------------------
// CLOCK check for serial input
//------------------------------------------------------------------------------
void SerialCheck(void)
{
 static String SerialString;
 while (Serial.available())
  {
   char c = Serial.read();  delay(3); 
   if (c>31 && c<127) SerialString += c;                            // allow input from Space - Del
   else c = 0;
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
}
                           #ifdef HC12MOD
//------------------------------------------------------------------------------
// CLOCK check for HC-12 input
//------------------------------------------------------------------------------  
//                                                                                            //                          
void HC12Check(void)
{
 String HC12String; 

 HC12String.reserve(64);
 HC12String ="";
 HC12.listen();                                                                               // When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (HC12.available()>0)                                                                   // If HC-12 has data
   {       
    char c = HC12.read();     
    if (c>31 && c<127)  HC12String += c;
    else c = 0;                                                      // Allow only input from Space - Del
    delay(3);
   }
 HC12String += "\n";
 if (HC12String.length()>0) 
   {
    Serial.print("Received HC-12: "); Serial.println(HC12String);
    ReworkInputString(HC12String);
    }                  
}                         
                           #endif  //HC12MOD
                           #ifdef BLUETOOTHMOD
//------------------------------------------------------------------------------
// CLOCK check for Bluetooth input
//------------------------------------------------------------------------------                           
void BluetoothCheck(void)
{ 
                           #ifdef ARDUINO_SAMD_MKRWIFI1010 
 while (Serial1.available()) 
  {
   char c = Serial1.read();
   Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;     // delete a CR
  }
                           #else
 Bluetooth.listen();                                                                  //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  { 
   char c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;
   delay(3);
  }
                           #endif  // 
 if (BluetoothString.length()>0) 
   {   
    ReworkInputString(BluetoothString);                                                      // Rework ReworkInputString();
    BluetoothString = "";
   }
}
                           #endif  //BLUETOOTHMOD
                           #ifdef ONEWIREKEYPAD3x4
//------------------------------------------------------------------------------
// KEYPAD check for Onewire Keypad input
//------------------------------------------------------------------------------
//                                                                                            //
void OnewireKeypad3x4Check(void)
{
 byte keyvalue;
 char Key;
 int sensorValue = analogRead(OneWirePin); // read the value from the sensor:
 switch(sensorValue)
  {
    case   0 ... 100:  keyvalue = 13; break;   // noise
    case 101 ... 132:  keyvalue = 12; Key = '*'; break;   // * 
    case 133 ... 154:  keyvalue =  0; Key = '0'; break;   // 0 
    case 155 ... 216:  keyvalue = 11; Key = '#'; break;   // # 
    case 217 ... 281:  keyvalue =  7; Key = '7'; break;   // 7 
    case 282 ... 318:  keyvalue =  4; Key = '4'; break;   // 4 
    case 319 ... 349:  keyvalue =  1; Key = '1'; break;   // 1 
    case 350 ... 390:  keyvalue =  8; Key = '8'; break;   // 8 
    case 391 ... 463:  keyvalue =  5; Key = '5'; break;   // 5 
    case 464 ... 519:  keyvalue =  2; Key = '2'; break;   // 2 
    case 520 ... 619:  keyvalue =  9; Key = '9'; break;   // 9 
    case 620 ... 848:  keyvalue =  6; Key = '6'; break;   // 6 
    case 849 ... 1023: keyvalue =  3; Key = '3'; break;   // 3
  }
 if(keyvalue<13) { Serial.println(Key); delay(300); }
  if (Key == 12)   // *                                                                       // Pressing a * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString ="";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>=0 && Key<10))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if (KeypadString.length()>5)                                                                 // If six numbers are entered rework this to a time hhmmss
   {       
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock settings resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }    
   }
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000) ) 
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
}
                           #endif  //ONEWIREKEYPAD3x4  
                           #ifdef ONEWIREKEYPAD3x1
//--------------------------------------------
// KEYPAD check for Onewire Keypad input with 5V and 1.1, 4.7, 4.7, 4.7 kOhm resistors
//--------------------------------------------
void OnewireKeypad3x1Check(void)
{
 char keyvalue, Key;
 int sensorValue = analogRead(OneWirePin);                         // Read the value from the sensor:
 switch(sensorValue)
   {
    case   0 ... 385:  keyvalue = 99;            break;            // Noise
    case 386 ... 635:  keyvalue = -1; Key = 'G'; break;            // G 
    case 636 ... 910:  keyvalue =  0; Key = 'Y'; break;            // Y 
    case 911 ... 1024: keyvalue =  1; Key = 'R'; break;            // R 
   }
 if(keyvalue<2) 
    { 
     Serial.print(sensorValue); Serial.println(Key); 
     if (Key == 'R') ProcessKeyPressTurn(1);                          // Pressing Red increases hour or minute. 
     if (Key == 'G') ProcessKeyPressTurn(-1);                         // Pressing Green decreases hour or minute. 
     if (Key == 'Y') ProcessKeyPressTurn(0);                          // Pressing Yellow activates the keyboard input. 
     delay(200);     
    }
}
                           #endif //ONEWIREKEYPAD3x1
                           #ifdef KEYPAD3x4
 //------------------------------------------------------------------------------
// KEYPAD check for Keypad input
//------------------------------------------------------------------------------ 
//                                                                                            //                          
void Keypad3x4Check(void)
{ 
 char Key = 0;
 Keypad3x4.tick(); 
 while(Keypad3x4.available())
  {
   keypadEvent e = Keypad3x4.read();  
   if(e.bit.EVENT == KEY_JUST_PRESSED)
     {                                                                                        // Serial.println(F(" pressed"));  
      delay(20);
     }  
   else  if(e.bit.EVENT == KEY_JUST_RELEASED) 
     {
      Key = (char) e.bit.KEY;                                                                 // Serial.print(Key);  Serial.println(F(" released"));
      Keypad3x4.clear();
      delay(20);
     }
   }
 if (Key == 42)   // *                                                                        // Pressing * activates the keyboard input. 
   { 
    KeyInputactivated = true;
    KeyLooptime = millis();
    KeypadString = "";
    ColorLeds("",0,NUM_LEDS-1,0x00FF00);                                                      // Turn all LEDs green
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(F("Key entry activated"));
   }
 if (KeyInputactivated && (Key>47 && Key<58))
   {
    delay(20); 
    KeypadString += Key;                                                                      // Digit keys 0 - 9
    ColorLeds("",0,Key-48,0xFF0000);                                                          // Turn all LEDs red
    ShowLeds();                                                                               // Push data in LED strip to commit the changes
    Serial.println(KeypadString);
   }
 if(KeypadString.length()>5)                                                                  // If six numbers are entered rework this to a time hhmmss
   {  
   if(KeypadString=="999999")
     { 
      KeypadString = "";   
      Reset();
      Serial.println(F("Clock setting resetted"));   
     }
    else 
     {      
      ReworkInputString(KeypadString);                                                        // Rework ReworkInputString();
      KeypadString = "";
      Serial.println(F("Time changed"));
     }
   }
  if (Key == 35)   // #                                                                       // Pressing # changes palettes. 
   { 
    KeypadString ="";
    Mem.DisplayChoice++;
    Mem.DisplayChoice = min(Mem.DisplayChoice, 9);
    Displaytime();
   }
   
 if ( KeyInputactivated && ((millis() - KeyLooptime) > 30000)                                 // Stop keyboard entry after 30 seconds
   {  
    KeyInputactivated = false;                                                                // Stop data entry after 30 seconds. This avoids unintended entry 
    KeypadString ="";
    Serial.println(F("Keyboard entry stopped"));
  }
} 
                           #endif  //KEYPAD3x4   
//                                                                                            //
                           #ifdef KEYPAD3x1
 //------------------------------------------------------------------------------
// KEYPAD check for Keypad input
//------------------------------------------------------------------------------                           
void Keypad3x1Check(void)
{ 
 char Key = 0;
 Keypad3x1.tick(); 
 while(Keypad3x1.available())
  {
   keypadEvent e = Keypad3x1.read();  
   Serial.print((char)e.bit.KEY);
   Key = (char) e.bit.KEY;   
   if(e.bit.EVENT == KEY_JUST_PRESSED) {Serial.println(F(" pressed"));}  
   if(e.bit.EVENT == KEY_JUST_RELEASED){Serial.println(F(" released"));}
   }
 if (Key == 'G') ProcessKeyPressTurn(-1);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'Y') ProcessKeyPressTurn( 0);                                                     // Pressing Yellow activates the keyboard input. 
 if (Key == 'R') ProcessKeyPressTurn( 1);                                                     // Pressing Yellow activates the keyboard input. 
 Keypad3x1.clear();                                                                           // and clear the keyboard buffer
 delay(200);
} 
                           #endif  //KEYPAD3x1      

                          #ifdef ROTARYMOD
//------------------------------------------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//------------------------------------------------------------------------------
void RotaryEncoderCheck(void)
{
 int ActionPress = 999;
 if (digitalRead(clearButton)==LOW ) ProcessKeyPressTurn(0);                     // Set the time by pressing rotary button
 else if(ChangeTime) ActionPress = myEnc.read();                                                 // If the knob is turned store the direction (-1 or 1)
 if (ActionPress != 999)             ProcessKeyPressTurn(ActionPress);                             // Process the ActionPress
 myEnc.write(0);                                                                              // Set encoder pos back to 0
}
                           #endif  //ROTARYMOD  

//------------------------------------------------------------------------------
// CLOCK
// KY-040 or Membrane 3x1 processing input
// encoderPos < 1 left minus 
// encoderPos = 0 attention and selection choice
// encoderPos > 1 right plus
//------------------------------------------------------------------------------
//                                                                                            //
void ProcessKeyPressTurn(int encoderPos)
{
if ((unsigned long) (millis() - RotaryPressTimer) > 60000)        // After 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                            // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos && ( (millis() - Looptime) > 250))               // If rotary turned avoid debounce within 0.25 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                             // Increase  
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }        // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              {if( ++I.Hour >23) { I.Hour = 0; } }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              {  I.Second = 0;
               if( ++I.Minute >59) { I.Minute = 0; if( ++I.Hour >23) { I.Hour = 0; } }   }
           } 
        }    
      if (encoderPos <0)                                            // Decrease
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                              // Change hours
            { if( I.Hour-- ==0) { I.Hour = 23; } }      
           if (NoofRotaryPressed == 2)                              // Change minutes
            { I.Second = 0;
             if( I.Minute-- == 0) { I.Minute = 59; if( I.Hour-- == 0) { I.Hour = 23; } }  }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
 //     myEnc.write(0);                                               // Set encoder pos back to 0
      Looptime = millis();       
     }                                                
   }
 if (digitalRead(clearButton) == LOW )                              // Set the time by pressing rotary button
   { 
    delay(250);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    SeeDCFsignalInDisplay = false;                                  // Shows the DCF-signal in the display
    RotaryPressTimer      = millis();                               // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >10 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                      // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;            BlinkUUR(3, 20);      break; // Change the hours
       case 2:  ChangeTime = true;            BlinkHETISWAS(3, 20); break; // Change the hours        
       case 3:  ChangeLightIntensity = true;  BlinkLetters(5, 10);  break; // Turn on all LEDs and change intensity 
       case 4:  Mem.DisplayChoice = DEFAULTCOLOUR;                  break;
       case 5:  Mem.DisplayChoice = HOURLYCOLOUR;                   break;        
       case 6:  Mem.DisplayChoice = WHITECOLOR;                     break;
       case 7:  Mem.DisplayChoice = OWNCOLOUR;                      break;
       case 8:  Mem.DisplayChoice = OWNHETISCLR;                    break;
       case 9:  Mem.DisplayChoice = WHEELCOLOR;                     break;    
       case 10: Mem.DisplayChoice = DIGITAL;                        break; 
       case 11:
       case 12:
       case 13:                                                     break;
       case 14:  Reset();                                           break;                     
//                     #if defined DCFMOD || defined DCFTINY
//       case 11:  SeeDCFsignalInDisplay = true;                      break; // Show DCF signal   
//                     #endif DCFMOD || DCFTINY
       default: NoofRotaryPressed = 0; 
                Mem.DisplayChoice = DEFAULTCOLOUR;
                SeeDCFsignalInDisplay = ChangeTime = ChangeLightIntensity  = false;  
                Selftest();        
                break;                         
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    sprintf(sptext,"Display choice stored: Q%d", Mem.DisplayChoice);
    Tekstprintln(sptext);  

    Looptime = millis();     
    Displaytime();                                                    // Turn on the LEDs with proper time
   }
 }

//                                                                                            //
//------------------------------------------------------------------------------
//  CLOCK Blink UUR
//------------------------------------------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); UUR; Laatzien(); delay(Delayms);} 
}
//------------------------------------------------------------------------------
//  CLOCK Blink HET IS WAS
//------------------------------------------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; Laatzien(); delay(Delayms);} 
}
//------------------------------------------------------------------------------
//  CLOCK Blink Letters
//------------------------------------------------------------------------------
void BlinkLetters (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); ColorLeds("",  36, 59, 0XFF0000FF);  Laatzien(); delay(Delayms);} 
}
//------------------------------------------------------------------------------
// CLOCK Self test sequence
//------------------------------------------------------------------------------
void Selftest(void)
{ 
 GetTijd(1);                                                                                  // Prints time in Serial monitor
 LedsOff(); 
 HET;   Laatzien(); IS;    Laatzien(); WAS;    Laatzien(); PRECIES; Laatzien(); MTIEN;  Laatzien();  MVIJF; Laatzien();    
 KWART; Laatzien(); VOOR;  Laatzien(); OVER;   Laatzien(); HALF;    Laatzien(); MIDDER; Laatzien(); VIJF;   Laatzien();
 TWEE;  Laatzien(); EEN;   Laatzien(); VIER;   Laatzien(); TIEN;    Laatzien(); TWAALF; Laatzien(); DRIE;   Laatzien();
 NEGEN; Laatzien(); NACHT; Laatzien(); ACHT;   Laatzien(); ZES;     Laatzien(); ZEVEN;  Laatzien();  ELF;   Laatzien(); 
 NOEN;  Laatzien(); UUR;   Laatzien(); EDSOFT; Laatzien();
 Tekstprintln("*");    
 Play_Lights();    
 Zelftest = false; 
 Displaytime();                                                                               // Turn on the LEDs with proper time
}

//------------------------------------------------------------------------------
// CLOCK Say the time and load the LEDs 
// with the proper colour and intensity
//------------------------------------------------------------------------------
void Displaytime(void)
{ 
 LedsOff();                                                                                   // Start by clearing the display to a known state   
 if( Mem.DisplayChoice == DIGITAL ) { TimePlaceDigit(I.Hour,I.Minute); }
 else                                                                                        // If not a difital display 
   {
                     #ifdef NL
    Dutch();
                     #endif  //NL
                     #ifdef UK
    English();
                     #endif  //UK
                     #ifdef DE
    German();
                     #endif  //DE
                     #ifdef FR
    French();
                     #endif  //FR
   }
//                                                                                            //
 SetSecondColour();                                                                           // Set the colour per second of 'IS' and 'WAS'   
 ShowLeds();                                                                                  // and turn on the LEDs
}
//--------------------------- Time functions --------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK utility function prints time to serial
//------------------------------------------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d",I.Hour,I.Minute,I.Second);
 Tekstprintln(sptext);
}

                    #ifdef LCDMOD
//------------------------------------------------------------------------------
// LCD Print time to LCD display
//------------------------------------------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%02d:%02d:%02d",Inow.hour(),Inow.minute(),Inow.second());  lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                      lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%02d-%02d-%04d",Inow.day(),Inow.month(),Inow.year());      lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                    lcd.print(sptext);
}
                    #endif  //LCDMOD


//----------------------- RTC DS3231 functions  -------------------------
//                                                                                            //
//------------------------------------------------------------------------------
// DS3231 Adjust time in RTC with DCF time from tmElements_t Dtime struct
//------------------------------------------------------------------------------
void AdjustRTC(int TimeDiff, tmElements_t Dtime, char const *tekst) 
{
 sprintf(sptext,"%d sec difference RTC:%02d:%02d:%02d %7s:%02d:%02d:%02d ----> time updated ",
     TimeDiff, I.Hour, I.Minute,I.Second,tekst, Dtime.Hour, Dtime.Minute, Dtime.Second); 
 Tekstprintln(sptext);    
 RTCklok.adjust(DateTime(Dtime.Year, Dtime.Month, Dtime.Day, Dtime.Hour, Dtime.Minute, 0));   // Update time here if DCFNOINT is not used otherwise both receiver must received the same date-time
}

//------------------------------------------------------------------------------
// DS3231 Return time from RTC module in seconds since 1-1-2000
//------------------------------------------------------------------------------
time_t GetSecondsSince2000TimeRTC(void)
{
 DateTime RTCtm = RTCklok.now();                                                              // This give the time from 1970
 time_t RTCtime = RTCtm.unixtime() - SECONDS_FROM_1970_TO_2000 - SECS_PER_DAY;                // These are the seconds since 2000
 //  Serial.print("===> RTCtime: "); Serial.println(RTCtime);
 //  sprintf(sptext,"  RTC OK --> %02d:%02d %02d-%02d-%04d ",
 //      RTCtm.hour(), RTCtm.minute(), RTCtm.day(), RTCtm.month(), RTCtm.year());   Tekstprintln(sptext); 
 return(RTCtime);                                                                             // Seconds since 1-1-2000
}
//------------------------------------------------------------------------------
// DS3231 Get time from DS3231
//------------------------------------------------------------------------------
void GetTijd(byte printit)
{
 Inow    = RTCklok.now();
 I.Hour   = min(Inow.hour()  , 24);
 I.Minute = min(Inow.minute(), 59);
 I.Second = min(Inow.second(), 59);
 I.Day    = Inow.day();
 I.Month  = Inow.month();
 I.Year   = Inow.year()-2000;
 I.Wday   = Inow.dayOfTheWeek();
// if (I.Hour > 24) { I.Hour = random(12)+1; I.Minute = random(60)+1; I.Second = 30;}          // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}


//------------------------------------------------------------------------------
// DS3231 utility function prints time to serial
//------------------------------------------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",
     Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
//------------------------------------------------------------------------------
// DS3231 Set time in module and print it
//------------------------------------------------------------------------------
//                                                                                            //
void SetRTCTime(void)
{ 
 I.Hour   = min((byte) I.Hour  , 24);
 I.Minute = min((byte) I.Minute, 59); 
 I.Second = min((byte) I.Second, 59); 
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), I.Hour, I.Minute, I.Second));
 GetTijd(0);                                                                                  // Synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}

//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int  temp3231;
 
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    // Temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & 0b01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else   {temp3231 = -273; }  
  return (temp3231);
}

//------------------------------------------------------------------------------
//  DS3231 Read NVRAM at I2C address 0x57
//  The size of the NVRAM is 32 bytes
//------------------------------------------------------------------------------
template <class T> int DS3231NVRAMWrite(int EEPROMaddress, const T& value)
{
 const byte* p = (const byte*)(const void*)&value;
 unsigned int i,x;
 for (x=0; x< sizeof(value)/32 +1; x++)   // Write in blocks of 32 bytes
  {
  Wire.beginTransmission(0x57);
  Wire.write((int)(EEPROMaddress >> 8));      // MSB
  Wire.write((int)(EEPROMaddress & 0xFF));    // LSB
  for (i = 0; i < 32; i++)   Wire.write(*p++);
  Wire.endTransmission();
  delay(5);
//  sprintf(sptext,"Size:%d writeP: %d x:%d",sizeof(value), p, x); Tekstprintln(sptext);    
  }
return i;
}
//------------------------------------------------------------------------------
//  DS3231 Write NVRAM at I2C address 0x57
//  The size of the NVRAM is 32 bytes
//------------------------------------------------------------------------------
template <class T> int DS3231NVRAMRead(int EEPROMaddress, T& value)
{
 byte* p = (byte*)(void*)&value;
 unsigned int i,x;
 for (x=0; x< sizeof(value)/32 +1; x++)   // Read in blocks of 32 bytes
  {
   Wire.beginTransmission(0x57);
   Wire.write((int)(EEPROMaddress >> 8));     // MSB
   Wire.write((int)(EEPROMaddress & 0xFF));   // LSB
   Wire.endTransmission();
   Wire.requestFrom(0x57,32);                 
   for (i = 0; i<32; i++) {if(Wire.available()) *p++ = Wire.read(); }
   delay(5);
//   sprintf(sptext,"Size:%d readP; %d x:%d",sizeof(value), p, x); Tekstprintln(sptext); 
  }
return i;
}
//                                                                                            //
                           #if defined DCFNOINT || defined DCFMOD
//------------------------------------------------------------------------------
// DCF check for DCF input.
// This subroutine assumes that both DCF-receivers
// are defined with #define DCFMOD and DCFNOINT
//------------------------------------------------------------------------------
//                                                                                            //
void DCF77Check(void)
{
                              #ifdef DCFMOD
 TimeDCF77Seconds = GetSecondsSince2000DCF77();                                               // Seconds since 1-1-2000
 if(TimeDCF77Seconds) //!=0)
   {
//    tmElements_t DCFtm; 
//    breakTime(TimeDCF77Seconds,DCFtm );                                                       // Store the DCFtime in struct DCFtm
//    time_t TimeDiff = abs(TimeDCF77Seconds - GetSecondsSince2000TimeRTC() );
//    if(TimeDiff >2 && TimeDiff <5400 && !DCFlocked)  
//    AdjustRTC(TimeDiff, DCFtm, "DCF77");                                 // In Seconds. Differences less than 1.5 hour with RTC  are changed unchecked
    LastDCF77Seconds = TimeDCF77Seconds;                                                      // Remember the recorded DCF time
    Mem.ThMin++;                                                                              // Add  
    DCFTh = true;                                                                             // Used to check if DCF77 and DCFNOINT receive both a good time    
   }
                             #endif  //DCFMOD 
                             #ifndef DCFMOD 
  DCFTh = true;                                                                               // Necessary for DCFlocking
                             #endif  //DCFMOD    
                             #ifdef DCFNOINT 
  TimeDCFNoIntSecs = GetSecondsSince2000DCFNoInt();                                           // Decode the signals and retrieve seconds since 1-1-2000 if we have received a proper time after 59 secs
  if(TimeDCFNoIntSecs)
    {
//     tmElements_t DCFnitm;                                                                    // Initialyse a time struct
//     breakTime(TimeDCFNoIntSecs, DCFnitm);                                                    // Store the time_t DCFtime in struct DCFnitm (no interrupt time)
//     time_t TimeDiff = abs(TimeDCFNoIntSecs - GetSecondsSince2000TimeRTC());
//     if(TimeDiff >2 && TimeDiff <5400 && !DCFlocked)  
//     AdjustRTC(TimeDiff, DCFnitm, "DCFNoInt");                            // > in Seconds. If difference between DCF and RTC time update the RTC
     LastDCFNoIntSecs = TimeDCFNoIntSecs;                                                     // Remember the recorded DCF time
     Mem.EdMin++;                                                                             // in Seconds. Differences less than 1.5 hour with RTC  are changed unchecked 
     DCFEd = true;                                                                            // Used to check if DCF77 and DCFNoInt receive both a good time 
    }    //  end if (TimeDCFNoInt!=0)       
                             #endif  //DCFNOINT 
 
}
//                                                                                            //
//------------------------------------------------------------------------------
// DCF77 DCFNoInt Check and count valid & invalid 
// DCF decodings,  update RTC if necessary and
// DCFlocked=true to prevent updating RTC.
//------------------------------------------------------------------------------
void CheckValidityDCF(time_t NoIntSecs, time_t DCF77Secs)
{
 if (DCFEd || DCFTh) { DCF_signal++;  Mem.ValidTimes++; }
 else  DCF_signal--;  
 DCF_signal = constrain(DCF_signal,1,99);
 //  if(DCFlocked) return();                                                                  // If no Stats needs about wrong reported times remove //
 if(DCFEd && DCFTh)                                                                           // Check for invalid reported DCF date/times
   {  
    time_t TimeRTCSecs = GetSecondsSince2000TimeRTC(); 
    time_t SecDiffDCF  = abs(NoIntSecs - DCF77Secs);
    if(DCFlocked && DCFEd && (abs(NoIntSecs - TimeRTCSecs >10)))
       {
        Mem.EdWrong++;                                                                        // The DCFNoInt was false  
        PrintTijd("DCFNoInt: ",NoIntSecs,1);
        PrintTijd("     RTC: ",TimeRTCSecs,1);
       }     
    if(DCFlocked && DCFTh && (abs(DCF77Secs - TimeRTCSecs >10)))
      { 
        Mem.ThWrong++;                                                                        // The DCF77 was false. IF DCFlocked then RTC is te right time 
        PrintTijd("   DCF77: ",DCF77Secs,1);
        PrintTijd("     RTC: ",TimeRTCSecs,1);
      }
  //  Serial.print("===>   RTCtime: "); Serial.println(TimeRTCSecs);    Serial.print("===> DCF77time: "); Serial.println(DCF77Secs);    Serial.print("===> NoInttime: "); Serial.println(NoIntSecs);
    if(SecDiffDCF==0)                                                                         // Allow one second difference
      {
       tmElements_t DCF;                                                                      // Initialyse a time struct
       breakTime(NoIntSecs, DCF);                                                             // Store the time_t DCFtime in struct DCFnitm (no interrupt time)
       if(!DCFlocked) AdjustRTC(SecDiffDCF, DCF, "Both DCF");
       Mem.EdThMin++;                                                                         // Count the times both DCFalgorithm received a good time. 
       DCFlocked = true;                                                                      // Lock changing the time because the two DCF algoritm wee identical until unlocked
 //    Tekstprintln("DCFlock = true"); 
      } 
   }                                  
}      
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Print time from seconds since 1-1-2000  
//       and return time struct
//------------------------------------------------------------------------------
 tmElements_t PrintTijd(char const *tekst, time_t Seconds2000, bool printit)  
 {
  tmElements_t TijdReturn; 
  breakTime(Seconds2000,TijdReturn );  
  if(printit) 
   {
    sprintf(sptext,"%s,%02d:%02d:%02d %02d-%02d-%04d/%01d", 
           tekst,TijdReturn.Hour, TijdReturn.Minute, TijdReturn.Second, TijdReturn.Day, TijdReturn.Month, TijdReturn.Year+2000, TijdReturn.Wday);  Tekstprintln(sptext);
   }
 return(TijdReturn); 
 }  
//                                                                                            //
                             #ifdef DCFMOD 
//------------------------------------------------------------------------------
// DCF77  Return time in seconds since 1-1-2000 
//  with Arduino DCF77 library
//------------------------------------------------------------------------------
time_t GetSecondsSince2000DCF77(void)
{
 time_t DCFtime = DCF.getTime() ;                                                             // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   setTime(DCFtime);                                                                          // Needed for DCF77 lib  
   DCFtime -= (SECONDS_FROM_1970_TO_2000 + SECS_PER_DAY);
 //  Serial.print("===> DCFtime: "); Serial.println(DCFtime);
 //  sprintf(sptext,"  DCF77 Lib OK --> %02d:%02d %02d-%02d-%04d ",DCFtm.Hour, DCFtm.Minute, DCFtm.Day, DCFtm.Month, DCFtm.Year+1970);   Tekstprintln(sptext); 
  }
 return(DCFtime);                                                                             // Seconds since 1-1-2000
}
                             #endif  //DCFMOD 
                             #ifdef DCFNOINT
//------------------------------------------------------------------------------
// DCFNoInt Make the time from the received DCF-bits
// Return time in seconds since 1-1-2000
//------------------------------------------------------------------------------
time_t GetSecondsSince2000DCFNoInt(void)
{
        byte     TimeOK     , Receivebit     , Receivebitinfo;                                // Return flag is proper time is computed and received bit value
        byte     BMinutemark, BReserveantenna, BOnehourchange; 
        byte     BSummertime, BWintertime    , BLeapsecond;
        bool     TimeSignaldoubtfull = false;                                                 // If a bit length signal is not between 5% and 30%
 static byte     Bitpos=15,  Paritybit;                                                        //  
 static byte     MinOK=2, HourOK=2, YearOK=2;                                                 // 2 means not set. static because the content must be remembered
 static int      TimeMinutes     = 999;
 static int      LastTimeMinutes = 0;
 static byte     Lastyear        = I.Year;                                                    // Remember the last valid year
 static int      TimeMinutesDiff, DateDaysDiff;
 static uint32_t DateDays ,LastDateDays;
 static uint32_t PreviousSumSecondSignal = 0;
  
 SumSignalCounts++; 
 SumSecondSignal += !digitalRead(DCF_PIN);                                                    // invert 0 to 1 and 1 to 0
                                                                                              // SignalBin[( (millis() - DCFmsTick)/50)] += r;
 if( (millis() - DCFmsTick) <996) return(0);                                                  // Compute every second the received DCF-bit to a time 
 while (!digitalRead(DCF_PIN))                                                                // Avoid splitting a positive signal and wait until the signal hets low
   {
    SumSignalCounts++;        
    SumSecondSignal++;
    if( (millis() - DCFmsTick) >1005) break;                                                 // A little retard of loop speed is fine 
   }
 DCFmsTick = millis();                                                                       // Set one second counter
 SumSignalCounts *= 1.10;                                                                    // Correction to correct pulses to an average of 100 and 200 msecs 
 if(Bitpos >= 60) 
   {
    if(LastDCFNoIntSecs && LastDCF77Seconds) 
        CheckValidityDCF(LastDCFNoIntSecs,LastDCF77Seconds);                                 // Compare the last recorded DCF times and update RTC if necessary
    if(PrintDebugInfo) 
       {
       sprintf(sptext,"EdT:%ld Ed:%ld Th:%ld EdW:%ld ThW:%ld Both:%ld Valid:%ld Min:%ld Signal:%d%% Lock:%s",    
          Mem.EdMinTijd, Mem.EdMin, Mem.ThMin, Mem.EdWrong, Mem.ThWrong, Mem.EdThMin, Mem.ValidTimes, Mem.MinutesSinceStart,DCF_signal, DCFlocked?"Y":"N");
       Tekstprintln(sptext); 
       }
    DCFEd = false;                                                                            // Turn off good DCF-time received
    DCFTh = false;                                              
    Bitpos             = 0;                                                                   // Reset the second counter
    TimeDCF77Seconds   = 0;                                                                   // Time since 1-1-2000 from DCF
    TimeDCFNoIntSecs   = 0;                                                                   // Time since 1-1-2000 from DCF
   } 
 int msec  = (int)(1000 * SumSecondSignal / SumSignalCounts);                                 // This is roughly the milliseconds times 10 of the signal length
 switch(msec)
  {
   case   0 ...  10: if(SumSignalCounts > 500)// && (Bitpos > 1))                                // Check if there were enough signals and there were several positive signals 
                        { 
                         Receivebitinfo = 2; Receivebit = 0;
                         if(PreviousSumSecondSignal!=0)  Bitpos = 59; 
                        }                                                                     // If enough signalcounts and for one second no signal found. This is position zero
                      else { Receivebitinfo = 9; Receivebit = 0; }                            // If not it is a bad signal probably a zero bit
                                                                        break;                // If signal is less than 0.05 sec long than it is the start of minute signal
   case  11 ...  50: Receivebitinfo = 8; Receivebit = 0;                break;                // In all other cases it is an error
   case  51 ... 160: Receivebitinfo = 0; Receivebit = 0;                break;                // If signal is 0.1 sec long than it is a 0 
   case 161 ... 320: Receivebitinfo = 1; Receivebit = 1;                break;                // If signal is 0.2 sec long than it is a 1 
   default:          Receivebitinfo = 9; Receivebit = 1;                break;                // In all other cases it is an error probably a one bit
  } 
 PreviousSumSecondSignal = SumSecondSignal;
 if(Receivebitinfo > 2)  SignalFaults++;                                                      // Count the number of faults per hour
 TimeOK = 0;                                                                                  // Set Time return code to false  
//                                                                                            //
 switch (Bitpos)                                                                              // Decode the 59 bits to a time and date.
  {                                                                                           // It is not binary but "Binary-coded decimal". Blocks are checked with an even parity bit. 
    case  0: BMinutemark = Receivebit;                                                         // Blocks are checked with an even parity bit. 
             BMinutemark = BMinutemark;                                 break;                // Suppress compiler warning                              
   case   1: TimeSignaldoubtfull = 0;                                                         // Start a fresh minute          
             D.Minute = D.Hour = D.Day = D.Wday = D.Month = D.Year = 0;                       // Reset the variables
             MinOK = HourOK  = YearOK = 2;                                                    // Gives the dot (.) at MHY or mhy
   case 2 ... 14:                                                       break;                // reserved for wheather info we will not use
   case  15: BReserveantenna = Receivebit;
             BReserveantenna = BReserveantenna;                         break;                // 1 if reserve antenna is in use
   case  16: BOnehourchange  = Receivebit;
             if(BOnehourchange) COnehourchange++;
             else               COnehourchange--;
             COnehourchange = constrain(COnehourchange,0, 50);
             if(COnehourchange >25)
               {
                 DCFlocked = false; 
                 Tekstprintln("DCFLock unlocked One hour change");
              }                                                         break;                // Unlock if one hour change comes up 
   case  17: BSummertime = Receivebit;
             if(Receivebit) CSummertime++; 
             else           CSummertime--;
             CSummertime = constrain(CSummertime,5,60);                
             BSummertime = BSummertime;                                 break;                // 1 if summer time 
   case  18: BWintertime = Receivebit; 
             if(Receivebit) CWintertime++;
             else           CWintertime--;
             CWintertime = constrain(CWintertime,5,60);
             BWintertime = BWintertime;                                 break;                // 1 if winter time
   case  19: BLeapsecond = Receivebit; BLeapsecond = BLeapsecond;       break;                // Announcement of leap second in time setting is coming up
   case  20: StartOfEncodedTime=Receivebit; Paritybit = 0;              break;                // Bit must always be 1 
   case  21: if(Receivebit) {D.Minute  = 1;  Paritybit = 1 - Paritybit;} break;
   case  22: if(Receivebit) {D.Minute += 2 ; Paritybit = 1 - Paritybit;} break;
   case  23: if(Receivebit) {D.Minute += 4 ; Paritybit = 1 - Paritybit;} break;
   case  24: if(Receivebit) {D.Minute += 8 ; Paritybit = 1 - Paritybit;} break;
   case  25: if(Receivebit) {D.Minute += 10; Paritybit = 1 - Paritybit;} break;
   case  26: if(Receivebit) {D.Minute += 20; Paritybit = 1 - Paritybit;} break;
   case  27: if(Receivebit) {D.Minute += 40; Paritybit = 1 - Paritybit;} break;
   case  28: MinOK = (Receivebit==Paritybit);  Paritybit = 0;           
             if(D.Minute>59) MinOK=0;                                    break;                // The minute parity is OK or NOK    
   case  29: if(Receivebit) {D.Hour   =  1; Paritybit = 1 - Paritybit;}  break;
   case  30: if(Receivebit) {D.Hour  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  31: if(Receivebit) {D.Hour  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  32: if(Receivebit) {D.Hour  +=  8; Paritybit = 1 - Paritybit;}  break;
   case  33: if(Receivebit) {D.Hour  += 10; Paritybit = 1 - Paritybit;}  break;
   case  34: if(Receivebit) {D.Hour  += 20; Paritybit = 1 - Paritybit;}  break;
   case  35: HourOK = (Receivebit==Paritybit); Paritybit = 0;           
             if(D.Hour>23) HourOK=0;                                     break;                // The hour parity is OK or NOK 
   case  36: if(Receivebit) {D.Day    =  1; Paritybit = 1 - Paritybit;}  break;
   case  37: if(Receivebit) {D.Day   +=  2; Paritybit = 1 - Paritybit;}  break;
   case  38: if(Receivebit) {D.Day   +=  4; Paritybit = 1 - Paritybit;}  break;
   case  39: if(Receivebit) {D.Day   +=  8; Paritybit = 1 - Paritybit;}  break;
   case  40: if(Receivebit) {D.Day   += 10; Paritybit = 1 - Paritybit;}  break;
   case  41: if(Receivebit) {D.Day   += 20; Paritybit = 1 - Paritybit;}  break;
   case  42: if(Receivebit) {D.Wday   =  1; Paritybit = 1 - Paritybit;}  break;
   case  43: if(Receivebit) {D.Wday  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  44: if(Receivebit) {D.Wday  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  45: if(Receivebit) {D.Month  =  1; Paritybit = 1 - Paritybit;}  break;
   case  46: if(Receivebit) {D.Month +=  2; Paritybit = 1 - Paritybit;}  break;
   case  47: if(Receivebit) {D.Month +=  4; Paritybit = 1 - Paritybit;}  break;
   case  48: if(Receivebit) {D.Month +=  8; Paritybit = 1 - Paritybit;}  break;
   case  49: if(Receivebit) {D.Month += 10; Paritybit = 1 - Paritybit;}  break;
   case  50: if(Receivebit) {D.Year   =  1; Paritybit = 1 - Paritybit;}  break;
   case  51: if(Receivebit) {D.Year  +=  2; Paritybit = 1 - Paritybit;}  break;
   case  52: if(Receivebit) {D.Year  +=  4; Paritybit = 1 - Paritybit;}  break;
   case  53: if(Receivebit) {D.Year  +=  8; Paritybit = 1 - Paritybit;}  break;
   case  54: if(Receivebit) {D.Year  += 10; Paritybit = 1 - Paritybit;}  break;
   case  55: if(Receivebit) {D.Year  += 20; Paritybit = 1 - Paritybit;}  break;
   case  56: if(Receivebit) {D.Year  += 40; Paritybit = 1 - Paritybit;}  break;
   case  57: if(Receivebit) {D.Year  += 80; Paritybit = 1 - Paritybit;}  break;
   case  58: YearOK = (Receivebit==Paritybit);                                                // The year month day parity is OK or NOK   
             Paritybit = 0;
             if(D.Day>31 || D.Month==0 || D.Month>12 || Lastyear!=D.Year) YearOK=0; 
             Mem.MinutesSinceStart++;                                                           // 
                                                                         break;              
   case  59:                                                            // 
            if(MinOK && HourOK) TimeMinutes = D.Hour*60 + D.Minute;
            else TimeMinutes++;                                                                // 
            if(D.Month==0) break;
            DateDays        = ((D.Year*12 + D.Month)*31) + D.Day;                             //
            TimeMinutesDiff = abs(TimeMinutes - LastTimeMinutes);                             //
            DateDaysDiff    = (uint32_t) abs(DateDays - LastDateDays);                        //
            if(PrintDebugInfo)
             { 
               sprintf(sptext,"%d minute%s difference between Now:%d and Last:%d minutes",
                    TimeMinutesDiff, TimeMinutesDiff==1?"":"s", TimeMinutes, LastTimeMinutes);            
               Serial.println(sptext);
              }
            LastTimeMinutes = TimeMinutes;                                                    //
            LastDateDays    = DateDays;  
            if(MinOK && HourOK &&  YearOK && TimeMinutesDiff<2 && DateDaysDiff<2) TimeOK = 1; // All OK          
            if(MinOK && HourOK && !YearOK && TimeMinutesDiff<2)                   TimeOK = 2; // If a date difference time remains usable
            if(TimeOK) LastTimeMinutes = TimeMinutes; 
            if(TimeOK==1)Lastyear = D.Year;   
            if(TimeSignaldoubtfull)      {}//  {Dday = 0; TimeOK = 5;}                        // If a signal lenght was too short or long this last minute time is not OK
                                                                        break;
    default:  Serial.println(F("Default in BitPos loop. Impossible"));  break;
   }   // End switch(bitpos)
   if(TimeOK==2)
     {
      D.Day = I.Day; D.Month = I.Month; D.Year = I.Year;  D.Wday = I.Wday;
      Mem.EdMinTijd++;   
      TimeOK = 1;
     }
  if((Bitpos>19 && Bitpos<36) && Receivebitinfo>2) TimeSignaldoubtfull = true;                // If a date time bit is received and the received bit is not 0 or 1 then time becomes doubtfull
  if(PrintDebugInfo)
    {
     sprintf(sptext,"@%s%s%s", (MinOK<2?(MinOK?"M":"m"):"."),(HourOK<2?(HourOK?"H":"h"):"."),(YearOK<2?(YearOK?"Y":"y"):"."));   Tekstprint(sptext);
     sprintf(sptext,"%6ld %5ld %2ld%% [%d] %02d:%02d|%02d| %02d-%02d-%04d/%01d F%ld OK:%d",
             SumSecondSignal, SumSignalCounts, 100*SumSecondSignal/SumSignalCounts, Receivebitinfo, 
             D.Hour, D.Minute, Bitpos, D.Day, D.Month, D.Year+2000, D.Wday, SignalFaults, TimeOK);  Tekstprintln(sptext);
     //if(Bitpos==10) {   
     //for(int i=0 ;i<10;i++){sprintf(sptext,"%4lu ", SignalBin[i]);  Serial.print(sptext); SignalBin[i]=0;}
     //for(int i=10;i<20;i++){sprintf(sptext,"%4lu ", SignalBin[i]);  Serial.print(sptext); SignalBin[i]=0;}   Serial.println("");}
    }
 SumSecondSignal = SumSignalCounts = 0;
 D.Second = Bitpos++;
 return(TimeOK == 1?makeTime(D):0);                                                           // If D = DCF time struct tmElements_t
}
                             #endif  //DCFNOINT
//                                                                                            //     
                             #endif  //defined DCFNOINT || defined DCFMOD

// --------------------Colour Clock Light functions -----------------------------------
//------------------------------------------------------------------------------
//  LED Set color for LEDs in strip and print tekst
//------------------------------------------------------------------------------
void ColorLeds(char const *Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
 Stripfill(RGBWColor, FirstLed, ++LastLed - FirstLed );                                        // for(int n = FirstLed; n <= LastLed; n++)  strip.setPixelColor(n,RGBWColor  ); Serial.println(RGBWColor,HEX);  
 if (!NoTextInColorLeds && strlen(Tekst) > 0 )
     {sprintf(sptext,"%s ",Tekst); Tekstprint(sptext); }                                      // Print the text  
}
//--------------------------------------------
//  LED Set color for one LED
//--------------------------------------------
void ColorLed(int Lednr, uint32_t RGBWColor)
{   
 Stripfill(RGBWColor, Lednr, 1 );
}
//------------------------------------------------------------------------------
//  LED Clear display settings of the LED's
//------------------------------------------------------------------------------
void LedsOff(void) 
{ 
 Stripfill(0, 0, NUM_LEDS );                                                                  // for(int n = 0; n <= NUM_LEDS; n++)  strip.setPixelColor(n,0); 
}
//------------------------------------------------------------------------------
// LED Turn On and Off the LED's after Delaytime is milliseconds
//------------------------------------------------------------------------------
void Laatzien()
{ 
 ShowLeds();
 delay(Delaytime);
 LedsOff(); 
 InputDevicesCheck();                                                                         // Check for input from input devices
}

//------------------------------------------------------------------------------
//  LED Push data in LED strip to commit the changes
//------------------------------------------------------------------------------
void ShowLeds(void)
{
                     #ifdef LIBSK6812
 LED.show();
                     #endif // LIBSK6812         
                     #ifdef NEOPIXEL  
 strip.show();
                     #endif //NEOPIXEL
}
//------------------------------------------------------------------------------
//  LED Set brighness of LEDs
//------------------------------------------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
                        #ifdef LIBSK6812
 LED.setBrightness(Bright);                                           // Set brightness of LEDs   
                       #endif  // LIBSK6812
                     #ifdef NEOPIXEL  
 strip.setBrightness(Bright); 
                     #endif //NEOPIXEL
 ShowLeds();
}
//--------------------------------------------
//  LED Fill the strip array for LEDFAB library
//--------------------------------------------
void Stripfill(uint32_t RGBWColor, int FirstLed, int NoofLEDs)
{  
                  #ifdef LIBSK6812   
 LED.fill(RGBWColor, FirstLed, NoofLEDs );
                   #endif  // LIBSK6812  

                   #ifdef NEOPIXEL    
 strip.fill(RGBWColor, FirstLed, NoofLEDs );
                   #endif //NEOPIXEL

}
//--------------------------------------------
//  LED Strip Get Pixel Color 
//--------------------------------------------
uint32_t StripGetPixelColor(int Lednr)
{
uint32_t PixColor;
                  #ifdef LIBSK6812   
PixColor = LED.getPixelColor(Lednr);
                   #endif  // LIBSK6812  
                   #ifdef NEOPIXEL    
PixColor = strip.getPixelColor(Lednr);
                   #endif //NEOPIXEL
return(PixColor);
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED convert HSV to RGB  h is from 0-360, s,v values are 0-1
//  r,g,b values are 0-255
//------------------------------------------------------------------------------
uint32_t HSVToRGB(double H, double S, double V) 
{
  int i;
  double r, g, b, f, p, q, t;
  if (S == 0)  {r = V;  g = V;  b = V; }
  else
  {
    H >= 360 ? H = 0 : H /= 60;
    i = (int) H;
    f = H - i;
    p = V * (1.0 -  S);
    q = V * (1.0 - (S * f));
    t = V * (1.0 - (S * (1.0 - f)));
    switch (i) 
    {
     case 0:  r = V;  g = t;  b = p;  break;
     case 1:  r = q;  g = V;  b = p;  break;
     case 2:  r = p;  g = V;  b = t;  break;
     case 3:  r = p;  g = q;  b = V;  break;
     case 4:  r = t;  g = p;  b = V;  break;
     default: r = V;  g = p;  b = q;  break;
    }
  }
return FuncCRGBW((int)(r*255), (int)(g*255), (int)(b*255), 0 );                                // R, G, B, W 
}
//------------------------------------------------------------------------------
//  LED function to make RGBW color
//------------------------------------------------------------------------------ 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//------------------------------------------------------------------------------
//  LED functions to extract RGBW colors
//------------------------------------------------------------------------------ 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Set second color
//  Set the colour per second of 'IS' and 'WAS'
//------------------------------------------------------------------------------
void SetSecondColour(void)
{
 MINColor = FuncCRGBW(15 + I.Minute * 4, 255 - I.Minute * 4,0,0); 
 SECColor = FuncCRGBW(15 + I.Second * 4, 255 - I.Second * 4,0,0 );    
 switch (Mem.DisplayChoice)
  {
   case DEFAULTCOLOUR: LetterColor = DefaultColor;                                     break; // Yellow text with changing MIN and SEC  
   case HOURLYCOLOUR : LetterColor = HourColor[I.Hour];                                break; // A colour every hour
   case WHITECOLOR   : LetterColor = MINColor = SECColor = WhiteColour;                break; // all white
   case OWNCOLOUR    : LetterColor = Mem.OwnColour;                                    break; // own colour
   case OWNHETISCLR  : LetterColor = Mem.OwnColour; MINColor = SECColor = LetterColor; break; // own colour except HET IS WAS  
   case WHEELCOLOR   : LetterColor = MINColor = SECColor = Wheel((I.Minute*4));        break; // Colour of all letters changes per second
   case DIGITAL      : LetterColor = white; MINColor = SECColor = 0;                   break; // digital display of time. No IS WAS turn color off in display
  }
 //uint32_t ff = SECColor;                                                                    // Remember the colour of the seconds 
 NoTextInColorLeds  = true;                                                                   // Flag to control printing of the text IS en WAS in serial
// if (Is) { IS; SECColor=0; WAS; SECColor=ff; }                                              // Light up  IS or WAS with the proper colour
//   else  { SECColor=0; IS; SECColor=ff; WAS; }
 if (Is) { IS;  }  // Light up  IS or WAS with the proper colour
   else  { WAS; }
 NoTextInColorLeds  = false;                                                                  // Flag to control printing of the text IS en WAS in serial
 ShowLeds();                                                                                  // Updating IS and WAS with ShowLeds is done here to avoid updating all letters every second with Displaytime function
//    Serial.print("SecColor: ");    Serial.println(SECColor,HEX);  
//                                                                                            //
}
//------------------------------------------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//------------------------------------------------------------------------------                                                                                           

void DimLeds(byte print) 
{                                                                                                       
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;                       // Read lightsensor 0 - 1024 bits 
  Previous_LDR_read = LDR_read;
  OutPhotocell = (int)((Mem.LightReducer * sqrt(63.5*LDR_read))/100);                          // 0-255 bits.  Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
  BrightnessCalcFromLDR = (byte)constrain((int)OutPhotocell, Mem.LowerBrightness, 255);        // Filter out of strange results 
  SumLDRreadshour += LDR_read;                                                                 // For statistics LDR readings per hour
  NoofLDRreadshour++;
  if(print)
  {
   sprintf(sptext,"Sensor:%3d",(analogRead(PhotoCellPin)));        Tekstprint(sptext);
   sprintf(sptext," Min:%3d",MinPhotocell);                        Tekstprint(sptext);
   sprintf(sptext," Max:%3d",MaxPhotocell);                        Tekstprint(sptext);
   sprintf(sptext," Out:%3d",OutPhotocell);                        Tekstprint(sptext);
   sprintf(sptext,"=%2d%%",(int)(100*BrightnessCalcFromLDR/255));  Tekstprint(sptext);
   sprintf(sptext," Temp:%2dC ", get3231Temp());                   Tekstprintln(sptext);  
 //  Print_tijd(); 
  }
 if(LEDsAreOff) BrightnessCalcFromLDR = 0 ;
 SetBrightnessLeds(BrightnessCalcFromLDR);
}
//------------------------------------------------------------------------------
//  LED Turn On en Off the LED's
//------------------------------------------------------------------------------
void Play_Lights()
{
 for(int n=0; n<12; n++)   { for(int i=0; i<NUM_LEDS; i++) { ColorLeds("",i,i,HourColor[n]); ShowLeds(); } }
 WhiteOverRainbow(50, 50, 5 );
 WhiteOverRainbow( 5,  5, 5 );  // wait, whiteSpeed, whiteLength
 LedsOff();
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//------------------------------------------------------------------------------
uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

//------------------------------------------------------------------------------
//  LED RainbowCycle
// Slightly different, this makes the rainbow equally distributed throughout
//------------------------------------------------------------------------------
void RainbowCycle(uint8_t wait) 
{
  uint16_t i, j;
  for(j=0; j<256 * 5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< NUM_LEDS; i++) {
 //     strip.setPixelColor(i, Wheel(((i * 256 / NUM_LEDS) + j) & 255));
      ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
    }
    ShowLeds();
    delay(wait);
  }
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED WhiteOverRainbow
//------------------------------------------------------------------------------
void WhiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint8_t whiteLength ) 
{
  if(whiteLength >= NUM_LEDS) whiteLength = NUM_LEDS - 1;
  int head = whiteLength - 1;
  int tail = 0;
  int loops = 1;
  int loopNum = 0;
  static unsigned long lastTime = 0;
  while(true)
  {
    for(int j=0; j<256; j++) 
     {
      for(int i=0; i<NUM_LEDS; i++) 
       {
        if((i >= tail && i <= head) || (tail > head && i >= tail) || (tail > head && i <= head) )
             // strip.setPixelColor(i, strip.Color(0,0,0, 255 ) );
              ColorLeds("",i,i,0XFFFFFF );
        else  //strip.setPixelColor(i, Wheel(((i * 256 / NUM_LEDS) + j) & 255));
        ColorLeds("",i,i,Wheel(((i * 256 / NUM_LEDS) + j) & 255));
       }
      if(millis() - lastTime > whiteSpeed) 
       {
        head++;        tail++;
        if(head == NUM_LEDS) loopNum++;
        lastTime = millis();
      }
      if(loopNum == loops) return;
      head %= NUM_LEDS;
      tail %= NUM_LEDS;
      ShowLeds();
      delay(wait);
    }
  }  // end while
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Place digits 0 - 9 in Matrix display
// First row and column = 0, PosX,PosY is left top position of 3x5 digit
// Calculate position LED #define MATRIX_WIDTH 12 #define MATRIX_HEIGHT 12
//------------------------------------------------------------------------------
void Zet_Pixel(byte Cijfer,byte Pos_X, byte Pos_Y) 
{ 
 uint32_t LEDnum;
 uint32_t OrgColor;
// CheckColourStatus();
 for(int i=0;i<3;i++)
  {  
   for(int j=0;j<5;j++)
   {
    int c = pgm_read_byte_near ( &Getal[Cijfer][i][j]); 
    if ( c )                                                                                // if Digit == 1 then turn that light on
     {                                                                                      // Serial.print(strip.getPixelColor(LEDnum) & 0X00FFFFFF,HEX); Serial.print(" ");
      if((Pos_Y+j)%2) LEDnum = ((MATRIX_WIDTH -1) - (Pos_X + i) + (Pos_Y + j) * (MATRIX_HEIGHT));
      else            LEDnum =                      (Pos_X + i) + (Pos_Y + j) * (MATRIX_HEIGHT); 
      StripGetPixelColor(LEDnum) && white ? OrgColor = LetterColor : OrgColor = 0;
      ColorLeds("",  LEDnum, LEDnum, (uint32_t)(OrgColor + white));
     }
   }
 }
}
//------------------------------------------------------------------------------
//  LED Time in four digits in display
//------------------------------------------------------------------------------
void TimePlaceDigit(byte uur, byte minuut)
{   
 Zet_Pixel(    uur / 10, 2, 1);  Zet_Pixel(    uur % 10, 7, 1);
 Zet_Pixel( minuut / 10, 2, 7);  Zet_Pixel( minuut % 10, 7, 7);
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED In- or decrease light intensity value
//------------------------------------------------------------------------------
void WriteLightReducer(int amount)
{
 int value = Mem.LightReducer + amount;                                                       // Prevent byte overflow by making it an integer before adding
 Mem.LightReducer = min(value,  255);                                                         // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
 Tekstprintln(sptext);
}

//------------------------------------------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//------------------------------------------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 Mem.LowerBrightness = min(waarde, 255);                                                      // Range between 0 and 255
 sprintf(sptext,"Lower brightness: %3d bits", Mem.LowerBrightness);
 Tekstprintln(sptext);
}

// --------------------End Light functions 
//------------------------------------------------------------------------------
//  CLOCK Constrain a string with integers
// The value between the first and last character in a string is returned between the  low and up bounderies
//------------------------------------------------------------------------------
int SConstrainInt(String s,byte first,byte last,int low,int up){return constrain(s.substring(first, last).toInt(), low, up);}
int SConstrainInt(String s,byte first,          int low,int up){return constrain(s.substring(first).toInt(), low, up);}
//------------------------------------------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//------------------------------------------------------------------------------
//                                                                                            //
void ReworkInputString(String InputString)
{
 InputString.trim();                                                                          // Remove trailing spaces
 if (InputString.length()>10) return;                                                         // If string is too long for some reason
 if (InputString[0] > 64 && InputString[0] <123)                                              // If the first charater is a letter
  {
  sprintf(sptext,"**** Length fault ****");                                                   // Default message 
  Serial.println(InputString);
  switch (InputString[0]) 
   {
    case 'A':
    case 'a':
            if (InputString.length() == 1)
              {
               PrintDebugInfo = 1 - PrintDebugInfo;
               sprintf(sptext,"See DCF debug info: %s",PrintDebugInfo ? "On" : "Off");
              }
             break;  
    case 'C':
    case 'c':
             if(InputString.length() == 1)
               {
                Mem.LightReducer    = MAXBRIGHTNESS;
                Mem.LowerBrightness = LOWBRIGHTNESS;
                Mem.DisplayChoice = Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = Mem.OwnColour = 0;
                Mem.EdMinTijd = Mem.EdMin = Mem.ThMin = Mem.EdTh = Mem.EdThMin = Mem.ThWrong = 0;
                Mem.EdWrong = Mem.ValidTimes = Mem.MinutesSinceStart = 0;
//                                                                                            //
                for (int i=0;i<24;i++) NVRAMmem[i] = 2;                                       // Reset LDR readings 
                DS3231NVRAMWrite(0,NVRAMmem);                                                 // And write it of NVRAM of DS3231  
                sprintf(sptext,"Data were cleared");   
               }
              if(InputString.length() == 3)
               {
                for (unsigned int i=0 ; i<EEPROM.length(); i++) { EEPROM.write(i, 0); }
                Tekstprintln("EEPROM data were erased"); 
                strcpy(sptext,""); 
                setup();
               }
               break;
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              I.Day   = (byte) SConstrainInt(InputString,1,3,0,31);
              I.Month = (byte) SConstrainInt(InputString,3,5, 0, 12); 
              Jaar   =        SConstrainInt(InputString,5,9, 2000, 3000); 
              RTCklok.adjust(DateTime(Jaar, I.Month, I.Day, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%02d:%02d:%02d %02d-%02d-%04d",Inow.hour(),Inow.minute(),Inow.second(),I.Day,I.Month,Jaar);
             }
            break;
//                                                                                            //
    case 'G':                                                                                 // Toggle DCF Signal on Display
    case 'g':
            if (InputString.length() == 1)
              {
               SeeDCFsignalInDisplay = 1 - SeeDCFsignalInDisplay;
               sprintf(sptext,"SeeDCFsignal:%s",SeeDCFsignalInDisplay ? "On" : "Off");
               }
             break;
    case 'H':                                                                                 // Toggle DCF Signal on Display
    case 'h':
            if (InputString.length() == 1)
              {
               UseDCF = 1 - UseDCF;
               sprintf(sptext,"Use DCF-receiver:%s",UseDCF ? "On" : "Off");
               }            
             break; 
    case 'I':
    case 'i': 
            if (InputString.length() == 1)
            {
             SWversion();
             sptext[0] = 0;                                                                      // Clear sptext    

            }
            break;
    case 'L':                                                                                 // Lowest value for Brightness
    case 'l':
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                sprintf(sptext,"Lower brightness changed to: %d bits",Mem.LowerBrightness);
               }
             break;  
    case 'M':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':
            if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Max brightness changed to: %d%%",Mem.LightReducer);
               }
              break;
    case 'N':
    case 'n':
             if (InputString.length() == 1 )         Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = 0;
             if (InputString.length() == 5 )
              {
               Mem.TurnOffLEDsAtHH = SConstrainInt(InputString,1,3,0,23);
               Mem.TurnOnLEDsAtHH  = SConstrainInt(InputString,3,5,0,23); 
              }
              sprintf(sptext,"LEDs are OFF between %2d:00 and %2d:00", Mem.TurnOffLEDsAtHH,Mem.TurnOnLEDsAtHH );
             break;
    case 'O':
    case 'o':
             if(InputString.length() == 1)
               {
                LEDsAreOff = !LEDsAreOff;
                sprintf(sptext,"LEDs are %s", LEDsAreOff?"OFF":"ON" );
                if(LEDsAreOff) { LedsOff(); ShowLeds();}                                      // Turn the LEDs off
                else {DimLeds(true); Displaytime();}                                          // Turn the LEDs on                
               }
                                                                  break;                                                                   
    case 'P':
    case 'p':  
             if (InputString.length() == 9 )
               {

                LetterColor = Mem.OwnColour = HexToDec(InputString.substring(1,9));           // Display letter color 
                sprintf(sptext,"Own colour stored0X%lX", Mem.OwnColour);
                Displaytime();
               }
             else Tekstprintln("Enter Pwwrrggbb");            
             break;
    case 'q':
    case 'Q':  
             if (InputString.length() == 1 )
               {
             Tekstprintln("  Q0= Yellow colour, HETISWAS changing");
             Tekstprintln("  Q1= Hourly colour");
             Tekstprintln("  Q2= All white");
             Tekstprintln("  Q3= All Own colour");
             Tekstprintln("  Q4= Own colour, HETISWAS changing");
             Tekstprintln("  Q5= Wheel colour");
             Tekstprintln("  Q6= Digital display");
               }
             if (InputString.length() == 2 )
               {
                 Mem.DisplayChoice = (byte) InputString.substring(1,2).toInt(); 
                sprintf(sptext,"Display choice: Q%d", Mem.DisplayChoice);
                lastminute = 99;                                                              // Force a minute update
               }
             Displaytime();                                                                   // Turn on the LEDs with proper time             
             break;     
//                                                                                            //
    case 'R':
    case 'r':
            if (InputString.length() == 1)
              {
               Reset();                                                                       // Reset all settings 
               Tekstprintln("\n**** Reset to default settings ****"); 
               strcpy(sptext,""); 
              }
            break;
    case 'S':
    case 's':
             if (InputString.length() == 1)
               {   
                Zelftest = 1 - Zelftest; 
                sprintf(sptext,"Zelftest: %d",Zelftest);
                Displaytime();                                                               // Turn on the LEDs with proper time
               }                                
             break; 
    case 'T':
    case 't':
            if(InputString.length() >= 7)  // T125500
              {              
              I.Hour   = (byte) SConstrainInt(InputString,1,3,0,23);
              I.Minute = (byte) SConstrainInt(InputString,3,5,0,59); 
              I.Second = (byte) SConstrainInt(InputString,5,7,0,59); 
              sprintf(sptext,"Time set");
              SetRTCTime();
                                  #ifdef DCFNOINT
              DCFlocked = false;                                                              // RTC time can be updated from DCF time  
              Tekstprintln("DCFlock unlocked in Thhmmss "); 
                                  #endif  //DCFNOINT
              }
              break; 
    case 'W':
    case 'w':
             if (InputString.length() >1) break;   
             TestLDR = 1 - TestLDR;                                                           // If TestLDR = 1 LDR reading is printed every second instead every 30s
             sprintf(sptext,"TestLDR: %s",TestLDR? "On" : "Off");
             break;        
    case 'X':
    case 'x':    
             MilliSecondValue = 1000;                                                         // Clock runs at normal speed minus 1 ms 
              if (InputString.length() >1 && InputString.length() < 6 )
                MilliSecondValue = InputString.substring(1,5).toInt();                
             Demo = 1 - Demo;                                                                 // Toggle Demo mode
             sprintf(sptext,"Demo mode: %d MillisecondTime=%d",Demo,MilliSecondValue);
             break;        
     default:
             break;
    }
  }
 else if (InputString.length() == 6 )                                                         // This part is used for compatibility
  {
   I.Hour   = (byte) SConstrainInt(InputString,0,2,0,23);
   I.Minute = (byte) SConstrainInt(InputString,2,4,0,59); 
   I.Second = (byte) SConstrainInt(InputString,4,6,0,59);
   sprintf(sptext,"Time set");  
   SetRTCTime();
                      #ifdef DCFNOINT
 //  Tekstprintln("DCFlock unlocked in hhmmss "); 
   DCFlocked = false;                                                                         // RTC time can be updated from DCF time  Tekstprintln("DCFlock is unlocked ");
                    #endif  //DCFNOINT
  }
 EEPROM.put(0,Mem);                                                                           // Update EEPROM  
 Tekstprintln(sptext);                                                
 InputString = "";
}
//                                                                                            //
//------------------------------------------------------------------------------
//  CLOCK Convert Hex to uint32
//------------------------------------------------------------------------------
uint32_t HexToDec(String hexString) 
{
 uint32_t decValue = 0;
 int nextInt;
 for (unsigned int i = 0; i < hexString.length(); i++) 
  {
   nextInt = int(hexString.charAt(i));
   if (nextInt >= 48 && nextInt <= 57)  nextInt = map(nextInt, 48, 57, 0, 9);
   if (nextInt >= 65 && nextInt <= 70)  nextInt = map(nextInt, 65, 70, 10, 15);
   if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
   nextInt = min(nextInt, 15);
   decValue = (decValue * 16) + nextInt;
  }
return decValue;
}

//------------------------------------------------------------------------------
//  CLOCK Dutch clock display
//------------------------------------------------------------------------------
void Dutch(void)
{
                       #ifdef NL
 if (I.Hour == 12 && I.Minute == 0 && random(10)==0) { HET; IS; NOEN; return; }
 if (I.Hour == 00 && I.Minute == 0 && random(5)==0) { HET; IS; MIDDER; NACHT; return; } 

 HET;                                       // HET light is always on
 switch (I.Minute)
 {
  case  0: IS;  PRECIES; break;
  case  1: IS;  break;
  case  2: 
  case  3: WAS; break;
  case  4: 
  case  5: 
  case  6: IS;  MVIJF; OVER; break;
  case  7: 
  case  8: WAS; MVIJF; OVER; break;
  case  9: 
  case 10: 
  case 11: IS;  MTIEN; OVER; break;
  case 12: 
  case 13: WAS; MTIEN; OVER; break;
  case 14: 
  case 15: 
  case 16: IS;  KWART; OVER; break;
  case 17: 
  case 18: WAS; KWART; OVER; break;
  case 19: 
  case 20: 
  case 21: IS;  MTIEN; VOOR; HALF; break;
  case 22: 
  case 23: WAS; MTIEN; VOOR; HALF; break;
  case 24: 
  case 25: 
  case 26: IS;  MVIJF; VOOR; HALF; break;
  case 27: 
  case 28: WAS; MVIJF; VOOR; HALF; break;
  case 29: IS;  HALF; break;
  case 30: IS;  PRECIES; HALF; break;
  case 31: IS;  HALF; break;
  case 32: 
  case 33: WAS; HALF; break;
  case 34: 
  case 35: 
  case 36: IS;  MVIJF; OVER; HALF; break;
  case 37: 
  case 38: WAS; MVIJF; OVER; HALF; break;
  case 39: 
  case 40: 
  case 41: IS;  MTIEN; OVER; HALF; break;
  case 42: 
  case 43: WAS; MTIEN; OVER; HALF; break;
  case 44: 
  case 45: 
  case 46: IS;  KWART; VOOR; break;
  case 47: 
  case 48: WAS; KWART; VOOR; break;
  case 49: 
  case 50: 
  case 51: IS;  MTIEN; VOOR;  break;
  case 52: 
  case 53: WAS; MTIEN; VOOR;  break;
  case 54: 
  case 55: 
  case 56: IS;  MVIJF; VOOR; break;
  case 57: 
  case 58: WAS; MVIJF; VOOR; break;
  case 59: IS;  break;
}
//if (I.Hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);
//                                                                                            //
 sayhour = I.Hour;
 if (I.Minute > 18 )  sayhour = I.Hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1: EEN; break;
  case 14:
  case 2: TWEE; break;
  case 15:
  case 3: DRIE; break;
  case 16:
  case 4: VIER; break;
  case 17:
  case 5: VIJF; break;
  case 18:
  case 6: ZES; break;
  case 19:
  case 7: ZEVEN; break;
  case 20:
  case 8: ACHT; break;
  case 21:
  case 9: NEGEN; break;
  case 22:
  case 10: TIEN; break;
  case 23:
  case 11: ELF; break;
  case 0:
  case 12: TWAALF; break;
 } 
 switch (I.Minute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }
}
                      #endif  //NL
                      #ifdef UK
//------------------------------------------------------------------------------
//  CLOCK English clock display
//------------------------------------------------------------------------------
void English(void)
{
 IT;                                       // HET light is always on
 switch (I.Minute)
 {
  case  0: ISUK;  EXACTUK; break;
  case  1: ISUK;  break;
  case  2: 
  case  3: WASUK; break;
  case  4: 
  case  5: 
  case  6: ISUK;  MFIVE; PAST; break;
  case  7: 
  case  8: WASUK; MFIVE; PAST; break;
  case  9: 
  case 10: 
  case 11: ISUK;  MTEN; PAST; break;
  case 12: 
  case 13: WASUK; MTEN; PAST; break;
  case 14: 
  case 15: 
  case 16: ISUK;  QUARTER; PAST; break;
  case 17: 
  case 18: WASUK; QUARTER; PAST; break;
  case 19: 
  case 20: 
  case 21: ISUK;  TWENTY; PAST; break;
  case 22: 
  case 23: WASUK; TWENTY; PAST; break;
  case 24: 
  case 25: 
  case 26: ISUK;  TWENTY; MFIVE; PAST; break;
  case 27: 
  case 28: WASUK; TWENTY; MFIVE; PAST; break;
  case 29: ISUK;  HALFUK; PAST; break;
  case 30: ISUK;  EXACTUK; HALFUK; PAST; break;
  case 31: ISUK;  HALFUK; PAST; break;
  case 32: 
  case 33: WASUK; HALFUK; PAST; break;
  case 34: 
  case 35: 
  case 36: ISUK;  TWENTY; MFIVE; TO; break;
  case 37: 
  case 38: WASUK; TWENTY; MFIVE; TO; break;
  case 39: 
  case 40: 
  case 41: ISUK;  TWENTY; TO; break;
  case 42: 
  case 43: WASUK; TWENTY; TO break;
  case 44: 
  case 45: 
  case 46: ISUK;  QUARTER; TO; break;
  case 47: 
  case 48: WASUK; QUARTER; TO; break;
  case 49: 
  case 50: 
  case 51: ISUK;  MTEN; TO;  break;
  case 52: 
  case 53: WASUK; MTEN; TO;  break;
  case 54: 
  case 55: 
  case 56: ISUK;  MFIVE; TO; break;
  case 57: 
  case 58: WASUK; MFIVE; TO; break;
  case 59: ISUK;  break;
}
//if (I.Hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = I.Hour;
 if (I.Minute > 33 ) sayhour = I.Hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1:  ONE; break;
  case 14:
  case 2:  TWO; break;
  case 15:
  case 3:  THREE; break;
  case 16:
  case 4:  FOUR; break;
  case 17:
  case 5:  FIVE; break;
  case 18:
  case 6:  SIXUK; break;
  case 19:
  case 7:  SEVEN; break;
  case 20:
  case 8:  EIGHT; break;
  case 21:
  case 9:  NINE; break;
  case 22:
  case 10: TEN; break;
  case 23:
  case 11: ELEVEN; break;
  case 0:
  case 12: TWELVE; break;
 } 
 switch (I.Minute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: OCLOCK;  break; 
 }
}
                           #endif  //UK
                           #ifdef DE
//------------------------------------------------------------------------------
//  CLOCK German clock display
//------------------------------------------------------------------------------
void German(void)
{
  ES;                                       // HET light is always on
 switch (I.Minute)
 {
  case  0: IST;  GENAU; break;
  case  1: IST; KURZ; NACH; break;
  case  2: 
  case  3: WAR; break;
  case  4: 
  case  5: 
  case  6: IST; MFUNF; NACH; break;
  case  7: 
  case  8: WAR; MFUNF; NACH; break;
  case  9: 
  case 10: 
  case 11: IST; MZEHN; NACH; break;
  case 12: 
  case 13: WAR; MZEHN; NACH; break;
  case 14: 
  case 15: 
  case 16: IST; VIERTEL; NACH; break;
  case 17: 
  case 18: WAR; VIERTEL; NACH; break;
  case 19: 
  case 20: 
  case 21: IST; MZEHN; VOR; HALB; break;
  case 22: 
  case 23: WAR; MZEHN; VOR; HALB; break;
  case 24: 
  case 25: 
  case 26: IST; MFUNF; VOR; HALB; break;
  case 27: 
  case 28: WAR; MFUNF; VOR; HALB; break;
  case 29: IST; KURZ;  VOR; HALB; break;
  case 30: IST; GENAU; HALB; break;
  case 31: IST; KURZ;  NACH; HALB; break;
  case 32: 
  case 33: WAR; HALB; break;
  case 34: 
  case 35: 
  case 36: IST; MFUNF; NACH; HALB; break;
  case 37: 
  case 38: WAR; MFUNF; NACH; HALB; break;
  case 39: 
  case 40: 
  case 41: IST; MZEHN; NACH; HALB; break;
  case 42: 
  case 43: WAR; MZEHN; NACH; HALB; break;
  case 44: 
  case 45: 
  case 46: IST; VIERTEL; VOR; break;
  case 47: 
  case 48: WAR; VIERTEL; VOR; break;
  case 49: 
  case 50: 
  case 51: IST; MZEHN; VOR;  break;
  case 52: 
  case 53: WAR; MZEHN; VOR;  break;
  case 54: 
  case 55: 
  case 56: IST; MFUNF; VOR; break;
  case 57: 
  case 58: WAR; MFUNF; VOR; break;
  case 59: IST;  break;
}
//if (I.Hour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = I.Hour;
 if (I.Minute > 18 ) sayhour = I.Hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1: EINS; break;
  case 14:
  case 2: ZWEI; break;
  case 15:
  case 3: DREI; break;
  case 16:
  case 4: VIERDE; break;
  case 17:
  case 5: FUNF; break;
  case 18:
  case 6: SECHS; break;
  case 19:
  case 7: SIEBEN; break;
  case 20:
  case 8: ACHTDE; break;
  case 21:
  case 9: NEUN; break;
  case 22:
  case 10: ZEHN; break;
  case 23:
  case 11: ELFDE; break;
  case 0:
  case 12: ZWOLF; break;
 } 
 switch (I.Minute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UHR;  break; 
 }
}
                      #endif  //DE
                      #ifdef FR
//------------------------------------------------------------------------------
//  CLOCK French clock display
//------------------------------------------------------------------------------
void French(void)
{
 IL;                                       // HET light is always on
 switch (I.Minute)
 {
  case  0: EST;   EXACT; DITLEHEURE; break;
  case  1: EST;   DITLEHEURE; break;
  case  2: 
  case  3: ETAIT; DITLEHEURE; break;
  case  4: 
  case  5: 
  case  6: EST;   DITLEHEURE; MCINQ; break;
  case  7: 
  case  8: ETAIT; DITLEHEURE; MCINQ; break;
  case  9: 
  case 10: 
  case 11: EST;   DITLEHEURE; MDIX;  break;
  case 12: 
  case 13: ETAIT; DITLEHEURE; MDIX;  break;
  case 14: 
  case 15: 
  case 16: EST;   DITLEHEURE; ET; QUART; break;
  case 17: 
  case 18: ETAIT; DITLEHEURE; ET; QUART; break;
  case 19: 
  case 20: 
  case 21: EST;   DITLEHEURE; VINGT; break;
  case 22: 
  case 23: ETAIT; DITLEHEURE; VINGT; break;
  case 24: 
  case 25: 
  case 26: EST;   DITLEHEURE; VINGT; MCINQ; break;
  case 27: 
  case 28: ETAIT; DITLEHEURE; VINGT; MCINQ; break;
  case 29: EST;   DITLEHEURE; ET; DEMI; break;
  case 30: EST;   EXACT; DITLEHEURE;  ET; DEMI; break;
  case 31: EST;   DITLEHEURE; ET; DEMI; break;
  case 32: 
  case 33: ETAIT; DITLEHEURE; ET; DEMI; break;
  case 34: 
  case 35: 
  case 36: EST;   DITLEHEURE; MOINS; VINGT; MCINQ; break;
  case 37: 
  case 38: ETAIT; DITLEHEURE; MOINS; VINGT; MCINQ; break;
  case 39: 
  case 40: 
  case 41: EST;   DITLEHEURE; MOINS; VINGT;  break;
  case 42: 
  case 43: ETAIT; DITLEHEURE; MOINS; VINGT;  break;
  case 44: 
  case 45: 
  case 46: EST;   DITLEHEURE; MOINS; LE; QUART; break;
  case 47: 
  case 48: ETAIT; DITLEHEURE; MOINS; LE; QUART; break;
  case 49: 
  case 50: 
  case 51: EST;   DITLEHEURE; MOINS; MDIX;   break;
  case 52: 
  case 53: ETAIT; DITLEHEURE; MOINS; MDIX;   break;
  case 54: 
  case 55: 
  case 56: EST;   DITLEHEURE; MOINS; MCINQ;  break;
  case 57: 
  case 58: ETAIT; DITLEHEURE; MOINS; MCINQ;  break;
  case 59: EST;   DITLEHEURE;  break;
 }
}

void DitLeHeure(void)
{
 byte sayhour = I.Hour;
 if (I.Minute > 33 ) sayhour = I.Hour+1;
 if (sayhour == 24) sayhour = 0;

switch (sayhour)
 {
  case 13:  
  case 1:  UNE;    HEURE;  break;
  case 14:
  case 2:  DEUX;   HEURES;  break;
  case 15:
  case 3:  TROIS;  HEURES;  break;
  case 16:
  case 4:  QUATRE; HEURES; break;
  case 17:
  case 5:  CINQ;   HEURES;   break;
  case 18:
  case 6:  SIX;    HEURES;   break;
  case 19:
  case 7:  SEPT;   HEURES;  break;
  case 20:
  case 8:  HUIT;   HEURES; break;
  case 21:
  case 9:  NEUF;   HEURES; break;
  case 22:
  case 10: DIX;    HEURES; break;
  case 23:
  case 11: ONZE;   HEURES; break;
  case 0:  MINUIT; break;
  case 12: MIDI;   break;
 } 
}
                      #endif  //FR


                      #ifdef ARDUINO_SAMD_MKRWIFI1010
//------------------------------------------------------------------------------
// MKR1010  ARDUINO_SAMD_MKRWIFI1010
//------------------------------------------------------------------------------ 
//                                                                                            //
void Setup_MKRWIFI1010()
{                                              
  pinMode(led, OUTPUT);                        // set the LED pin mode
  if (WiFi.status() == WL_NO_MODULE)           // check for the WiFi module:
    {                                          // don't continue
    Serial.println("Communication with WiFi module failed!");
    while (true);
    }
  String fv = WiFi.firmwareVersion();
  if (fv < "1.0.0") { Serial.println("Please upgrade the firmware");  }
                                               // by default the local IP address of will be 192.168.4.1
                                               // you can override it with the following: WiFi.config(IPAddress(10, 0, 0, 1));
  Serial.print("Creating access point named: ");   // print the network name (SSID);
  Serial.println(ssid);
                                               // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING)               // don't continue
    {
    Serial.println("Creating access point failed");
    while (true);
    } 
  delay(10000);                                // wait 10 seconds for connection:
  server.begin();                              // start the web server on port 80  
  printWiFiStatus();                           // you're connected now, so print out the status
}
//------------------------------------------------------------------------------
// SERVER Update routine done every second
//------------------------------------------------------------------------------
//                                                                                            //
void WIFIServercheck(void)     
{
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }
 
  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("Click <a href=\"/H\">here</a> turn the LED on<br>");
            client.print("Click <a href=\"/L\">here</a> turn the LED off<br>");

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          }
          else {      // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r') {    // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /H")) {
          digitalWrite(led, HIGH);               // GET /H turns the LED on
        }
        if (currentLine.endsWith("GET /L")) {
          digitalWrite(led, LOW);                // GET /L turns the LED off
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }                     
}
//                                                                                            //
void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);

}
                          #endif //ARDUINO_SAMD_MKRWIFI1010

//
//------------------------------------------------------------------------------
// ARDUINO Reset function
//------------------------------------------------------------------------------
void SoftwareReset( uint8_t prescaler) 
{
  prescaler=prescaler;
// wdt_enable( prescaler);    // start watchdog with the provided prescaler
 //while(1) {}
}
//------------------------------------------------------------------------------
// ARDUINO Reset to default settings
//------------------------------------------------------------------------------
void ResetArduino(void)
{
//SoftwareReset( WDTO_60MS); //WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, WDTO_1S, WDTO_2S, WDTO_4S, and WDTO_8S
}
