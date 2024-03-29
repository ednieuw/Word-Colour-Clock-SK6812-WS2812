// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 (Nano) ATMEGA1284 chip and Arduino MKR1010 that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05 and HM10 BLE
- DCF77 module DCF-2
- LCD
- FM-radio
- WIFI on MKR 1010 to get NTP time

Load and install in IDE:
http://arduino.esp8266.com/stable/package_esp8266com_index.json  (ESP-12)
https://mcudude.github.io/MightyCore/package_MCUdude_MightyCore_index.json ATMEGA1260 and 644
https://raw.githubusercontent.com/damellis/attiny/ide-1.6.x-boards-manager/package_damellis_attiny_index.json ATTINY
Arduino SAMD for MKR1010

The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module
The FM-module can be used to read the RDS-time from radio station or to play FM-radio station from within the clock

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
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
//#define BWClock      // Defines Black and White Word Clock with 2835 / 2538 LEDs 
//--------->>             If ColourClock is defined also choose between LED type
#define ColourClock    // Defines colour Word Clock with WS2812 LEDs
#define LED2812
//#define LEDSK6812

//#define FMRADIOMOD    // in development time retrieval works. Needs automatic optimal sender search function
//#define BLUETOOTHMOD   // Kan niet uit bij ATMEGA1280 boards?
//#define DCFMOD
#define ROTARYMOD
//#define LCDMOD
//--------------------------------------------
// ARDUINO Definition of installed language word clock
//--------------------------------------------
 #define NL
// #define UK
// #define DE
// #define FR        
//--------------------------------------------
// ARDUINO Includes defines and initialisations
//--------------------------------------------
                     #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
#define ATmega644_1284   
                     #endif
#include <Wire.h>
#include "RTClib.h"
#include "TimeLib.h"
#include <SoftwareSerial.h>   // moet hier staan. geen idee waarom
#include <avr/pgmspace.h>
                     #if defined(ARDUINO_SAMD_MKRWIFI1010)
#include <FlashAsEEPROM.h>
#include <SPI.h>                  // for web server
#include <WiFiNINA.h>             // for web server
//#include "arduino_secrets.h"    // for web server
#define SECRET_SSID "PWDis_klok"
#define SECRET_PASS "klok"
                     #else if
#include <EEPROM.h>
                     #endif
                     #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                     #endif  LCDMOD  
                     #ifdef LED6812    
#include <Adafruit_NeoPixel.h>
                     #endif LED6812  
                     #ifdef LED2812
#include <WS2812.h>
                     #endif LED2812
                     #ifdef ATmega644_1284
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>     // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #endif 

                     #ifdef ROTARYMOD
#include <Encoder.h>
                     #ifdef ATmega644_1284
  #define CORE_NUM_INTERRUPT 3
  #define CORE_INT0_PIN   10
  #define CORE_INT1_PIN   11
  #define CORE_INT2_PIN   2
                     #endif
                     #endif ROTARYMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     #endif DCFMOD
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
#define EDSOFF  ColorLeds("EdSoff", 132,132, 0);
#define X_OFF   ColorLeds("",        90, 90, 0);
#define X_ON    ColorLeds("",        90, 90, LetterColor);

                     #endif NL
                     #ifdef UK
#define IT      ColorLeds("\nIt",   145, 145, MINColor);   
#define ISUK    ColorLeds("is",     145, 145, SECColor);    Is = true;
#define WASUK   ColorLeds("was",    145, 145, SECColor);    Is = false;
#define EXACT   ColorLeds("exact",  145, 145, LetterColor);
#define HALFUK  ColorLeds("half",   145, 145, LetterColor); 
#define TWENTY  ColorLeds("twenty", 145, 145, LetterColor); 
#define MFIVE   ColorLeds("five",   145, 145, LetterColor);
#define QUARTER ColorLeds("quarter",145, 145, LetterColor);
#define MTEN    ColorLeds("ten",    145, 145, LetterColor);
#define PAST    ColorLeds("past",   145, 145, LetterColor);
#define TO      ColorLeds("to",     145, 145, LetterColor);
#define SIX     ColorLeds("six",    145, 145, LetterColor);
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
                      #endif UK
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
                      #endif DE
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
#define DEMI    ColorLeds("demie",   145, 145, LetterColor);
#define QUART   ColorLeds("quart",  145, 145, LetterColor);
#define MDIX    ColorLeds("dix",    145, 145, LetterColor);
#define VINGT   ColorLeds("vingt",  145, 145, LetterColor);
#define MCINQ   ColorLeds("cinq",   145, 145, LetterColor);
#define DITLEHEURE DitLeHeure();
                      #endif FR
//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
             #if defined(__AVR_ATmega328P__) || defined(ARDUINO_SAMD_MKRWIFI1010)
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 328 ----
 BT_RX        = 0,                // Connects to Bluetooth TX
 BT_TX        = 1,                // Connects to Bluetooth RX
 DCF_PIN     =  2,                // DCFPulse on interrupt  pin
 encoderPinA  = 3,                // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 4,                // switch (labeled SW on decoder)
 LED_PIN      = 5,                // Pin to control colour SK6812/WS2812 LEDs
 PIN06        = 6,                // PIN 6
 PIN07        = 7,                // PIN 7 
 encoderPinB  = 8,                // left (labeled CLK on decoder)no interrupt pin  
 DCF_LED_Pin  = 9,                // define pin voor AM PM Led
 LEDDataPin   = 10,               // blauw HC595
 LEDStrobePin = 11,               // groen HC595
 LEDClockPin  = 12,               // geel  HC595
 secondsPin   = 13,
 HeartbeatLED = 13};
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 PhotoCellPin = 2,                // LDR pin
 EmptyA3      = 3,                //
 SDA_pin      = 4,                // SDA pin
 SCL_pin      = 5};               // SCL pin
                    #endif

                             #ifdef ATmega644_1284  
//--------------------------------------------//--------------------------------------------
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
 TX1          = 9,               // TX1           PD1 digital
 DCF_LED_Pin  = 10,                // LED10         PD2 digital
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
                     #endif ATmega644_1284
/*                                         
enum DigitalPinAssignments {      // Digital hardware constants ATMEGA 1284P ----
 BT_RX        = 0,                // Bluetooth RX Connects to Bluetooth TX
 BT_TX        = 1,                // Bluetooth TX Connects to Bluetooth RX
 DCF_PIN      = 2,                // DCFPulse on interrupt  pin
 PWMpin       = 3,                // Pin that controle PWM signal on BC327 transistor to dim light
 PIN04        = 4,                // Pin 4         PB4 PWM
 LED_PIN      = 5,                // Pin to control colour 2811/2812 leds PB5 digital
 PIN06        = 6,                // PIN 6         PB6 PWM 
 PIN07        = 7,                // PIN 7         PB7 PWM  
 PIN08        = 8,                // RX1           PD0 digital
 PIN09        = 9,                // TX1           PD1 digital
 LED10        = 10,               // LED10         PD2 digital
 encoderPinB  = 11,               // left (labeled CLK on decoder)no interrupt pin
 encoderPinA  = 12,               // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 13,               // switch (labeled SW on decoder)  
 DCF_LED_Pin  = 14,               // define pin voor AM PM Led
 HeartbeatLED = 15,               // LED15         PD7 PWM
 SCL_pin      = 16,               // SCL pin       PC0 interrupt
 SDA_pin      = 17,               // SDA pin       PC1 interrupt
 PIN18        = 18,               // Empty         PC2 digital
 LED19        = 19,               // LED19         PC3 digital
 LEDDataPin   = 20,               // blauw HC595
 LEDStrobePin = 21,               // groen HC595
 LEDClockPin  = 22,               // geel  HC595
 secondsPin   = 23};
                                  
enum AnaloguePinAssignments {     // Analogue hardware constants ----
  EmptyA0     = 24,               // Empty
  EmptyA1     = 25,               // Empty
  PhotoCellPin= 26,               // LDR pin  AI 2
  EmptyA3     = 27,               // Empty
  EmptyA4     = 28,               // Empty
  EmptyA5     = 29,               // Empty
  EmptyA6     = 30};              // Empty
 #endif 
*/
//--------------------------------------------
// LED
//--------------------------------------------
const byte NUM_LEDS      = 144;    // How many leds in  strip?
const byte MATRIX_WIDTH  = 12;
const byte MATRIX_HEIGHT = 12;
const byte BRIGHTNESS    = 32;    // BRIGHTNESS 0 - 255
                     #ifdef LED6812    
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                     #endif LED6812  
                     #ifdef LED2812
WS2812  strip(NUM_LEDS);           // Initialyse RGB strip
                     #endif LED2812
byte     BrightnessCalcFromLDR = BRIGHTNESS;    // Initial brightness value 
int      Previous_LDR_read = 512;
uint32_t MINColor      = 0X00FFDD00;
uint32_t SECColor      = 0X00FFDD00;
uint32_t LetterColor   = 0X00FFDD00;       
uint32_t DefaultColor  = 0X00FFDD00;       // Yellow
uint32_t OwnColour     = 0X002345DD;       // Blue
uint32_t WhiteColour   = 0XFF000000;
uint32_t WheelColor    = 0X000000FF;
uint32_t HourColor[24] ={0XFFFFFF,0X800080,0XFFD500,0X00B8FF,0XFF6200,0X80FF00,
                         0XFF8000,0X00C8FF,0X7500BC,0XFFD000,0X00FF00,0XFF0050,
                         0XFF0000,0XFFFF00,0XFF4000,0XFF0088,0XF7FF00,0XFF00BF,
                         0X00FF00,0X00F2A0,0X6FFF00,0X0073FF,0X70ff70,0X0000FF };     
// Definition of the digits 0 - 9 
const byte PROGMEM Getal[10][3][5]  = { 
                     { {1, 1, 1, 1, 1}, {1, 0, 0, 0, 1}, {1, 1, 1, 1, 1} },  //0
                     { {1, 0, 0, 0, 1}, {1, 1, 1, 1, 1}, {0, 0, 0, 0, 1} },  //1
                     { {1, 0, 1, 1, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 0, 1} },  //2
                     { {1, 0, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} },  //3
                     { {1, 1, 1, 0, 0}, {0, 0, 1, 0, 0}, {1, 1, 1, 1, 1} },  //4
                     { {1, 1, 1, 0, 0}, {1, 0, 1, 0, 1}, {1, 0, 1, 1, 1} },  //5
                     { {1, 1, 1, 1, 1}, {0, 0, 1, 0, 1}, {0, 0, 1, 1, 1} },  //6
                     { {1, 1, 0, 0, 0}, {1, 0, 0, 0, 0}, {1, 1, 1, 1, 1} },  //7
                     { {1, 1, 1, 1, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} },  //8
                     { {1, 1, 1, 0, 1}, {1, 0, 1, 0, 1}, {1, 1, 1, 1, 1} }   //9
                     };  
      
//--------------------------------------------
// KY-040 ROTARY
//-------------------------------------------- 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);              // Use digital pin  for encoder
                          #endif ROTARYMOD      
long     Looptime          = 0;
byte     RotaryPress       = 0;     // Keeps track displaychoice and how often the rotary is pressed.
uint32_t RotaryPressTimer  = 0;
byte     NoofRotaryPressed = 0;

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
byte     LightReducer    = 80;     // Factor (%) to dim LED intensity with. Between 1% and 250%
byte     LowerBrightness = 10;     // Lower limit of Brightness ( 0 - 255)  
int      OutPhotocell;             // stores reading of photocell;
int      MinPhotocell    = 1024;   // stores minimum reading of photocell;
int      MaxPhotocell    = 1;      // stores maximum reading of photocell;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 50
static  uint32_t msTick;                // the number of millisecond ticks since we last incremented the second counter
int     count; 
int     Delaytime = 200;
byte    Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte    lastminute = 0, lasthour = 0, sayhour = 0;
byte    ChangeTime           = false;
byte    ChangeLightIntensity = false;
byte    SecPulse             = 0;       // give a pulse to the Isecond led
byte    Demo                 = false;
byte    Zelftest             = false;
byte    Is                   = true;    // toggle of displaying Is or Was
byte    ZegUur               = true;    // Say or not Uur in NL clock

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
RTC_DS3231 rtc; //RTC_DS3231 RTC;    //RTC_DS1307 RTC;   
DateTime Inow;

//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
                           #ifdef BLUETOOTHMOD               // Bluetooth ---------------------
                           #ifdef ATmega644_1284 
SoftwareSerial Bluetooth(BT_RX, BT_TX);    // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
// SoftwareSerial mySerial(10, 11); // RX, TX
                           #endif
String  BluetoothString;
                           #endif BLUETOOTHMOD 
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
byte   DCF_signal = 50;             // is a proper time received?
bool   SeeDCFsignalInDisplay=false; // if ON then the display line HET IS WAS will show the DCF77-signal received
time_t tijd;
                    #ifdef DCFMOD 
                    #if defined(__AVR_ATmega328P__) 
#define DCF_INTERRUPT 0             // DCF Interrupt number associated with DCF_PIN
#endif
                    #ifdef ATmega644_1284
#define DCF_INTERRUPT 2             // DCF Interrupt number associated with DCF_PIN
                    #endif
const byte DCF_INVERTED = LOW;      // HIGH (HKW) or LOW (Reichelt). Some DCF modules invert the signal
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,DCF_INVERTED);
                    #endif DCFMOD 
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
//LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);// 0x27 is the I2C bus address for an unmodified backpack
LiquidCrystal_I2C lcd(0x3F,2,1,0,4,5,6,7);
                    #endif 
//--------------------------------------------
// Webserver
//--------------------------------------------
                     #if defined(ARDUINO_SAMD_MKRWIFI1010)
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;                 // your network key Index number (needed only for WEP)
int led =  LED_BUILTIN;
int status = WL_IDLE_STATUS;
WiFiServer server(80);
                     #endif

//----------------------------------------
// Common
//----------------------------------------
char sptext[MAXTEXT+2];                 // For common print use    
int  MilliSecondValue     = 1000;       // The duration of a second  minus 1 ms. Used in Demo mode
static uint32_t last_time = 0;          // Heartbeat defines
byte hbval                = 128;        // Heartbeat initial intensity
byte hbdelta              = 10;         // Determines how fast heartbeat is
//--------------------------------------------
// COLOURS
//--------------------------------------------   
const byte DEFAULTCOLOUR = 0;
const byte HOURLYCOLOUR  = 1;          
const byte WHITECOLOR    = 2;
const byte OWNCOLOUR     = 3;
const byte OWNHETISCLR   = 4;
const byte WHEELCOLOR    = 5;
const byte DIGITAL       = 6;
byte DisplayChoice = WHITECOLOR;       
byte ResetDisplayChoice = WHITECOLOR; 
//--------------------------------------------
// Menu
//--------------------------------------------  
const byte MenuItems = 13;              // sentences, rows, in menu
const char menu[MenuItems][MAXTEXT] PROGMEM = {
 "Character B/W-Colour Clock_No36",
 "Enter time as: \nhhmm (1321)or hhmmss (132145)",
 "D D15122017 is date 15 December 2017",
 "G DCF-signalinfo in display",
 "Lnn (L5) Min light intensity ( 1-255)",
 "Mnn (M90)Max light intensity (1%-250%)",
 "Pnnnnnn (P234F8A) own colour (n=0-F)",
 "Qn Display Choice (Q0-6)",
 "I for this info",
 "R reset to default settings",
 "Xnn (X50) Demo mode. ms delay (0-9999)",
 "Z for Self test",
 "Ed Nieuwenhuys  V043 Oktober 2019" };
//  -------------------------------------   End Definitions  ---------------------------------------

//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
SerialCheck();
if (Demo)          Demomode();
else if (Zelftest) Selftest(); 
else
 { 
                              #ifdef ATmega644_1284
  heartbeat();                // only heartbeat with ATMEGA1284
                              #endif
  EverySecondCheck();  
  EveryMinuteUpdate();
                              #ifdef FMRADIOMOD     
  FMradioCheck();  
                              #endif FMRADIOMOD
                              #ifdef BLUETOOTHMOD   
  BluetoothCheck(); 
                              #endif BLUETOOTHMOD
                              #ifdef DCFMOD         
  DCF77Check();
                              #endif DCFMOD

                              #ifdef ROTARYMOD      
  RotaryEncoderCheck(); 
                              #endif ROTARYMOD 
 }
}  
//--------------------------------------------
// ARDUINO Setup
//--------------------------------------------
void setup()
{                                        
 Serial.begin(9600);                                                // Setup the serial port to 9600 baud  
 Wire.begin();                                                      // Start communication with I2C / TWI devices                            
 pinMode(DCF_LED_Pin,  OUTPUT);                                     // For showing DCF-pulse or other purposes
 pinMode(secondsPin,   OUTPUT );  
 pinMode(HeartbeatLED, OUTPUT );      
 pinMode(PhotoCellPin, INPUT);
                          #ifdef BLUETOOTHMOD 
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.begin(9600); 
                          #else if
 Bluetooth.begin(9600);  
                          #endif 
 Tekstprintln("Bluetooth enabled");
                          #endif BLUETOOTHMOD
                          #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary enabled"); 
 myEnc.write(0);                                                    // Clear Rotary encode buffer
                          #endif ROTARYMOD 
 pinMode(DCF_PIN,      INPUT_PULLUP);

// FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalSMD5050 );
// FastLED.setTemperature(CoolWhiteFluorescent ); // LEDS.setBrightness(BRIGHTNESS); currentPalette = RainbowColors_p;
                          #ifdef LED2812
 strip.setOutput(LED_PIN);                                          // Set strip dat output to pin LED pin
 strip.sync();                                                      // Initialize all pixels to 'off'
 strip.setBrightness(BRIGHTNESS);
 Tekstprintln("WS2812 enabled");  
                          #endif LED2812
                          #ifdef LED6812
 strip.begin();
 strip.setBrightness(BRIGHTNESS); //BRIGHTNESS);
 ShowLeds();                                                        // Initialize all pixels to 'off' 
 Tekstprintln("SK6812 enabled");
                          #endif LED6812
                          #ifdef FMRADIOMOD 
 Setup_FMradio();                                                   // Start the FM-radio
 Tekstprintln("FM-radio enabled");
                          #endif FMRADIOMOD 
                          #ifdef DCFMOD
 DCF.Start();                                                       // Start the DCF-module
 Tekstprintln("DCF enabled");
                          #endif DCFMOD
 rtc.begin();                                                       // Start the rtc-module  
                          #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                         // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Tekstprintln("LCD enabled");
                          #endif LCDMOD
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Setup_MKRWIFI1010();                                               // Start the MKR1010
 Tekstprintln("MKR-1010 WIFI enabled");
                          #endif ARDUINO_SAMD_MKRWIFI1010
 DateTime now = rtc.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
   rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 if (EEPROMread(0) <3 || EEPROMread(0) > 200)  EEPROMwrite(0, LightReducer);     // Default intensity for this clock
 if (EEPROMread(1) <1 || EEPROMread(1) > 100)  EEPROMwrite(1, LowerBrightness);  // Default Lower Brightness for this clock
 if (EEPROMread(2) > DIGITAL)                  EEPROMwrite(2, DEFAULTCOLOUR);    // Default Display setting
 LightReducer  =   EEPROMread(0);                  // Store it is the work variable
 LowerBrightness = EEPROMread(1);                  // Store it is the work variable
 DisplayChoice   = EEPROMread(2);                  // Store it is the work variable
 OwnColour       = GetEEPROM_RGBW();               // Get the store RGBW value
 if (OwnColour == 0 ) OwnColour = 0X002345DD;      // If memory is empty then store default value, blue
 msTick = Looptime = millis();                     // Used in KY-040 rotary for debouncing and seconds check                                                     
// Selftest();                                     // Play the selftest
 GetTijd(0);                                       // Get the time and store it in the proper variables
 SWversion();                                      // Display the version number of the software
 
 Displaytime();
 } 

//--------------------------------------------
// ARDUINO Reset to default settings
//--------------------------------------------
void Reset(void)
{
 OwnColour            = 0X002345DD;  // Blue 
 LightReducer         = 80;        // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 EEPROMwrite(0, LightReducer);     // Default intensity for this clock
 LowerBrightness      = 10;          // Lower limit of Brightness ( 0 - 255)
 EEPROMwrite(1, LowerBrightness);    // Default Lower Brightness for this clock
 DisplayChoice        = ResetDisplayChoice; 
 EEPROMwrite(2, ResetDisplayChoice); // Default Display mode for the clock
 StoreEEPROM_RGBW(0X002345DD);       // Default own colour
 MinPhotocell         = 1024;        // stores minimum reading of photocell;
 MaxPhotocell         = 1;           // stores maximum reading of photocell;
 ChangeTime           = false;
 ChangeLightIntensity = false;
 Demo                 = false;
 Zelftest             = false;
 Is                   = true;         // toggle of displaying Is or Was
 ZegUur               = true;         // Say or not Uur in NL clock
 Selftest();                          // Play the selftest
 GetTijd(0);                          // Get the time and store it in the proper variables
 SWversion();                         // Display the version number of the software
 Displaytime();
}
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<39; n++) {Serial.print(F("_"));} Serial.println();
 for (int i = 0; i < MenuItems; i++)   {strcpy_P(sptext, menu[i]);   Tekstprintln(sptext);  }
 for (int n=0; n<39; n++) {Serial.print(F("_"));} Serial.println();
 sprintf(sptext,"Brightness Min: %3d bits  Max: %3d%%",LowerBrightness, LightReducer) ;
 Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min: %3d bits  Max: %3d bits",MinPhotocell, MaxPhotocell);
 Tekstprintln(sptext); 
 sprintf(sptext,"Display choice: %3d",DisplayChoice);
 Tekstprintln(sptext);
 for (int n = 0; n < 39; n++) {Serial.print(F("_"));} Serial.println();
}

//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
 if ( millis() - msTick > 10)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
 if ( millis() - msTick >= MilliSecondValue)                        // Flash the onboard Pin 13 Led so we know something is happening
 {    
  msTick = millis();                                                // second++; 
  digitalWrite(secondsPin,HIGH);                                    // turn ON the second on pin 13
  ++SecPulse;                                                       // second routine in function DimLeds
  Isecond = 60-Iminute;
  if( ++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
  if( Ihour >12) 
   {    
    Ihour = 9; 
    if (++DisplayChoice > DIGITAL) DisplayChoice = 0;
   }
  DimLeds(false);
  Displaytime();
  Tekstprintln("");
 }
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
                          #ifdef ATmega644_1284
 Bluetooth.print(tekst);  
                          #endif
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.print(tekst);  
                          #endif
                          #endif BLUETOOTHMOD
}
void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
                          #ifdef ATmega644_1284
 Bluetooth.println(tekst);  
                          #endif
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
 Serial1.println(tekst);  
                          #endif
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
if (millis() - msTick > 10) digitalWrite(secondsPin,LOW);  // Turn OFF the second on SecondsPin
if (millis() - msTick > 999)                          // Flash the onboard Pin 13 Led so we know something is happening
   {    
    msTick = millis();                                 // second++; 
    digitalWrite(secondsPin,HIGH);                     // turn ON the second on pin 13
    ++SecPulse;                                        // second routine in function DimLeds 
                    #ifdef LCDMOD
   Print_tijd_LCD();
                    #endif LCDMOD  
   GetTijd(0);         
   if(Isecond % 30 == 0)  DimLeds(true);               // Text LED intensity control + seconds tick print every 30 seconds   
   else                   DimLeds(false);              // every second an intensitiy check and update from LDR reading
   SetSecondColour();                                  // Sets the colour per second of 'IS' and 'WAS' 
   SetMinuteColour();                                  // to be sure all colours are OK
/*   if(Iminute == 0 && Isecond <9)
    {                                                  // 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light 
     if (Isecond % 2) {EDSOFT;} else {EDSOFF;}    }*/
                          #if defined(ARDUINO_SAMD_MKRWIFI1010)
   WIFIServercheck(); 
                          #endif
  }
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                            // Show time every minute
  { 
   lastminute = Iminute;
   Displaytime();
   Print_RTC_tijd();
   DCF_signal--;
   DCF_signal = constrain( DCF_signal,1,99);
  } 
 if (Ihour != lasthour) {lasthour = Ihour;}
 }

//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 String  SerialString; 
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;               // allow input from Space - Del
   else c = 0;                                         // delete a CR
  }
 if (SerialString.length()>0) 
         ReworkInputString(SerialString);              // Rework ReworkInputString();
 SerialString = "";

                          #ifdef BLUETOOTHMOD   
 BluetoothCheck(); 
                          #endif BLUETOOTHMOD
}                         
                          #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 long looptimeBT = millis();  //avoid an hangup in this loop  
                           #if defined(ARDUINO_SAMD_MKRWIFI1010)
 while (Serial1.available() and (millis() - looptimeBT < 10000) )
                           #else if
 while (Bluetooth.available() and (millis() - looptimeBT < 10000) )
                           #endif
  {
   delay(3); 
                           #if defined(ARDUINO_SAMD_MKRWIFI1010)
char c = Serial1.read();
                           #else if
char c = Bluetooth.read();
                           #endif   
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0)  
      ReworkInputString(BluetoothString);       // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD  
                           
                           #ifdef DCFMOD
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 time_t DCFtime = DCF.getTime();                       // Check if new DCF77 time is available
 time_t RTCtime = now();
 if (DCFtime!=0)
  {
   Tekstprintln("DCF: Time received ->  ");
   DCF_signal += 2;
   setTime(DCFtime); 
   if(abs( DCFtime - RTCtime) > 5)                     // if DCF time differs more than 5 secons from RTC time them adjust    
     {  
      RTC.adjust(DCFtime);        
      Tekstprint("DCF: RTC time is updated ----->  ");     
     }
  }
 bool LHbit = digitalRead(DCF_PIN) - (1 - DCF_INVERTED);// subtract DCF_INVERTED (= HIGH or LOW) from signal to invert it
 digitalWrite(DCF_LED_Pin, LHbit);                      // write DCF pulse to LED on board 
 if (SeeDCFsignalInDisplay)
  {
   if(LHbit){ X_OFF;}                       
   else     { X_ON; }
   ShowLeds();
  }
  DCF_signal = constrain(DCF_signal,0,99);             // DCF_signal <100
} 
                           #endif DCFMOD 
//--------------------------------------------
// CLOCK Heart beat in LED
//--------------------------------------------
void heartbeat() 
{
 unsigned long now = millis();
 if ((now - last_time) < 40)    return;
 last_time = now;
 if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
 hbval += hbdelta;
 analogWrite(HeartbeatLED, hbval);
}
                        #ifdef ROTARYMOD
//--------------------- KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
void RotaryEncoderCheck(void)
{
 long encoderPos = myEnc.read();
 if ( (unsigned long) (millis() - RotaryPressTimer) > 60000)        // After 60 sec after shaft is pressed time of light intensity can not be changed 
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
    if ( encoderPos && ( (millis() - Looptime) > 50))              // If rotary turned avoid debounce within 0.05 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                             // Increase  MINUTES of light intensity
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }        // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              {
               if( ++Ihour >23) { Ihour = 0; }
              }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              { 
               Isecond = 0;
               if( ++Iminute >59) { Iminute = 0; if( ++Ihour >23) { Ihour = 0; } }   
              }
           } 
        }    
      if (encoderPos <0)                                            // Increase the HOURS
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                              // Change hours
            {
             if( Ihour-- ==0) { Ihour = 23; }
            }      
           if (NoofRotaryPressed == 2)                              // Change minutes
            { 
             Isecond = 0;
             if( Iminute-- == 0) { Iminute = 59; if( Ihour-- == 0) { Ihour = 23; } }   
            }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      myEnc.write(0);                                               // Set encoder pos back to 0
      Looptime = millis();       
     }                                                
   }
 if (digitalRead(clearButton) == LOW )                              // Set the time by pressing rotary button
   { 
    delay(200);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    SeeDCFsignalInDisplay = false;                                  // Shows the SCG signal in the display
    RotaryPressTimer      = millis();                               // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >10 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                      // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;             BlinkUUR(3, 20);      break; // Change the hours
       case 2:  ChangeTime = true;             BlinkHETISWAS(3, 20); break; // Change the hours        
       case 3:  ChangeLightIntensity = true;   BlinkLetters(5, 10);  break; // Turn on all LEDs and change intensity 
       case 4:  DisplayChoice = DEFAULTCOLOUR;                       break;
       case 5:  DisplayChoice = HOURLYCOLOUR;                        break;        
       case 6:  DisplayChoice = WHITECOLOR;                          break;
       case 7:  DisplayChoice = OWNCOLOUR;                           break;
       case 8:  DisplayChoice = OWNHETISCLR;                         break;
       case 9:  DisplayChoice = WHEELCOLOR;                          break;    
       case 10: DisplayChoice = DIGITAL;                             break;                     
                     #ifdef DCFMOD
       case 11:  SeeDCFsignalInDisplay = true;                       break; // Show DCF signal   
                     #endif DCFMOD
       default: NoofRotaryPressed = 0; 
                SeeDCFsignalInDisplay = ChangeTime = ChangeLightIntensity  = false;  
                Selftest();        
                break;                         
      }
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed);   
    myEnc.write(0);
    Looptime = millis();     
    Displaytime();  
   }
  myEnc.write(0);
 }
//--------------------------------------------
//  Blink UUR
//--------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); UUR; Laatzien(); delay(Delayms);} 
}
//--------------------------------------------
//  Blink HET IS WAS
//--------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; Laatzien(); delay(Delayms);} 
}
//--------------------------------------------
//  Blink Letters
//--------------------------------------------
void BlinkLetters (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); ColorLeds("",  36, 59, 0XFF0000FF);  Laatzien(); delay(Delayms);} 
}
                          #endif ROTARYMOD

//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{ 
 //  Serial.print(F("Self test in : "));
 // start by clearing the display to a known state
  GetTijd(1); //Prints time in Serial monitor
  LedsOff(); 
  HET;   Laatzien(); IS;    Laatzien(); WAS;    Laatzien(); PRECIES; Laatzien(); MTIEN;  Laatzien();  MVIJF; Laatzien();    
  KWART; Laatzien(); VOOR;  Laatzien(); OVER;   Laatzien(); HALF;    Laatzien(); MIDDER; Laatzien(); VIJF;   Laatzien();
  TWEE;  Laatzien(); EEN;   Laatzien(); VIER;   Laatzien(); TIEN;    Laatzien(); TWAALF; Laatzien(); DRIE;   Laatzien();
  NEGEN; Laatzien(); NACHT; Laatzien(); ACHT;   Laatzien(); ZES;     Laatzien(); ZEVEN;  Laatzien();  ELF;   Laatzien(); 
  NOEN;  Laatzien(); UUR;   Laatzien(); EDSOFT; Laatzien();
  Tekstprintln("*");    
  Play_Lights();    
//  Displaytime();
}
// -------------------------- End Selftest

//--------------------------- Time functions --------------------------
void Displaytime(void)
{   
 SetMinuteColour();                         // Set the colour per minute of 'HET'
 SetSecondColour();                         // Set the colour per second of 'IS' and 'WAS'
 LedsOff();                                 // start by clearing the display to a known state   
 if (DisplayChoice == DIGITAL ) { TimePlaceDigit(Ihour,Iminute); }
 else
   {
                     #ifdef NL
    Dutch();
                     #endif NL
                     #ifdef UK
    English();
                     #endif UK
                     #ifdef DE
    German();
                     #endif DE
                     #ifdef FR
    French();
                     #endif FR
   }
// Tekstprintln(""); 
 ShowLeds();
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    rtc.now();
 Ihour =   Inow.hour();
 Iminute = Inow.minute();
 Isecond = Inow.second();
// if (Ihour > 24) { Ihour = random(12)+1; Iminute = random(60)+1; Isecond = 30;}  // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}

//--------------------------------------------
// DS3231 utility function prints time to serial
//--------------------------------------------
void Print_RTC_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Inow.day(),Inow.month(),Inow.year());
 Tekstprintln(sptext);
}
                    #ifdef LCDMOD
//--------------------------------------------
// CLOCK Print time to LCD display
//--------------------------------------------
void Print_tijd_LCD(void)
{
 lcd.home (); // set cursor to 0,0
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Inow.hour(),Inow.minute(),Inow.second());   lcd.print(sptext);
 sprintf(sptext," LDR%d   ",analogRead(PhotoCellPin));                          lcd.print(sptext);
 lcd.setCursor (0,1);        // go to start of 2nd line
 sprintf(sptext,"%0.2d-%0.2d-%0.4d",Inow.day(),Inow.month(),Inow.year());       lcd.print(sptext);
 sprintf(sptext," DCF%d   ",DCF_signal);                                        lcd.print(sptext);
}
                    #endif LCDMOD
//--------------------------------------------
// CLOCK utility function prints time to serial
//--------------------------------------------
void Print_tijd(void)
{
 sprintf(sptext,"%0.2d:%0.2d:%0.2d",Ihour,Iminute,Isecond);
 Tekstprintln(sptext);
}

//--------------------------------------------
// DS3231 Set time in module and print it
//--------------------------------------------
void SetRTCTime(void)
{ 
 Ihour   = constrain(Ihour  , 0,24);
 Iminute = constrain(Iminute, 0,59); 
 Isecond = constrain(Isecond, 0,59); 
 rtc.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                                      // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int Get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
 Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
 Wire.write(0x11);
 Wire.endTransmission();
 Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
 if(Wire.available()) 
  {
    tMSB = Wire.read();                          // 2's complement int portion
    tLSB = Wire.read();                          // fraction portion 
    temp3231 = (tMSB & B01111111);               // do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    // only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
 else {  temp3231 = -273; }   
 return (temp3231);
}
// ------------------- End  Time functions 

// --------------------Colour Clock Light functions -----------------------------------
//--------------------------------------------
//  LED Set color for LED
//--------------------------------------------
void ColorLeds(char* Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
                          #ifdef LED6812    
 for (int n = FirstLed; n <= LastLed; n++) strip.setPixelColor(n,RGBWColor);
                          #endif LED6812  
                          #ifdef LED2812
 for (int n = FirstLed; n <= LastLed; n++) strip.setRGB(n,RGBWColor);
                          #endif LED2812 
                          
 //  Serial.println(RGBWColor,HEX);  
 if (strlen(Tekst) > 0 )
 {
  sprintf(sptext,"%s ",Tekst);                  // if (SeeDCFsignalInDisplay == false) 
  Tekstprint(sptext);                           // Print Het, is, was, vijf, etc from TimeText string    
 }
}

///--------------------------------------------
//  LED Set colour for LED
//--------------------------------------------
void ColourLeds(byte Itemnum, int FirstLed, int LastLed, uint32_t RGBWColor)
{
 if(Itemnum < 750 && strlen(sptext) > 0)        // if Itemnum > 750 then it means no printing of text
  { 
   strcat(sptext," ");     
   if (SeeDCFsignalInDisplay == false)   
          Tekstprint(sptext);                    // Print Het, is, was, vijf, etc from TimeText string         
  }
                          #ifdef LED6812    
 for (int n = FirstLed; n <= LastLed; n++)  strip.setPixelColor(n,RGBWColor);
                          #endif LED6812  
                          #ifdef LED2812
 for (int n = FirstLed; n <= LastLed; n++)  strip.setRGB(n,RGBWColor);
                          #endif LED2812 
 //  Serial.println(RGBWColor,HEX);  
}

//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void) 
{
                          #ifdef LED6812    
 for (int n = 0; n <= NUM_LEDS; n++)  strip.setPixelColor(n,0); 
                          #endif LED6812  
                          #ifdef LED2812
 for (int n = 0; n <= NUM_LEDS; n++)   strip.setRGB(n,0);    //  FastLED.clear();
                          #endif LED2812 
}

//--------------------------------------------
// LED Turn On en Off the LED's after Delaytime is milliseconds
//--------------------------------------------
void Laatzien()
{ 
 ShowLeds();
 delay(Delaytime);
 LedsOff(); 
}

//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{                                    // FastLED.show();   // strip.sync();
                     #ifdef LED6812    
 strip.show();
                     #endif LED6812  
                     #ifdef LED2812
 strip.sync();
                     #endif LED2812 
}
//--------------------------------------------
//  LED Set brighness of LEDs
//--------------------------------------------  
void SetBrightnessLeds( byte Bright)
{
                                       #ifdef BWClock
 analogWrite(PWMpin, Bright);     // write PWM                  
                                       #endif BWClock
                                       #ifdef ColourClock 
 strip.setBrightness(Bright);    // strip.sync();    //LEDS.setBrightness(Bright); 
 ShowLeds();
                                        #endif ColourClock 
}

//--------------------------------------------
//  LED convert HSV to RGB 
//  h is from 0-360, s,v values are 0-1
//  r,g,b values are 0-255
//--------------------------------------------
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
return FuncCRGBW((int)(r*255), (int)(g*255), (int)(b*255), 0 );      // R, G, B, W 
}
//--------------------------------------------
//  LED function to make RGBW color
//-------------------------------------------- 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//--------------------------------------------
//  LED functions to extract RGBW colors
//-------------------------------------------- 
 uint8_t white(uint32_t c) { return (c >> 24);}
 uint8_t red(  uint32_t c) { return (c >> 16);}
 uint8_t green(uint32_t c) { return (c >> 8); }
 uint8_t blue( uint32_t c) { return (c);      }

//--------------------------------------------
//  LED Store RGBW colors in EEPROM 
//-------------------------------------------- 
void StoreEEPROM_RGBW(uint32_t RGBWvalue)
{
 EEPROMwrite(4, red(  RGBWvalue));
 EEPROMwrite(5, green(RGBWvalue));
 EEPROMwrite(6, blue( RGBWvalue));
 EEPROMwrite(7, white(RGBWvalue));
}
//--------------------------------------------
//  LED Retrieve RGBW colors from EEPROM
//-------------------------------------------- 
uint32_t GetEEPROM_RGBW(void)
{
 return FuncCRGBW( EEPROM.read(4),EEPROM.read(5),EEPROM.read(6),EEPROM.read(7));
}

//--------------------------------------------
//  LED Set display colors of clock
//-------------------------------------------- 
void CheckColourStatus(void)
{
  switch (DisplayChoice)
  {
   case DEFAULTCOLOUR: LetterColor = DefaultColor;                             break; // Yellow text with changing MIN and SEC  
   case HOURLYCOLOUR : LetterColor = HourColor[Ihour];                         break; // A colour every hour
   case WHITECOLOR   : LetterColor = WhiteColour; 
                       MINColor    = WhiteColour; 
                       SECColor    = WhiteColour;                              break; // all white
   case OWNCOLOUR    : LetterColor = OwnColour; 
                       MINColor    = OwnColour; 
                       SECColor    = OwnColour;                                break; // own colour
   case OWNHETISCLR  : LetterColor = OwnColour;                                break; // own colour except HET IS WAS  
   case WHEELCOLOR   : LetterColor = MINColor = SECColor = Wheel((Iminute*4)); break; // Colour of all letters changes per second
   case DIGITAL      : LetterColor = 0;                                        break; // digital display of time
  } 
//Serial.println(LetterColor,HEX);
}

//--------------------------------------------
// LED Set minute color
// Set the colour per minute of 'HET'
//--------------------------------------------
void SetMinuteColour(void)
{ 
 CheckColourStatus();
 MINColor = LetterColor; 
 switch (DisplayChoice)
  {
   case DEFAULTCOLOUR: 
   case HOURLYCOLOUR :
   case OWNHETISCLR  : MINColor = FuncCRGBW((uint32_t)(15 + (Iminute * 4)), (uint32_t)(255 - Iminute * 4),0,0);    // strip.Color(15 + Iminute * 4,255 - Iminute * 4,0, 0 ); 
                       break;
   case DIGITAL      : MINColor = SECColor = 0;   // No IS in display
                       break;
  }    
//  Serial.print("MINColor: ");   Serial.println(MINColor,HEX);   
}
//--------------------------------------------
//  LED Set second color
//  Set the colour per second of 'IS' and 'WAS'
//--------------------------------------------
void SetSecondColour(void)
{
 CheckColourStatus();  
 SECColor = LetterColor;  
 switch (DisplayChoice)
  {
   case DEFAULTCOLOUR: 
   case HOURLYCOLOUR :
   case OWNHETISCLR  : SECColor = FuncCRGBW(15 + Isecond * 4, 255 -(Isecond * 4), 0, 0 );
                       break;
   case DIGITAL      : MINColor = SECColor = 0;   // No IS in display
                       break;
  }
 if (Is) { ColorLeds("", 4, 5,SECColor); ColorLeds("", 8, 10, 0);        } // Light up  IS or WAS with the proper colour
   else  { ColorLeds("", 4, 5, 0);       ColorLeds("", 8, 10, SECColor); }
 ShowLeds(); // Updating with ColorLeds is done here to avoid updating all letters every second with Displaytime function
//    Serial.print("SecColor: ");    Serial.println(SECColor,HEX);  
}
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{                                                                                               
 if (SecPulse)   // if a second has passed 
 {
  int BrCalc, Temp;
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;   // Read lightsensor 
  Previous_LDR_read = LDR_read;
  OutPhotocell = (int) (((float)LightReducer/100) * sqrt( (float) 63.5 * (float) constrain(LDR_read,1,1023))); // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
 // sprintf(sptext,"LDR read: %d MinPhotocell %d\n",LDR_read, MinPhotocell);   Tekstprint(sptext);
  BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);  // filter out of strange results
  BrCalc = (int) (BrightnessCalcFromLDR/2.55);
  if(print)
  {
   sprintf(sptext,"LDR:%d=%d%% T=%dC ",LDR_read, BrCalc,Get3231Temp()-2);
   Tekstprint(sptext);
                              #ifdef DCFMOD
   sprintf(sptext,"DCF:%d%% ",DCF_signal);   Tekstprint(sptext);
                              #endif DCFMOD                            
   Print_tijd();
  }
  SetBrightnessLeds(BrightnessCalcFromLDR);                                      
 }
 SecPulse = 0;
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
//  for (int n=0; n<12; n++)   { for (int i=0; i<NUM_LEDS; i++) { ColorLeds("",i,i,HourColor[n]); ShowLeds(); } }
//  WhiteOverRainbow(50, 50, 5 );
  WhiteOverRainbow(5, 5, 5 );
  LedsOff();
}

//--------------------------------------------
//  LED Wheel
//  Input a value 0 to 255 to get a color value.
//  The colours are a transition r - g - b - back to r.
//--------------------------------------------

uint32_t Wheel(byte WheelPos) 
{
 WheelPos = 255 - WheelPos;
 if(WheelPos < 85)   { return FuncCRGBW( 255 - WheelPos * 3, 0, WheelPos * 3, 0);  }
 if(WheelPos < 170)  { WheelPos -= 85;  return FuncCRGBW( 0,  WheelPos * 3, 255 - WheelPos * 3, 0); }
 WheelPos -= 170;      
 return FuncCRGBW(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}

//--------------------------------------------
//  LED RainbowCycle
//--------------------------------------------
// Slightly different, this makes the rainbow equally distributed throughout
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

//--------------------------------------------
//  LED WhiteOverRainbow
//--------------------------------------------
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
      for(uint16_t i=0; i<NUM_LEDS; i++) 
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

//--------------------------------------------
//  LED Place digits 0 - 9 in Matrix display
//--------------------------------------------
void Zet_Pixel(byte Cijfer,byte Pos_X, byte Pos_Y) 
{                                   // first row and column = 0, PosX,PoY is left top position of 3x5 digit
                                    // Bereken positie LED #define MATRIX_WIDTH 12 #define MATRIX_HEIGHT 12
 byte     LEDnum;
 byte     IntensityLED = 150;
 uint32_t OrgColor;

 CheckColourStatus();
 for(int i=0;i<3;i++)
  {  
   for(int j=0;j<5;j++)
   {
    int c = pgm_read_byte_near ( &Getal[Cijfer][i][j]); 
    if ( c )          // if Digit == 1 then turn that light on
     {
     //     Serial.print(strip.getPixelColor(LEDnum) & 0X00FFFFFF,HEX); Serial.print(" ");
     if((Pos_Y+j)%2) LEDnum = ((MATRIX_WIDTH -1) - (Pos_X + i) + (Pos_Y + j) * (MATRIX_HEIGHT));
     else            LEDnum =                      (Pos_X + i) + (Pos_Y + j) * (MATRIX_HEIGHT); 
                     #ifdef LED6812    
     strip.getPixelColor(LEDnum) & 0X00FFFFFF  ? OrgColor = LetterColor : OrgColor = 0;
                     #endif LED6812
                     #ifdef LED2812
      strip.sync();
                     #endif LED2812 
     ColorLeds("",  LEDnum, LEDnum, OrgColor + 0X88000000);
      (Pos_Y + j) * (MATRIX_HEIGHT);  
     }
   }
 }
}

//--------------------------------------------
//  LED Time in for digits in display
//--------------------------------------------
void TimePlaceDigit(byte uur, byte minuut)
{   
  Zet_Pixel(    uur / 10, 2, 1);  Zet_Pixel(    uur % 10, 7, 1);
  Zet_Pixel( minuut / 10, 2, 7);  Zet_Pixel( minuut % 10, 7, 7);
}

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(float amount)
{
 LightReducer += amount; 
 WriteLightReducerEeprom(LightReducer);
}

//--------------------------------------------
//  LED Write light intensity to EEPROM
//--------------------------------------------
void WriteLightReducerEeprom(byte waarde)
{
 LightReducer = constrain (waarde, 0 , 255);  // May not be larger than 255
 EEPROMwrite(0, LightReducer);              // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3d%%",LightReducer);
 Tekstprintln(sptext);
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
// Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}

//--------------------------------------------
//  Write to EEPROM
//--------------------------------------------
void EEPROMwrite(byte pos, byte waarde)
{ 
  EEPROM.write(pos, waarde);     
                     #if defined(ARDUINO_SAMD_MKRWIFI1010)
  EEPROM.commit();
                     #endif 
}
//--------------------------------------------
//  Read  EEPROM
//--------------------------------------------
byte EEPROMread(byte pos)  
{ 
  return EEPROM.read(pos);
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(byte waarde)
{
 LowerBrightness = constrain (waarde, 0 , 150);                    // Range between 1 and 100
 EEPROMwrite(1, LowerBrightness);                                    // Default Lower Brightness for this clock
 sprintf(sptext,"Lower brightness: %3ld bits",(long) LowerBrightness);
 Tekstprintln(sptext);
}

// --------------------End Light functions 

//--------------------------------------------
//  CLOCK Input from Bluetooth or Serial
//--------------------------------------------
void ReworkInputString(String InputString)
{
 String temp;
 byte  Intensityfactor;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if (  InputString[0] > 64 && InputString[0] <123 )                                           // Does the string start with a letter?
  { 
  Tekstprintln(sptext);
  switch ((byte)InputString[0])
   {
    case 'A':
    case 'a':          
             break;
    case 'B':
    case 'b':    
           
             break;
    case 'C':
    case 'c':    
                         
            break;
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              int Jaar;
              temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
              temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
              temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
              Iday   = constrain(Iday  , 0, 31);
              Imonth = constrain(Imonth, 0, 12); 
              Jaar   = constrain(Jaar , 1000, 9999); 
              rtc.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
              Tekstprintln(sptext);
             }
             else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");

            break;
    case 'E':
    case 'e':
            Tekstprintln("Entered an E but this has no function");
            break;       
                     #ifdef FMRADIO                        
    case 'F':
    case 'f':
            //set FM frequency
             temp = InputString.substring(1);
             int FMfreq = temp.toInt();
             if (FMfreq < 8750 ) FMfreq = 8750;
             if (FMfreq > 10800) FMfreq = 10800;   
             RDA5807_setFreq((float) FMfreq/100);           
             break;
                     #endif FMRADIO

    case 'G':                                                         // Toggle DCF Signal on Display
    case 'g':
             SeeDCFsignalInDisplay = 1 - SeeDCFsignalInDisplay;
             sprintf(sptext,"SeeDCFsignal: %d",SeeDCFsignalInDisplay);
             Tekstprintln(sptext);
             break;
    case 'I':
    case 'i':   
            SWversion();
            break;
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             temp = InputString.substring(1);
             LowerBrightness = temp.toInt();
             LowerBrightness   = constrain(LowerBrightness  , 1, 255);
             WriteLowerBrightness(LowerBrightness);
             sprintf(sptext,"Lower brightness changed to: %d bits",LowerBrightness);
             Tekstprintln(sptext);
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':    
             temp = InputString.substring(1);
             Intensityfactor = temp.toInt();
             Intensityfactor = constrain(Intensityfactor, 1, 255);
             WriteLightReducerEeprom(Intensityfactor);
             sprintf(sptext,"Max brightness changed to: %d%%",Intensityfactor);
             Tekstprintln(sptext);
             break;  
    case 'P':
    case 'p':  
             if (InputString.length() == 7 )
              {
               temp = InputString.substring(1,7);
               OwnColour = HexToDec(temp);                           // display letter color
               LetterColor = OwnColour;                              // display letter color 
               StoreEEPROM_RGBW(OwnColour);                         // Store the RGBW value in EEPROM
               DisplayChoice = OWNCOLOUR;
               Tekstprintln("**** Palette changed ****"); 
               Displaytime();
               }
             else Tekstprintln("**** Length fault. Enter PFFFFFF ****");            
             break;
    case 'q':
    case 'Q':  
             if (InputString.length() == 2 )
              {
                temp   = InputString.substring(1,2);     
                DisplayChoice = (byte) temp.toInt(); 
                EEPROMwrite(2, DisplayChoice);                       // store the display choice in EEPROM
                sprintf(sptext,"Display Choice Q%d",DisplayChoice);
                Tekstprintln(sptext);
//            SetSecondColour();                                    // Set the colour per second of 'IS' and 'WAS' 
                lastminute = 99;                                      // force a minute update
//                SetMinuteColour();                                   // Set the colour per minute of 'HET' 
//                Displaytime();
               }
             else Tekstprintln("**** Display Choice Length fault. Enter Q0 - Q6"); 
             Displaytime();             
            break;       


    case 'R':
    case 'r':
            Reset();                                                 // Reset all settings 
            Tekstprintln("**** Resetted to default settings ****"); 
            break;
                     #ifdef FMRADIO
    case 'S':
    case 's':
            RDA5807_ReadStatus();            
            break;
    case 'T':
    case 't':    
            RDA5807_RDS_Dump();                                          
            break;
    case 'U':
    case 'u':
            RDA5807_Report();
            break;      
                     #endif FMRADIO 
    case 'X':
    case 'x':    
             if (InputString.length() >1 )
               {
                temp = InputString.substring(1,5);
                MilliSecondValue = temp.toInt();                
               }
             Demo = 1 - Demo;                                          // toggle Demo mode
             if (!Demo)  
              { 
                MilliSecondValue = 1000;                              // So clock runs again at normal speed minus 1 ms
                DisplayChoice    = EEPROMread(2);                     // read the display mode from EEPROM
              }
             sprintf(sptext,"Demo mode: %d MillisecondTime=%d",Demo,MilliSecondValue);
             Tekstprintln(sptext);
             break; 
    case 'Z':
    case 'z':
             Zelftest = 1 - Zelftest; 
             sprintf(sptext,"Zelftest: %d",Zelftest);
             Tekstprintln(sptext); 
             Displaytime();                                 
             break;         
    default:
            break;
    }
  }
 else if (InputString.length() > 3 && InputString.length() <7 )      // If time is entered as digits (hhmmss) 
           {        
           temp = InputString.substring(0,2);   
           Ihour = constrain(temp.toInt(), 0, 23); 
           if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = constrain(temp.toInt(), 0, 59); }
           if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = constrain(temp.toInt(), 0, 59); }           
           SetRTCTime();
           }
 InputString = "";
 temp = "";
}

//--------------------------------------------
//  CLOCK Convert Hex to uint32
//--------------------------------------------
uint32_t HexToDec(String hexString) 
{
 uint32_t decValue = 0;
 int nextInt;
 for (int i = 0; i < hexString.length(); i++) 
  {
   nextInt = int(hexString.charAt(i));
   if (nextInt >= 48 && nextInt <= 57)  nextInt = map(nextInt, 48, 57, 0, 9);
   if (nextInt >= 65 && nextInt <= 70)  nextInt = map(nextInt, 65, 70, 10, 15);
   if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
   nextInt = constrain(nextInt, 0, 15);
   decValue = (decValue * 16) + nextInt;
  }
return decValue;
}


//--------------------------------------------
//  CLOCK Dutch clock display
//--------------------------------------------
void Dutch(void)
{
 if (Ihour == 12 && Iminute == 0 && random(10)==0) { HET; IS; NOEN; return; }
 if (Ihour == 00 && Iminute == 0 && random(5)==0) { HET; IS; MIDDER; NACHT; return; } 

 HET;                                       // HET light is always on
 switch (Iminute)
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
//if (Ihour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = Ihour;
 if (Iminute > 18 )  sayhour = Ihour+1;
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
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UUR;  break; 
 }
}
                      #ifdef UK
//--------------------------------------------
//  CLOCK English clock display
//--------------------------------------------
void English(void)
{
 IT;                                       // HET light is always on
 switch (Iminute)
 {
  case  0: ISUK;  EXACT; break;
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
  case 29: ISUK;  HALF; PAST; break;
  case 30: ISUK;  EXACT; HALF; PAST; break;
  case 31: ISUK;  HALF; PAST; break;
  case 32: 
  case 33: WASUK; HALF; break;
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
//if (Ihour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = Ihour;
 if (Iminute > 33 ) sayhour = Ihour+1;
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
  case 6:  SIX; break;
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
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: OCLOCK;  break; 
 }

}
                           #endif UK
                           #ifdef DE
//--------------------------------------------
//  CLOCK German clock display
//--------------------------------------------
void German(void)
{
  ES;                                       // HET light is always on
 switch (Iminute)
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
//if (Ihour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

 sayhour = Ihour;
 if (Iminute > 18 ) sayhour = Ihour+1;
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
 switch (Iminute)
 {
  case 59: 
  case  0: 
  case  1: 
  case  2: 
  case  3: UHR;  break; 
 }
}
                      #endif DE
                      #ifdef FR
//--------------------------------------------
//  CLOCK French clock display
//--------------------------------------------
void French(void)
{
 IL;                                       // HET light is always on
 switch (Iminute)
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
//if (Ihour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);
// switch (Iminute)
// {
//  case 59: 
//  case  0: 
//  case  1: 
//  case  2: 
//  case  3: if(sayhour%12 == 1) {HEURE;} 
//           else                {HEURES;}  
//           break; 
// }
}

void DitLeHeure(void)
{
 byte sayhour = Ihour;
 if (Iminute > 33 ) sayhour = Ihour+1;
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
                      #endif FR
                      #if defined(ARDUINO_SAMD_MKRWIFI1010)
//--------------------------------------------
// SETUP  ARDUINO_SAMD_MKRWIFI1010
//-------------------------------------------- 
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
//--------------------------------------------
// SERVER Update routine done every second
//--------------------------------------------
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
                          #endif
