// =============================================================================================================================
/* 
This Arduino code controls the Nano Every, ATMEGA1284 chip and Arduino MKR1010 that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor GL5528
- Bluetooth RF Transceiver Module like HC-05 HM-10, JDY-23 CC25nn and Nordic nRFnn in RP2040 BLE33 etc


The CC254x or nRF52 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module.

Arduino Uno, Nano with SK6812 LEDs: Program size must stay below approx 23572, 1129 bytes bytes with 144 LEDs in ATMEGA 32KB 
With a Arduino Nano Every with 48kB memory this is not a problem
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
WHEELCOLOR    = 5; Every minute rotating colour of rainbow
DIGITAL       = 6; Digital display
************************************************************************************
 Author .: Ed Nieuwenhuys

Changes: V083 Used for 4-language clock No1 with Nano Every and compiled with MegaCore X, ATMega4809, JTAG2UDPI
              Updated menu. Added toggle On/Off LEDs 
              In Viertalenklok No1
Changes: V084 Removed LedsOff(); in Displaytime(). Caused flickering. 
              Repaired RotaryEncoderCheck(0 and ProcessKeyPressTurn(0. comparable with Character_Clock_V117.ino & CharacterClock23cm_V005
              Mem.NVRAMmem byte --> int 
Changes: V085 DimLeds() between 0 and 1023 indtead of betwen 0 and 255
Changes: V086 DCF defines corrected to compile without errors
              Corrected bug in colouring seconds and flickering between minutes
              Cleaned up code. Removed DCFNointerrupt
Changes: V087tiny  
              
add in Additional boards URL
https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
examples here:
https://arduino-pico.readthedocs.io/en/latest/eeprom.html#eeprom-examples

https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-01-technical-reference

 */

 
// ===============================================================================================================================
//                                                                                            //
//------------------------------------------------------------------------------
//  Definition of installed modules
//------------------------------------------------------------------------------

// ------------------>   Define only one library
#define NEOPIXEL                                   // Adafruit Neopixel for WS2812 or SK6812 LEDs
//#define LIBSK6812                                // SK6812 library. Only with SK6812 LEDs  (saves 614 bytes with NEOPIXEL)

// ------------------>  Define which module is present.  
#define BLUETOOTHMOD                               // Use  this define if Bluetooth needs other pins than pin 0 and pin 1.
                                //#define BLEnRF52MOD                               // turn on for RP2040, Nano BLE33 , MKR1010 etc
#define MOD_DS3231                                 // The DS3231 module is installed, if not the Arduino internal clock is used
                                //#define LCDMOD                                   // For LCD support
#define ROTARYMOD                                  // Rotary encoder installed
  
//------------------------------------------------------------------------------
//  Includes defines and initialisations
//------------------------------------------------------------------------------
//                                                                                            //
                                      
#include <TimeLib.h>                               // https://github.com/PaulStoffregen/Time 
#include <Wire.h>                                  // Arduino standard library
#include "RTClib.h"                                // https://github.com/adafruit/RTClib 
#include <EEPROM.h>
                     #ifdef NEOPIXEL
#include <Adafruit_NeoPixel.h>   // https://github.com/adafruit/Adafruit_NeoPixel   for LED strip WS2812 or SK6812
                     #endif  // NEOPIXEL
                     #ifdef LIBSK6812
#include <EdSoft_SK6812.h>                         // https://github.com/ednieuw/EdSoft_SK6812
                     #endif // LIBSK6812                
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>                        // Arduino for Bluetooth communication
                     #endif //BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>                               // http://www.pjrc.com/teensy/td_libs_Encoder.html 
                     #endif //ROTARYMOD

#define HET     ColorLeds("Het",     0,   2, MINColor);   
#define IS      ColorLeds("is",      4,   5, SECColor);  ColorLeds("", 8,10, 0); Is = true;
#define WAS     ColorLeds("was",     8,  10, SECColor);  ColorLeds("", 4, 5, 0); Is = false;
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


//                                                                                            //
//------------------------------------------------------------------------------
// PIN Assigments
//------------------------------------------------------------------------------ 
             #if defined(__AVR_ATmega328P__) || defined(ARDUINO_ARCH_RP2040) || defined (ARDUINO_AVR_NANO_EVERY)|| defined (ARDUINO_AVR_ATmega4809) || defined(ARDUINO_ARCH_SAMD)
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
 DCF_LED_Pin  = 9,                // DCF signal
 DCFgood      = 10,               // DCF-signal >25    
 HeartbeatLED = 11,               // Shows heeartbeat ritm 
 PIN11        = 11,               // PIN 11
 PIN12        = 12,               // PIN 12             // LED = 12,          //
 secondsPin   = LED_BUILTIN,      // PIN 13   
 };
 
enum AnaloguePinAssignments {     // Analogue hardware constants ----
 EmptyA0      = A0,                // Empty
 EmptyA1      = A1,                // Empty
 PhotoCellPin = A2,                // LDR pin
 OneWirePin   = A3,                // OneWirePin
 SDA_pin      = A4,                // SDA pin
 SCL_pin      = A5,                // SCL pin
 EmptyA6     =  A6,                // Empty
 EmptyA7     =  A7};               // Empty
                             #endif

                             #if defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
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
 DCFgood      = 15,               // LED15         PD7 PWM
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

const uint32_t white  = 0xFF000000, lgray  = 0x66000000;           // The SK6812 LED has a white LED that is pure white
const uint32_t dgray  = 0x22000000, gray   = 0x33000000;

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
//                                                                 //

const byte NUM_LEDS      = 144;                                    // How many leds in  strip?
const byte MATRIX_WIDTH  = 12;
const byte MATRIX_HEIGHT = 12;                                            
const byte BRIGHTNESS    = 32;                                     // BRIGHTNESS 0 - 255

                             #ifdef NEOPIXEL   
Adafruit_NeoPixel LEDstrip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRBW + NEO_KHZ800);  //NEO_RGBW
                             #endif //NEOPIXEL
                             #ifdef LIBSK6812
EdSoft_SK6812 LEDstrip(NUM_LEDS, LED_PIN);                          // Initialyse SK6812 library
                            #endif  //LIBSK6812  

bool     LEDsAreOff            = false;                             // If true LEDs are off except time display
bool     NoTextInColorLeds     = false;                             // Flag to control printing of the text in function ColorLeds()
int      Previous_LDR_read     = 512;                               // The actual reading from the LDR + 4x this value /5
uint32_t MINColor      = chromeyellow;
uint32_t SECColor      = chromeyellow;  
uint32_t DefaultColor  = chromeyellow;   
uint32_t LetterColor   = chromeyellow;   
uint32_t OwnColour     = 0X002345DD;        // Blueish
uint32_t WhiteColour   = white;
uint32_t WheelColor    = blue;
uint32_t HourColor[] ={  white,      darkviolet, cyberyellow, capri,         amber,         apple,
                         darkorange, cerulean,   edviolet,    cadmiumyellow, green,         edamaranth,
                         red,        yellow,     coquelicot,  pink,          apple,         hotmagenta,
                         green,      greenblue,  brightgreen, dodgerblue,    screamingreen, blue,
                         white,      darkviolet, chromeyellow};       

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
const byte SLOPEBRIGHTNESS  = 80;                                 // Steepness of with luminosity of the LED increases
const int  MAXBRIGHTNESS    = 999;                                // Maximun value in bits  for luminosity of the LEDs (1 - 1023)
const byte LOWBRIGHTNESS    = 5;                                  // Lower limit in bits of Brightness ( 0 - 255)   
byte     TestLDR            = 0;                                  // If true LDR inf0 is printed every second in serial monitor
int      OutPhotocell;                                            // stores reading of photocell;
int      MinPhotocell       = 999;                                // stores minimum reading of photocell;
int      MaxPhotocell       = 1;                                  // stores maximum reading of photocell;
uint32_t SumLDRreadshour    = 0;
uint32_t NoofLDRreadshour   = 0;

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
//                                                                //
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
                          #ifdef BLUETOOTHMOD                     // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);                           // BT_RX <=> TXD on BT module, BT_TX <=> RXD on BT module
                           #endif  //BLUETOOTHMOD 
                          #ifdef BLEnRF52MOD        
#include <HardwareBLESerial.h>
HardwareBLESerial &Bluetooth = HardwareBLESerial::getInstance();        
                          #endif //BLEnRF52MOD                                     
//------------------------------------------------------------------------------
// Webserver
//------------------------------------------------------------------------------
//                                                                 //
byte KeyInputactivated = false;                                    // if pressing the keys stop other events like EveryMinuteUpdate()   

//----------------------------------------
// Common
//----------------------------------------
// 
bool   SeeDCFsignalInDisplay = false;                              // If ON then the display line HET IS WAS will show the DCF77-signal received
bool   UseDCF                = false;                              // Use the DCF-receiver or not
char   sptext[100];                                                // For common print use
uint16_t  MilliSecondValue  = 1000;                                // The duration of a second  minus 1 ms. Used in Demo mode
static uint32_t last_time = 0;                                     // Heartbeat defines
struct EEPROMstorage {                                             // Data storage in EEPROM to maintain them after power loss
  byte LightReducer;
  byte LowerBrightness;
  int  UpperBrightness;
  int  NVRAMmem[24];                                               // LDR or DCF readings
  byte DisplayChoice;
  byte TurnOffLEDsAtHH;
  byte TurnOnLEDsAtHH;
  uint32_t OwnColour;                                              // Self defined colour for clock display
  uint32_t NOP[9];                                                 // For future use
  uint32_t Checksum;
} Mem;// {0};
//------------------------------------------------------------------------------
// Menu
//------------------------------------------------------------------------------  
//0        1         2         3         4         5
//12345678901234567890123456789012345678901234567890  
 char menu[][42] =  {                       // menu[][nn]  nn is largest length of sentence in the menu 
 "Character B/W-Colour Clock_No..",                                                                      // Beacon name of the BLE module
 "Enter time as: hhmmss (132145)",
 "D Date (D15012021) T Time (T132145)",   //
 "L (L5) Min light intensity(0-255 bits)", //
 "M (M90)Max light intensity(1-999)",
 "N (N2208)Turn OFF LEDs between Nhhhh",   //
 "O Display toggle On/Off", 
 "P (P00234F8A) own colour (n=0-F)",
 "Q Display Choice (Q0-6) Q=Choices)",
 "I For this info",
 "R Reset to default settings",
 "S Self test",
 "W Test LDR reading every second",       //
 "X (X50) Demo mode. ms delay (0-9999)",
 "Y (Y50) Slope lightintensity(1-250)",
 "Ed Nieuwenhuys Feb 2023" };

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
                         #if defined(MEGACOREX)
 Tekstprintln("Compiled with MegaCoreX board"); 
                         #else 
 Tekstprintln("Compiled with Arduino MegaAVR board");   
                         #endif
 pinMode(secondsPin,   OUTPUT );  
 pinMode(HeartbeatLED, OUTPUT );      
                          #ifdef MOD_DS3231
 RTCklok.begin();
 Tekstprintln("RTC DS3231 enabled");// start the RTC-module
                          #else 
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
                          #ifdef BLEnRF52MOD        
 if (!Bluetooth.beginAndSetupBLE(menu[0]))                                                    // menu[0] contains the BT beacon name 
      Tekstprintln("Failed to initialize HardwareBLESerial!");
 else Tekstprintln("Bluetooth nRF52 enabled");       
                          #endif //BLEnRF52MOD                                  
                          #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Tekstprintln("Rotary enabled"); 
 myEnc.write(0);                                                                              // Clear Rotary encode buffer
                          #endif  //ROTARYMOD 
                          #ifdef NEOPIXEL 
 LEDstrip.begin();                                                                            // Start communication to LED strip
 LEDstrip.setBrightness(BRIGHTNESS);                                                          // Set brightness of LEDs
 ShowLeds(); 
 Tekstprintln("LIB NEOPIXEL");   
                          #endif //NEOPIXEL 
                          #ifdef LIBSK6812
 LEDstrip.setBrightness(BRIGHTNESS);                                                          // Set brightness of LEDs
 Tekstprintln("LIBSK6812");                                                                   // Initialyse SK6812 library
                          #endif //LIBSK6812                                                                                                     // Initialize all pixels to 'off' 
                          #ifdef ARDUINO_SAMD_MKRWIFI1010
 Tekstprintln("Compiled for MKR-1010");
                          #endif
                          #if defined(ARDUINO_ARCH_RP2040)
 Tekstprintln("Compiled for ARDUINO NANO RP2040");
                          #endif
                          #ifdef ARDUINO_AVR_NANO_EVERY
 Tekstprintln("Compiled for ARDUINO AVR NANO EVERY");
                          #endif
                          #if defined(__AVR_ATmega328P__) 
 Tekstprintln("Compiled with ARDUINO AVR ATmega328P emulation");
                          #endif
                          #if defined(__AVR_ATmega4809__) 
 Tekstprintln("Compiled with ARDUINO_AVR_ATmega4809 emulation");
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
 Mem.UpperBrightness = min(Mem.UpperBrightness, 1023); 
 Mem.DisplayChoice   = min(Mem.DisplayChoice, DIGITAL);                                       // 
 if ( Mem.OwnColour == 0 ) Mem.OwnColour = 0X002345DD;                                        // If memory is empty then store default value, blue  
 EEPROM.put(0,Mem);                                                                           // update EEPROM if some data are out of the constrains     
 Previous_LDR_read = analogRead(PhotoCellPin);                                                // to have a start value
 MinPhotocell      = Previous_LDR_read;                                                       // Stores minimum reading of photocell;
 MaxPhotocell      = Previous_LDR_read;                                                       // Stores maximum reading of photocell;                                            
// Selftest();                                                                                // Play the selftest
 GetTijd(0);                                                                                  // Get the time and store it in the proper variables
 SWversion();                                                                                 // Display the version number of the software
 Displaytime();                                                                               // Turn on the LEDs with proper time and display choice
 sprintf(sptext,"\nChecksum (25065) = %ld (%s)",Mem.Checksum, Mem.Checksum==25065 ? "OK" : "NOK Will reset settings" );  
 Tekstprintln(sptext); 
 if( Mem.Checksum != 25065)  Reset();                                                         // If the checksum is incorrect the data were not set to default values
 while (Serial.available())  Serial.read();                                                   // Flush the serial input buffer. When a Bluetooth module is connected to RX/TX pin 0&1 garbage can set the time to 00:00:00
 msTick = Looptime = millis();                                                                // Used in KY-040 rotary for debouncing and seconds check, Start of DCF 1 second loop
 } 
//                                                                                            //

//------------------------------------------------------------------------------
// CLOCK Update routine done every second
//------------------------------------------------------------------------------
void EverySecondCheck(void)
{
 uint32_t ms = millis() - msTick;                                                             // A Digitalwrite() is very time consuming. 
 static bool Dpin;                                                                            // Only write once to improve program speed in the loop()
 if (ms > 1 && Dpin)  {Dpin = LOW; digitalWrite(secondsPin,LOW);}                             // Turn OFF the second on pin 13 
 if(UseDCF) if(ms==(ms>>5)<<5) digitalWrite(DCF_LED_Pin, !digitalRead(DCF_PIN));              // Write received signal to the DCF LED every 32 loops (>>5 <<5)
 if (ms > 999)                                                                                // Every second enter the loop
  {
   GetTijd(0);                                                                                // Update I.Second, I.Minute, I.Hour, I.Day, I.Month, I.Year
   msTick = millis();                                                                                            // Set the colour per second of 'IS' and 'WAS' 
   digitalWrite(secondsPin,Dpin=HIGH); //     Dpin = HIGH;                                    // turn ON the second on pin        
   SetSecondColour();                                                                         // Set the colour per second of 'IS' and 'WAS' 
   DimLeds(TestLDR);                                                                          // Every second an intensitiy check and update from LDR reading 
   }
  Heartbeat();                                                                            // Only heartbeat with ATMEGA1284 or Nano Every
  if (I.Minute != lastminute)   EveryMinuteUpdate();                                          // Enter the every minute routine after one minute
 }
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Update routine done every minute
//------------------------------------------------------------------------------
void EveryMinuteUpdate(void)
{
 lastminute = I.Minute;  
 GetTijd(0);
 Displaytime();
 Print_RTC_tijd();
 DimLeds(true);                                                                              // measure the LDR and print the results
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
    SumLDRreadshour  = 0;
    NoofLDRreadshour = 0;
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
    EEPROM.put(0,Mem);                                                                      // This function uses EEPROM.update() to perform the write, so does not rewrites the value if it didn't change
   }
}
//------------------------------------------------------------------------------
// ARDUINO Reset to default settings
//------------------------------------------------------------------------------
void Reset(void)
{
 Mem.Checksum         = 25065; 
 Mem.LightReducer     = SLOPEBRIGHTNESS;                                                      // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
 Mem.UpperBrightness  = MAXBRIGHTNESS;                                                        // Upper limit of Brightness in bits ( 1 - 1023)
 Mem.LowerBrightness  = LOWBRIGHTNESS;                                                        // Lower limit of Brightness in bits ( 0 - 255)
 Mem.DisplayChoice    = ResetDisplayChoice; 
 Mem.TurnOffLEDsAtHH  = Mem.TurnOnLEDsAtHH = 0;
 Mem.OwnColour        = 0X002345DD;                                                           // Blue 
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
 for (int i=0;i<24;i++) Mem.NVRAMmem[i] = 3;                                                  // Reset LDR readings 
 EEPROM.put(0,Mem);                                                                           // Update EEPROM    
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
 #define FILENAAM (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
 unsigned int i;
 PrintLine(48);
 for (i = 0; i < sizeof(menu) / sizeof(menu[0]); Tekstprintln(menu[i++]));
 PrintLine(48);
 for (i=0;i<12;i++) { sprintf(sptext," %02d ",i );             Tekstprint(sptext); }       Tekstprintln("");
 for (i=0;i<12;i++) { sprintf(sptext,"%03d ",Mem.NVRAMmem[i]); Tekstprint(sptext); }       Tekstprintln("");
 for (i=12;i<24;i++){ sprintf(sptext,"%03d ",Mem.NVRAMmem[i]); Tekstprint(sptext); }       Tekstprintln("");
 PrintLine(48);
 sprintf(sptext,"Brightness Min : %3d Max: %3d Slope: %3d%%",
                    Mem.LowerBrightness, Mem.UpperBrightness, Mem.LightReducer);           Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min :%4d bits. LDR Max: %3d bits",MinPhotocell, MaxPhotocell); Tekstprintln(sptext); 
                                   #ifdef DCF77MOD   
 sprintf(sptext,"DCF-receiver is %s  Signal:%3d%%",UseDCF ? "On" : "Off",DCF_signal);      Tekstprintln(sptext);
                                   #endif // DCF77MOD   

 byte dp = Mem.DisplayChoice;
 sprintf(sptext,"   Display off : %02dh - %02dh",Mem.TurnOffLEDsAtHH, Mem.TurnOnLEDsAtHH); Tekstprintln(sptext);
 sprintf(sptext,"Display choice : %s",dp==0?"Default":dp==1?"Hourly":dp==2?"White":
                      dp==3?"All Own":dp==4?"Own":dp==5?"Wheel":dp==6?"Digital":"NOP");    Tekstprintln(sptext);

 sprintf(sptext,"Software: %s\n Time Date:  ",FILENAAM);                                   Tekstprint(sptext);  // VERSION); 
 GetTijd(1);  
 sptext[0] = 0;
 PrintLine(48);
}
//------------------------------------------------------------------------------
// CLOCK Print line with ---
//------------------------------------------------------------------------------
void PrintLine(byte Length)
{
 for (int n=0; n<Length; n++) {Tekstprint("-");} Tekstprintln(""); 
}
//------------------------------------------------------------------------------
// CLOCK common print routines
//------------------------------------------------------------------------------
//                                                                                            //
void Tekstprint(char const *tekst)
{
 Serial.print(tekst);    
                           #if defined (BLUETOOTHMOD)  || defined (BLEnRF52MOD) 
 Bluetooth.print(tekst);  
                          #ifdef ARDUINO_SAMD_MKRWIFI1010
 Serial1.print(tekst);  
                          #endif
                          #endif  //BLUETOOTHMOD / BLEnRF52MOD
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
  if( I.Hour >12)                                                                             // If hour is after 12 o'clock 
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
// CLOCK check for input from devices
// This fubction is called from with the loop()
//------------------------------------------------------------------------------
//                                                                                            //
void InputDevicesCheck(void)
{
 SerialCheck();
                            #ifdef ROTARYMOD      
 RotaryEncoderCheck(); 
                           #endif  //ROTARYMOD
                           #if defined (BLUETOOTHMOD)  || defined (BLEnRF52MOD)
 BluetoothCheck(); 
                           #endif  //BLUETOOTHMOD / BLEnRF52MOD                           
}
//------------------------------------------------------------------------------
// CLOCK check for serial input
//------------------------------------------------------------------------------
void SerialCheck(void)
{
 String SerialString = "";
 while (Serial.available())
  {
   char c = Serial.read();  delay(3); 
   if (c>31 && c<127) SerialString += c;                                                      // Allow input from Space - Del
   else c = 0;
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);                            // Rework ReworkInputString();
 SerialString = "";
}
                           #if defined (BLUETOOTHMOD)  || defined (BLEnRF52MOD)
//------------------------------------------------------------------------------
// CLOCK check for Bluetooth input
//------------------------------------------------------------------------------                           
void BluetoothCheck(void)
{ 
  String  BluetoothString = "";
  char c = 0;
                          #ifdef BLEnRF52MOD 
  Bluetooth.poll();                                                                          // this must be called regularly to perform BLE updates
                          #endif //BLEnRF52MOD
// Bluetooth.listen();                                                                       //  When using two software serial ports, you have to switch ports by listen()ing on each one in turn.
 while (Bluetooth.available()) 
  { 
   c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<127) BluetoothString += c;
   else c = 0;
   delay(3);
  }
 if (BluetoothString.length()>0) 
   {   
    ReworkInputString(BluetoothString);                                                      // Rework ReworkInputString();
    BluetoothString = "";
   }
}
                           #endif  //BLUETOOTHMOD / BLEnRF52MOD
                           #ifdef ROTARYMOD
//------------------------------------------------------------------------------
// KY-040 ROTARY check if the rotary is moving  
//------------------------------------------------------------------------------
void RotaryEncoderCheck(void)
{
 int ActionPress = 999;
 if (digitalRead(clearButton) == LOW )          ProcessKeyPressTurn(0);                       // Set the time by pressing rotary button
 else if (ChangeTime)    
  {   
   ActionPress = myEnc.read();                                                                // If the knob is turned store the direction (-1 or 1)
   if (ActionPress == 0) {  ActionPress = 999;  ProcessKeyPressTurn(ActionPress);  }          // Sent 999 = nop 
   if (ActionPress == 1 || ActionPress == -1 )  ProcessKeyPressTurn(ActionPress);             // Process the ActionPress
  } 
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
if ((unsigned long) (millis() - RotaryPressTimer) > 60000)                                    // After 60 sec after shaft is pressed time of light intensity can not be changed 
   {
    if (ChangeTime || ChangeLightIntensity)                         
      {
        WhiteOverRainbow(0, 0, 12 );                                                          // wait, whiteSpeed, whiteLength
        Tekstprintln("<-- Changing time is over -->");
        NoofRotaryPressed = 0;
        lastminute = 99;                                                                      // Force a minute update
      }
    ChangeTime            = false;
    ChangeLightIntensity  = false;
   }  
 if (ChangeTime || ChangeLightIntensity)                                                      // If shaft is pressed time of light intensity can be changed
   {
    if ( encoderPos!=999 && ( (millis() - Looptime) > 150))                                   // If rotary turned avoid debounce within 0.15 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos == 1)                                                                     // Increase  
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }                                  // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
              {if( ++I.Hour >23) { I.Hour = 0; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
              {  I.Second = 0;
               if( ++I.Minute >59) { I.Minute = 0; if( ++I.Hour >23) { I.Hour = 0; } }   }
           } 
        }    
      if (encoderPos == -1)                                                                   // Decrease
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                                                        // Change hours
            { if( I.Hour-- ==0) { I.Hour = 23; } }      
           if (NoofRotaryPressed == 2)                                                        // Change minutes
            { I.Second = 0;
             if( I.Minute-- == 0) { I.Minute = 59; if( I.Hour-- == 0) { I.Hour = 23; } }  }
          }          
        } 
      SetRTCTime();  
      Print_RTC_tijd();
      Looptime = millis();       
     }                                                
   }
 if (encoderPos == 0 )                                                                        // Set the time by pressing rotary button
   { 
    delay(250);
    ChangeTime            = false;
    ChangeLightIntensity  = false;
    SeeDCFsignalInDisplay = false;                                                            // Shows the DCF-signal in the display
    RotaryPressTimer      = millis();                                                         // Record the time the shaft was pressed.
    if(++NoofRotaryPressed >10 ) NoofRotaryPressed = 0;
    switch (NoofRotaryPressed)                                                                // No of times the rotary is pressed
      {
       case 1:  ChangeTime = true;            BlinkUUR(3, 20);      break;                    // Change the hours
       case 2:  ChangeTime = true;            BlinkHETISWAS(3, 20); break;                    // Change the hours        
       case 3:  ChangeLightIntensity = true;  BlinkLetters(5, 10);  break;                    // Turn on all LEDs and change intensity 
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
    Displaytime();                                                                            // Turn on the LEDs with proper time
   }
 }
//                                                                                            //
//------------------------------------------------------------------------------
//  CLOCK Blink UUR
//------------------------------------------------------------------------------
void BlinkUUR(int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) 
     { 
      LedsOff(); Laatzien(); delay(Delayms); UUR; 
      Laatzien(); delay(Delayms);
     } 
}
//------------------------------------------------------------------------------
//  CLOCK Blink HET IS WAS
//------------------------------------------------------------------------------
void BlinkHETISWAS (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) 
     { 
      LedsOff(); Laatzien(); delay(Delayms); HET; IS; WAS; 
      Laatzien(); delay(Delayms);
     } 
}
//------------------------------------------------------------------------------
//  CLOCK Blink Letters
//------------------------------------------------------------------------------
void BlinkLetters (int NoofBlinks, int Delayms)
{
 for (int n=0 ; n<=NoofBlinks; n++) 
     { 
      LedsOff(); Laatzien(); delay(Delayms); ColorLeds("",  36, 59, 0XFF0000FF);  
      Laatzien(); delay(Delayms);
     } 
}
//------------------------------------------------------------------------------
// CLOCK Self test sequence
//------------------------------------------------------------------------------
void Selftest(void)
{ 
 GetTijd(1);                                                                                  // Prints time in Serial monitor
 LedsOff(); 
 HET;   Laatzien(); IS;    Laatzien(); WAS;    Laatzien(); PRECIES; Laatzien(); MTIEN;  Laatzien();  MVIJF; Laatzien();    
 KWART; Laatzien(); VOOR;  Laatzien(); OVER;   Laatzien(); HALF;    Laatzien(); MIDDER; Laatzien();  VIJF;  Laatzien();
 TWEE;  Laatzien(); EEN;   Laatzien(); VIER;   Laatzien(); TIEN;    Laatzien(); TWAALF; Laatzien();  DRIE;  Laatzien();
 NEGEN; Laatzien(); NACHT; Laatzien(); ACHT;   Laatzien(); ZES;     Laatzien(); ZEVEN;  Laatzien();  ELF;   Laatzien(); 
 NOEN;  Laatzien(); UUR;   Laatzien(); EDSOFT; Laatzien();
 Tekstprintln("*");    
 Play_Lights();    
 Zelftest = false; 
 Displaytime();                                                                               // Turn on the LEDs with proper time
}
//                                                                                            //
//------------------------------------------------------------------------------
// CLOCK Say the time and load the LEDs 
// with the proper colour 
//------------------------------------------------------------------------------
void Displaytime(void)
{ 
 LedsOff();                                                                                   // Start by clearing the display to a known state   
 if( Mem.DisplayChoice == DIGITAL ) { TimePlaceDigit(I.Hour,I.Minute); }
 else Dutch();                                                                               // If not a digital display 
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
 RTCklok.adjust(DateTime(Dtime.Year, Dtime.Month, Dtime.Day, Dtime.Hour, Dtime.Minute, 0));   // Update time here
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
// if (I.Hour > 24) { I.Hour = random(12)+1; I.Minute = random(60)+1; I.Second = 30;}         // set a time if time module is absent or defect
 if (printit)  Print_RTC_tijd(); 
}
//                                                                                            //

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
                      #ifdef MOD_DS3231
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
                      #endif // MOD_DS3231
                                                                                           //
// --------------------Colour Clock Light functions -----------------------------------
//------------------------------------------------------------------------------
//  LED Set color for LEDs in strip and print tekst
//------------------------------------------------------------------------------
void ColorLeds(char const *Tekst, int FirstLed, int LastLed, uint32_t RGBWColor)
{ 
 Stripfill(RGBWColor, FirstLed, ++LastLed - FirstLed );                                       //
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
 Stripfill(0, 0, NUM_LEDS );                                                                  // 
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
 LEDstrip.show();
}
//------------------------------------------------------------------------------
//  LED Set brighness of LEDs
//------------------------------------------------------------------------------  
void SetBrightnessLeds(byte Bright)
{
 LEDstrip.setBrightness(Bright);                                                              // Set brightness of LEDs   
 ShowLeds();
}
//--------------------------------------------
//  LED Fill the strip array for LEDFAB library
//--------------------------------------------
void Stripfill(uint32_t RGBWColor, int FirstLed, int NoofLEDs)
{   
 LEDstrip.fill(RGBWColor, FirstLed, NoofLEDs);
}
//--------------------------------------------
//  LED Strip Get Pixel Color 
//--------------------------------------------
uint32_t StripGetPixelColor(int Lednr)
{
return(LEDstrip.getPixelColor(Lednr));
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
                                                                                              // Light up  IS or WAS with the proper colour  
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
 NoTextInColorLeds  = true;                                                                   // Flag to control printing of the text IS en WAS in serial
 if(Is) {IS;} 
 else {WAS;} 
 NoTextInColorLeds  = false;                                                                  // Flag to control printing of the text IS en WAS in serial
 ShowLeds();                                                                                  // Updating IS and WAS with ShowLeds is done here to avoid updating all letters every second with Displaytime function
//    Serial.print("SecColor: ");    Serial.println(SECColor,HEX);  
}


//--------------------------------------------
//  LED Dim the leds measured by the LDR and print values
// LDR reading are between 0 and 1024. The Brightness send to the LEDs is between 0 and 255
//--------------------------------------------
void DimLeds(bool print) 
{                                                                                                       
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;                     // Read lightsensor (0-1024)and avoid rapid light intensity changes
  Previous_LDR_read = LDR_read;                                                              // by using the previous reads
  OutPhotocell = (int)( (Mem.LightReducer * sqrt(1023*(long)LDR_read))/100);                 // Linear --> hyperbolic with sqrt. (0-1024)
  MinPhotocell = min(MinPhotocell, LDR_read);                                                // Lowest LDR measurement (0-1024)
  MaxPhotocell = max(MaxPhotocell, LDR_read);                                                // Highest LDR measurement (0-1024)
  OutPhotocell = constrain(OutPhotocell, Mem.LowerBrightness, Mem.UpperBrightness);          // Keep result between lower and upper boundery ( 0-1024)
  SumLDRreadshour += LDR_read;    NoofLDRreadshour++;                                        // For statistics LDR readings per hour
  if(print)
  {
   sprintf(sptext,"LDR:%3d", LDR_read);                                 Tekstprint(sptext);
// sprintf(sptext,"LDR:%3d Avg:%3d",analogRead(PhotoCellPin),LDR_read); Tekstprint(sptext);
   sprintf(sptext," (%3d-%3d)",MinPhotocell,MaxPhotocell);              Tekstprint(sptext);
   sprintf(sptext," Out:%3d",OutPhotocell);                             Tekstprint(sptext);
   sprintf(sptext,"=%2d%%",(int)(OutPhotocell / 10));                   Tekstprint(sptext);
                 #ifdef MOD_DS3231
   sprintf(sptext," Temp:%2dC ",get3231Temp()-1);                       Tekstprintln(sptext);   // Correct the reported temperature 
                 #endif // MOD_DS3231
 //  Print_tijd();  
  }
 if(LEDsAreOff) OutPhotocell = 0;
 SetBrightnessLeds((byte) (OutPhotocell/4));                                                  // Set brighness of the LEDs (0-255)
}
//                                                                                            //
//------------------------------------------------------------------------------
//  LED Turn On en Off the LED's
//------------------------------------------------------------------------------
void Play_Lights()
{
 for(int i=0; i<NUM_LEDS; i++) { ColorLeds("",i,i,chromeyellow); ShowLeds(); }
 WhiteOverRainbow(0, 0, 5 );
 WhiteOverRainbow(0,0, 5 );  // wait, whiteSpeed, whiteLength
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
//                                                                                            //
//------------------------------------------------------------------------------
//  LED WhiteOverRainbow
//------------------------------------------------------------------------------
void WhiteOverRainbow(uint8_t wait, uint8_t whiteSpeed, uint32_t whiteLength ) 
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
              ColorLeds("",i,i,0XFFFFFF );
        else  
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
//  LED In- or decrease light intensity value i.e. Slope
//------------------------------------------------------------------------------
void WriteLightReducer(int amount)
{
 int value = Mem.LightReducer + amount;                                                       // Prevent byte overflow by making it an integer before adding
 Mem.LightReducer = (byte) min(value,  255);                                                         // May not be larger than 255
 sprintf(sptext,"Max brightness: %3d%%",Mem.LightReducer);
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
 if (InputString.length()< 1) return;                                                         // If string is empty for some reason
 if (InputString[0] > 47 && InputString[0] <123)                                              // If the first charater is a number or letter
  {
  sprintf(sptext,"**** Length fault ****");                                                   // Default message 
  Serial.println(InputString);
  switch (InputString[0]) 
   {
    case 'C':
    case 'c':
             if(InputString.length() == 1)
               {
                Mem.LightReducer    = SLOPEBRIGHTNESS;
                Mem.UpperBrightness = MAXBRIGHTNESS; 
                Mem.LowerBrightness = LOWBRIGHTNESS;
                Mem.DisplayChoice = Mem.TurnOffLEDsAtHH = Mem.TurnOnLEDsAtHH = Mem.OwnColour = 0;
                for (int i=0;i<24;i++) Mem.NVRAMmem[i] = 2;                                   // Reset LDR readings 
                EEPROM.put(0,Mem);                                                            // Update EEPROM  
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
//                                                                                            //
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
    case 'I':
    case 'i': 
            if (InputString.length() == 1)
            {
             SWversion();
             sptext[0] = 0;                                                                   // Clear sptext
            }
            break;
    case 'L':                                                                                 // Lowest value for Brightness
    case 'l':
             if (InputString.length() < 5)
               {      
                Mem.LowerBrightness = (byte) SConstrainInt(InputString,1,0,255);
                EEPROM.put(0,Mem);                                                            // Update EEPROM  
                sprintf(sptext,"Lower brightness changed to: %d bits",Mem.LowerBrightness);
               }
             break;  
    case 'M':                                                                                 // factor to multiply brighness (0 - 255) with 
    case 'm':
            if (InputString.length() < 6)
               {    
                Mem.UpperBrightness = (int) SConstrainInt(InputString,1,1,1023);
                EEPROM.put(0,Mem);                                                            // Update EEPROM  
                sprintf(sptext,"Upper brightness changed to: %d bits",Mem.UpperBrightness);
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
              EEPROM.put(0,Mem);                                                              // Update EEPROM   
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
                EEPROM.put(0,Mem);                                                            // Update EEPROM   
               }
             else Tekstprintln("Enter Pwwrrggbb");            
             break;
    case 'q':
    case 'Q':  
             if (InputString.length() == 1 )
               {
             Tekstprintln("  Q0= Default colour");
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
                EEPROM.put(0,Mem);                                                            // Update EEPROM  
               }    
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
//                Displaytime();                                                               // Turn on the LEDs with proper time
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
    case 'Y':                                                                                 // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'y':
             if (InputString.length() < 5)
               {    
                Mem.LightReducer = (byte) SConstrainInt(InputString,1,1,255);
                sprintf(sptext,"Slope brightness changed to: %d%%",Mem.LightReducer);
                EEPROM.put(0,Mem);                                                            // Update EEPROM   
               }
              break;      
    case '0':
    case '1':
    case '2':        
             if (InputString.length() == 6 )                                                  // For compatibility input with only the time digits
              {
               I.Hour   = (byte) SConstrainInt(InputString,0,2,0,23);
               I.Minute = (byte) SConstrainInt(InputString,2,4,0,59); 
               I.Second = (byte) SConstrainInt(InputString,4,6,0,59);
               sprintf(sptext,"Time set");  
               SetRTCTime();
               }
     default:
             break;
    }
  }
 Tekstprintln(sptext); 
 Displaytime();                                             
 InputString = "";
 Tekstprintln("");
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

 if (I.Hour == 12 && I.Minute == 0 ) { HET; IS; NOEN; return; }  //&& random(2)==0
 if (I.Hour == 00 && I.Minute == 0 ) { HET; IS; MIDDER; NACHT; return; } //&& random(5)==0

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
                                                                                        //
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
