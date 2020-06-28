// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 ot ARMEGA1284 chip on the PCB board that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05
- DCF77 module DCF-2
- FM Stereo Radio Module RDA5807M RRD-102V2.0  
- Red_MAX7219_8-Digit_LED_Display
- I2C LCD display
A 74HC595 ULN2803APG combination regulates the LEDs by shifting in bits into the 74HC595 LED are turn On or Off
A FT232RL 5.5V FTDI USB to TTL Serial Module can be attached to program te ATMEGA and read the serial port
The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module
The FM-module can be used to read the RDS-time from radio station or to play FM-radio station from within the clock

 Author .: Ed Nieuwenhuys
 Changes.: 0.24 Built in Clock Michelle. Made comparable with Character Clock V106
                Built in Jolanda Dick klok after minor changes. Last stable version for WS2812
 
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
//#define BWClock      // Defines Black and White Word Clock with 2835 / 2538 LEDs 
#define ColourClock    // Defines clour Word Clock with WS2812 LEDs

#define ENCODER_DO_NOT_USE_INTERRUPTS
//#define FMRADIOMOD  // in development time retrieval works. Needs automatic optimal sender search function
//#define BLUETOOTHMOD
//#define DCFMOD
#define ROTARYMOD
//#define LCDMOD
//--------------------------------------------
// ARDUINO Definition of installed language word clock
//--------------------------------------------
// #define NL
// #define UK
// #define DE
// #define FR        // in development
//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
// Get the LCD I2C Library here: 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Move any other LCD libraries to another folder or delete them
// See Library "Docs" folder for possible commands etc.

#include <Wire.h>
                    #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                    #endif  LCDMOD
#include <RTClib.h>
#include <EEPROM.h>
#include "TimeLib.h"       
//#include <Adafruit_NeoPixel.h>
#include <WS2812.h>

                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>     // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>
#ifdef defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
  #define CORE_NUM_INTERRUPT 3
  #define CORE_INT0_PIN   10
  #define CORE_INT1_PIN   11
  #define CORE_INT2_PIN   2
 #endif
                     #endif ROTARYMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     #endif DCFMOD

#define HET      Serial.print(F("Het "));    ColourLeds(0,3,MINColor);   
#define IS      Serial.print(F("is "));     ColourLeds(4,5,SECColor); Is = true;
#define WAS     Serial.print(F("was "));    ColourLeds(6,9,SECColor); Is = false;

#define MVIJF   Serial.print(F("vijf "));   ColourLeds(15,19,LetterColor); 
#define MTIEN   Serial.print(F("tien "));   ColourLeds(10,14,LetterColor);

#define KWART   Serial.print(F("kwart "));  ColourLeds(20,25, LetterColor);
#define VOOR    Serial.print(F("voor "));   ColourLeds(26,30, LetterColor); 

#define OVER    Serial.print(F("over "));   ColourLeds(39,43,LetterColor);
#define PRECIES Serial.print(F("precies "));ColourLeds(31,38,LetterColor);

#define HALF    Serial.print(F("half "));   ColourLeds(44,48, LetterColor);
#define ELF     Serial.print(F("elf "));    ColourLeds(49,52, LetterColor);

#define VIJF    Serial.print(F("vijf "));   ColourLeds(58,62,LetterColor);
#define TWEE    Serial.print(F("twee "));   ColourLeds(53,57,LetterColor);

#define EEN     Serial.print(F("een "));    ColourLeds(63,66, LetterColor);
#define VIER    Serial.print(F("vier "));   ColourLeds(67,71, LetterColor);

#define TIEN    Serial.print(F("tien "));   ColourLeds(79,83,LetterColor);
#define TWAALF  Serial.print(F("twaalf ")); ColourLeds(72,78,LetterColor);

#define DRIE    Serial.print(F("drie "));   ColourLeds(84,88, LetterColor);
#define NEGEN   Serial.print(F("negen "));  ColourLeds(89,94, LetterColor);

#define ACHT    Serial.print(F("acht "));   ColourLeds(99,103,LetterColor);
#define ZES     Serial.print(F("zes "));    ColourLeds(95,98,LetterColor); 
  
#define ZEVEN   Serial.print(F("zeven "));  ColourLeds(104,109, LetterColor);
#define UUR     Serial.print(F("uur "));    ColourLeds(110,112, LetterColor);
#define EDSOFT	Serial.print(F("Edsoft "));

/*
 #define HET     Serial.print(F("Het "));    ColourLeds(0,2,MINColor);   
#define IS      Serial.print(F("is "));     ColourLeds(3,4,SECColor); Is = true;
#define WAS     Serial.print(F("was "));    ColourLeds(5,7,SECColor); Is = false;

#define MVIJF   Serial.print(F("vijf "));   ColourLeds(12,15, LetterColor); 
#define MTIEN   Serial.print(F("tien "));   ColourLeds( 8,11, LetterColor);

#define KWART   Serial.print(F("kwart "));  ColourLeds(16,20, LetterColor);
#define VOOR    Serial.print(F("voor "));   ColourLeds(21,24, LetterColor); 

#define OVER    Serial.print(F("over "));   ColourLeds(32,35, LetterColor);
#define PRECIES Serial.print(F("precies "));ColourLeds(25,31, LetterColor);

#define HALF    Serial.print(F("half "));   ColourLeds(36,39, LetterColor);
#define ELF     Serial.print(F("elf "));    ColourLeds(40,42, LetterColor);

#define VIJF    Serial.print(F("vijf "));   ColourLeds(47,50, LetterColor);
#define TWEE    Serial.print(F("twee "));   ColourLeds(43,46, LetterColor);

#define EEN     Serial.print(F("een "));    ColourLeds(51,53, LetterColor);
#define VIER    Serial.print(F("vier "));   ColourLeds(54,57, LetterColor);

#define TIEN    Serial.print(F("tien "));   ColourLeds(64,67, LetterColor);
#define TWAALF  Serial.print(F("twaalf ")); ColourLeds(58,63, LetterColor);

#define DRIE    Serial.print(F("drie "));   ColourLeds(68,71, LetterColor);
#define NEGEN   Serial.print(F("negen "));  ColourLeds(72,76, LetterColor);

#define ACHT    Serial.print(F("acht "));   ColourLeds(80,83, LetterColor);
#define ZES     Serial.print(F("zes "));    ColourLeds(77,79, LetterColor);
  
#define ZEVEN   Serial.print(F("zeven "));  ColourLeds(84,88, LetterColor);
#define UUR     Serial.print(F("uur "));    ColourLeds(89,91, LetterColor);
//#define EDSOFT  Serial.print(F("Edsoft ")); 
#define NUM_LEDS    92            // How many leds in  strip?  
 */

//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 
#if defined(__AVR_ATmega328P__) 
// Digital hardware constants ATMEGA 328 ----
enum DigitalPinAssignments {

 encoderPinB  = 2,                // left (labeled CLK on decoder)no interrupt pin  
 encoderPinA  = 3,                // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 4,                // switch (labeled SW on decoder)
 LED_PIN      = 5,                // Pin to control colour 2811/2812 leds
 BT_RX        = 6,                // Connects to Bluetooth TX
 BT_TX        = 7,                // Connects to Bluetooth RX
 DCF_PIN     =  8,                // DCFPulse on interrupt  pin
 DCF_LED_Pin  = 9,                // define pin voor AM PM Led
 LEDDataPin   = 10,               // blauw HC595
 LEDStrobePin = 11,               // groen HC595
 LEDClockPin  = 12,               // geel  HC595
 secondsPin   = 13,
 HeartbeatLED = 13};
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
 PhotoCellPin  = 2,               // LDR pin
 EmptyA3       = 3,               //
 SDA_pin       = 4,               // SDA pin
 SCL_pin       = 5};              // SCL pin
# endif

//--------------------------------------------//-------------------------------------------- 
#if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
                                  // Digital hardware constants ATMEGA 1284P ----
enum DigitalPinAssignments {

 BT_RX        = 0,                // Bluetooth RX Connects to Bluetooth TX
 BT_TX        = 1,                // Bluetooth TX Connects to Bluetooth RX
 DCF_PIN     =  2,                // DCFPulse on interrupt  pin
 PWMpin       = 3,                // Pin that controle PWM signal on BC327 transistor to dim light
 PIN04        = 4,                // Pin 4              PB4 PWM
 LED_PIN      = 5,                // Pin to control colour 2811/2812 leds PB5 digital
 PIN06        = 6,                // PIN 6              PB6 PWM 
 PIN07        = 7,                // PIN 7              PB7 PWM  
 PIN08        = 8,                // RX1                PD0 digital
 PIN09        = 9,                // TX1                PD1 digital
 LED10        = 10,               // LED10              PD2 digital
 encoderPinB  = 11,               // left (labeled CLK on decoder)no interrupt pin
 encoderPinA  = 12,               // right (labeled DT on decoder)on interrupt  pin
 clearButton  = 13,               // switch (labeled SW on decoder)  
 DCF_LED_Pin  = 14,               // define pin voor AM PM Led
 HeartbeatLED = 15,               // LED15                                           PD7 PWM
 SCL_pin      = 16,               // SCL pin       PC0 interrupt
 SDA_pin      = 17,               // SDA pin       PC1 interrupt
 PIN18        = 18,               // Empty         PC2 digital
 LED19        = 19,               // LED19         PC3 digital
 LEDDataPin   = 20,               // blauw HC595
 LEDStrobePin = 21,               // groen HC595
 LEDClockPin  = 22,               // geel  HC595
 secondsPin   = 0};//23
                                  // Analogue hardware constants ----
enum AnaloguePinAssignments {
  EmptyA0      = 24,              // Empty
  EmptyA1      = 25,              // Empty
  PhotoCellPin = 26,              // LDR pin
  EmptyA3      = 27,              // Empty
  EmptyA4      = 28,              // Empty
  EmptyA5      = 29,              // Empty
  EmptyA6      = 30};             // Empty
 # endif 

//--------------------------------------------
// LED
//--------------------------------------------
#define NUM_LEDS    113            // How many leds in  strip?
#define BRIGHTNESS  200           // BRIGHTNESS 0 - 255
#define UPDATES_PER_SECOND 20
WS2812 RGBstrip(NUM_LEDS);       // Initialyse RGB strip
// Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
//CRGBPalette16 currentPalette;
//TBlendType    currentBlending;
//extern const TProgmemPalette16 myRedWhiteBluePalette_p PROGMEM;
byte BrightnessCalcFromLDR = BRIGHTNESS;
int ToggleEdsoft = 1;             // flash led every hour

//uint32_t leds[NUM_LEDS];              // Define the array of leds
uint32_t MINColor      = 0XFFDD00;
uint32_t SECColor      = 0XFFDD00;
uint32_t LetterColor   = 0XFFDD00;
uint32_t DefaultColor  = 0XFFDD00;         // Geel
uint32_t OwnColour     = 0X234588;
uint32_t WhiteColour   = 0XFFFFFF;
uint32_t HourColor[24] ={0XFFFFFF,0X5EFF00,0XFFD500,0X00E6FF,0XFF6200,0X88FF88,
                         0XAE00FF,0X00C8FF,0X660099,0XFFD000,0X00FF00,0XFF0050,
                         0XFF0000,0XFFFF00,0XFF4000,0XFF0088,0XF7FF00,0XFF00BF,
                         0X00FF00,0X00F2A0,0X6FFF00,0X0073FF,0XF200FF,0X0000FF };
                    
//--------------------------------------------
// KY-040 ROTARY
//-------------------------------------------- 
                          #ifdef ROTARYMOD                         
Encoder myEnc(encoderPinA, encoderPinB);              // Use digital pin  for encoder
                          #endif ROTARYMOD      
long Looptime = 0;
byte RotaryPress  = 2;
unsigned long RotaryPressTimer = 0;

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
float LightReducer    = 0.80;       // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
byte  LowerBrightness = 10;         // Lower limit of Brightness ( 0 - 255)
int   OutPhotocell;                 // stores reading of photocell;
int   MinPhotocell    = 1024;       // stores minimum reading of photocell;
int   MaxPhotocell    = 1;          // stores maximum reading of photocell;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define MAXTEXT 55
static unsigned long msTick;       // the number of millisecond ticks since we last incremented the second counter
int  count; 
int  Delaytime = 200;
byte Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte lastminute = 0, lasthour = 0, sayhour = 0;
byte Toggle_HetWasIs    = 1;       // Turn On/Off HetIsWas lights
byte Toggle_HetWasIsUit = 0;       // Turn off HetIsWas after 10 sec
byte SecPulse           = 0;       // give a pulse to the Isecond led
byte Demo = false;
byte Zelftest           = false;
byte hbval              = 128;
byte hbdelta            = 8;
String SerialString;
byte Is = true;                    // toggle of displaying Is or Was

const byte MenuItems = 12;         // sentences, rows, in menu
const char menu[MenuItems][MAXTEXT] PROGMEM =  {
 "JoDi woordklok No 20 26-dec-2015",
 "Voer in:",
 "Tijd uumm (1721) of uummss (172145)",
// "Enter A for normal display",
// "Enter B to suspress Het Is Was in display",
// "Enter C om Het Is Was na 10s uit te zetten",
 "Dddmmjjjj (D15122017 voor 15 December 2017)",
// "Enter G for DCF-signalinfo on display",
 "Lnn       (L5) Min licht intensiteit ( 1 - 255)",
 "Mnn       (M90)Max licht intensiteit (1% - 250%)",
 "Pnnnnnn   (P2345DD) voor eigen RGB-kleur (n=0-F)",
 "Qn        (P5) voor palettekeuze (Q1-5)",
 "I         info",
 "X         Demo mode",
 "Z         Zelftest",
 "Ed Nieuwenhuys    V024 Apr-2018" };

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
RTC_DS3231 RTC;    //RTC_DS1307 RTC;  
DateTime Inow;

//--------------------------------------------
// BLUETOOTH
//--------------------------------------------                                     
#ifdef BLUETOOTHMOD               // Bluetooth ---------------------
SoftwareSerial Bluetooth(BT_RX, BT_TX);    // RX, TX
String BluetoothString;
#endif BLUETOOTHMOD 
//--------------------------------------------
// DCF-2 DCF77 MODULE
//--------------------------------------------
byte DCF_signal = 50;                       // is a proper time received?
bool SeeDCFsignalInDisplay = false;         // if ON then the display line HET IS WAS will show the DCF77-signal received

#ifdef DCFMOD                               // DCF77 ------------------------------

#if defined(__AVR_ATmega328P__) 
#define DCF_INTERRUPT 0                     // DCF Interrupt number associated with DCF_PIN
#endif
#if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
#define DCF_INTERRUPT 2                     // DCF Interrupt number associated with DCF_PIN
#endif
time_t tijd;
DCF77 DCF = DCF77(DCF_PIN,DCF_INTERRUPT,LOW);
#endif DCFMOD 
                    #ifdef LCDMOD
//--------------------------------------------
// LCD Module
//--------------------------------------------
LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7); // 0x27 is the I2C bus address for an unmodified backpack
                    #endif 
//----------------------------------------
// Common
//----------------------------------------
char sptext[MAXTEXT+2];                     // for common print use    

//--------------------------------------------
// COLOURS
//--------------------------------------------   
#define DEFAULTCOLOUR  0
#define HOURLYCOLOUR   1           
#define NOFUSSCOLOUR   2
#define DEMOCOLOUR     3
#define OWNHETISCLR    4
//byte DisplayChoice = DEFAULTCOLOUR;
//byte DisplayChoice = NOFUSSCOLOUR;
//byte DisplayChoice = DEMOCOLOUR;
byte DisplayChoice = HOURLYCOLOUR;
 // End Definitions  ---------------------------------------------------------


//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
SerialCheck();
if(Demo)  Demomode();
else if (Zelftest) Selftest(); 
else
 { 
                              #if defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284__)
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
  // initialise the hardware // initialize the appropriate pins as outputs:  
 Serial.begin(9600);     
                                  #ifdef BLUETOOTHMOD   
 Bluetooth.begin(9600);                              // set up the Bluetooth port to 9600 baud 
 Serial.println("Bluetooth enabled");
                                  #endif BLUETOOTHMOD                                   
 pinMode(DCF_LED_Pin,  OUTPUT);
 pinMode(secondsPin,   OUTPUT );
                                  #ifdef ROTARYMOD   
 pinMode(encoderPinA,  INPUT_PULLUP);
 pinMode(encoderPinB,  INPUT_PULLUP);  
 pinMode(clearButton,  INPUT_PULLUP); 
 Serial.println("Rotary enabled"); 
                                 #endif ROTARYMOD 
 pinMode(DCF_PIN,      INPUT_PULLUP);
// FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS).setCorrection( TypicalSMD5050 );
// FastLED.setTemperature(CoolWhiteFluorescent );                 // CoolWhiteFluorescent); //UncorrectedTemperature ); DirectSunlight
// LEDS.setBrightness(BRIGHTNESS);// currentPalette = RainbowColors_p;// strip.begin();
 RGBstrip.setOutput(LED_PIN);                                       // Set strip dat output to pin LED pin
 RGBstrip.sync();                                                   // Initialize all pixels to 'off'
 RGBstrip.setBrightness(BRIGHTNESS);  
                                               // setup the serial port to 9600 baud 

                                  #ifdef FMRADIOMOD     
 Setup_FMradio();                                    // start the FM-radio
 Serial.println("FM-radio enabled");
                                  #endif FMRADIOMOD 
                          #ifdef DCFMOD
 DCF.Start();                                                       // start the DCF-module
 Serial.println("DCF enabled");
                          #endif DCFMOD
 Wire.begin();                                                      // start the wire communication I2C
 RTC.begin();                                                       // start the RTC-module
// analogWrite(PWMpin, BrightnessCalcFromLDR);                      // the duty cycle: between 0 (lights off) and 255 (light full on).  
                          #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                         // activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Serial.println("DCF enabled");
                          #endif LCDMOD
 GetTijd(1);                                         // Get the time and print it to serial 
 DateTime now = RTC.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));   // following line sets the RTC to the date & time this sketch was compiled
   RTC.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 if (EEPROM.read(0) <3 || EEPROM.read(0) > 200)    EEPROM.write(0,(int)(LightReducer * 100));    // default intensity for this clock
 if (EEPROM.read(1) <1 || EEPROM.read(1) > 100)    EEPROM.write(1, LowerBrightness);             // default Lower Brightness for this clock
 LightReducer  = ((float) EEPROM.read(0) / 100);                    // store it is the work variable
 LowerBrightness = EEPROM.read(1);                                  // store it is the work variable
 Looptime = millis();                                               // Used in KY-040 rotary
 msTick = millis(); 
 SWversion();                                                       // Display the version number of the software

 GetTijd(0);                                                        // Get the time and print it to serial
 SetMinuteColour();                                                 // Set the colour per minute of 'HET'
                     #ifdef ROTARYMOD
 myEnc.write(0);                                                    // Clear Rotary encode buffer
                     #endif ROTARYMOD
// Selftest();                                                        // Play the selftest
} 
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
 for (int i = 0; i < MenuItems; i++)   {strcpy_P(sptext, menu[i]);   Tekstprintln(sptext);  }
 for (int n=0; n<52; n++) {Serial.print(F("_"));} Serial.println();
 sprintf(sptext,"Max brightness: %3ld%%   ",(long)(LightReducer*100)) ;
 Tekstprintln(sptext);
 sprintf(sptext,"Min brightness: %3ld bits   ",(long)LowerBrightness) ;
 Tekstprintln(sptext);
 for (int n = 0; n < 52; n++) {Serial.print(F("_"));} Serial.println();
}

//--------------------------------------------
// CLOCK Demo mode
//--------------------------------------------
void Demomode(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
  {    
   msTick = millis();                                               // second++; 
   digitalWrite(secondsPin,HIGH);                                   // turn ON the second on pin 13
   ++SecPulse;                                                      // second routine in function DimLeds
   if( ++Iminute >59) { Iminute = 0; Isecond = 0; Ihour++;}
    if(     Ihour >24)  Ihour = 0;
   DimLeds(false);
   Displaytime();
   Serial.println("");
  }
}

//--------------------------------------------
// CLOCK common print routines
//--------------------------------------------
void Tekstprint(char tekst[])
{
 Serial.print(tekst);    
                          #ifdef BLUETOOTHMOD   
 Bluetooth.print(tekst);  
                          #endif BLUETOOTHMOD
}
void Tekstprintln(char tekst[])
{
 Serial.println(tekst);    
                          #ifdef BLUETOOTHMOD
 Bluetooth.println(tekst);
                          #endif BLUETOOTHMOD
}
//--------------------------------------------
// CLOCK Update routine done every second
//--------------------------------------------
void EverySecondCheck(void)
{
  if ( millis() - msTick >50)   digitalWrite(secondsPin,LOW);       // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
   {    
    msTick = millis();                                              // second++; 
    digitalWrite(secondsPin,HIGH);                                  // turn ON the second on pin 13
    ++SecPulse;                                                     // second routine in function DimLeds
    GetTijd(0);                                                     // synchronize time with RTC clock      
                    #ifdef LCDMOD
   Print_tijd_LCD();
                    #endif LCDMOD  
   if ((Toggle_HetWasIsUit == 2) && (Isecond > 10)) Toggle_HetWasIs = 0; 
     else Toggle_HetWasIs = 1;                                       // HET IS WAS is On
   if(Isecond % 30 == 0) DimLeds(true);                              // Led Intensity Control + seconds tick print every 30 seconds   
     else                DimLeds(false);
   if ((Toggle_HetWasIsUit == 2) && (Isecond == 11)) Displaytime();  // turn Leds OFF on second == 11
   SetSecondColour();                                                // Set the colour per second of 'IS' and 'WAS' 
   if(Iminute == 0 && Isecond <9)
    { 
     ToggleEdsoft = Isecond % 2;               // ToggleEdsoft bocomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
     Serial.println(ToggleEdsoft);
     Displaytime();                           // ----------------------------- dit moet iets laten knipperen
    }
                     #ifdef DS1820
   DS1820read();
                     #endif DS1820  
  }
 }
//--------------------------------------------
// CLOCK Update routine done every minute
//--------------------------------------------
 void EveryMinuteUpdate(void)
 {
 if (Iminute != lastminute)                                         // Show time every minute
  { 
   lastminute = Iminute;
   Displaytime();
   Print_RTC_tijd();
   SetMinuteColour();                                               // Set the colour per minute of 'HET'
   DCF_signal--;
   DCF_signal = constrain( DCF_signal,1,99);
  } 
 if (Ihour != lasthour) {lasthour = Ihour;}
 }
                            #ifdef BLUETOOTHMOD
//--------------------------------------------
// CLOCK check for Bluetooth input
//--------------------------------------------                           
void BluetoothCheck(void)
{ 
 long looptimeBT = millis();  //avoid an hangup in this loop  
 while (Bluetooth.available() and (millis() - looptimeBT < 10000) )
  {
   delay(3); 
   char c = Bluetooth.read();
   Serial.print(c);
   if (c>31 && c<128) BluetoothString += c;
   else c = 0;     // delete a CR
  }
 if (BluetoothString.length()>0)  
      ReworkInputString(BluetoothString);            // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD
                           
                           #ifdef DCFMOD
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 time_t DCFtime = DCF.getTime();                      // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   Tekstprint("DCF: Time is updated ----->  ");
   DCF_signal+=2;
   setTime(DCFtime); 
   RTC.adjust(DCFtime);
//   digitalClockDisplay(); *************************
  }
 bool LHbit = digitalRead(DCF_PIN);
 digitalWrite(DCF_LED_Pin, 1 - LHbit );               // write inverted DCF pulse to LED on board 
 if (SeeDCFsignalInDisplay == true)
  {
   Toggle_HetWasIs = LHbit;
   Display1 |= (Toggle_HetWasIs<<0);                  // Turn off the  HET IS WAS LEDs
   Displaytime();
  }
  DCF_signal = constrain(DCF_signal,0,99);            // DCF_signal <100
} 
                           #endif DCFMOD 
//--------------------------------------------
// CLOCK check for serial input
//--------------------------------------------
void SerialCheck(void)
{
 while (Serial.available())
  {
   delay(3);  
   char c = Serial.read();
   if (c>31 && c<128) SerialString += c;                            // allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
}
void heartbeat() 
{
  static unsigned long last_time = 0;
  unsigned long now = millis();
  if ((now - last_time) < 40)    return;
  last_time = now;
  if (hbval > 230 || hbval < 20 ) hbdelta = -hbdelta; 
  hbval += hbdelta;
  analogWrite(HeartbeatLED, hbval);
}
//------------------------ KY-040 rotary encoder ------------------------- 
//--------------------------------------------
// KY-040 ROTARY check if the rotary is moving
//--------------------------------------------
                           #ifdef ROTARYMOD
void RotaryEncoderCheck(void)
{
// If button pressed, 60 sec after start ATMEGA, then there are 60 seconds to adjust the light intensity. 
// RotaryPressTimer is the time in millisec after start ATMEGA 
 long encoderPos = myEnc.read();
 if ( (encoderPos) && ( (millis() - Looptime) >200))                  // if rotary turned debounce 0.2 sec
  {   
   Serial.print(F("--------> Index:"));   Serial.println(encoderPos);
   if  (encoderPos >0)                                                // increase the MINUTES
    {
     if ( millis() > 60000 && (millis() - RotaryPressTimer) < 60000)
         { WriteLightReducer(0.05); }                                 // If time < 60 sec then adjust light intensity factor
     else 
     {
      if( ++Iminute >59) { Iminute = 0; Isecond = 0;  }
      SetRTCTime(); 
      Looptime = millis();                                             // Set encoder pos back to 0   
      myEnc.write(0);  
     }     
    }
   if  (encoderPos <0)                                                // increase the HOURS
    {
     if (millis() > 60000 &&  (millis() - RotaryPressTimer) < 60000) 
         { WriteLightReducer(-0.05); }    // If time < 60 sec then adjust light intensity factor
     else
      { 
      if( ++Ihour >23) { Ihour = 0; }
      SetRTCTime();
      Looptime = millis();                                             // Set encoder pos back to 0 
      myEnc.write(0);   
      }  
    }                                                
  }
 if (digitalRead(clearButton) == LOW )                                // set the time by pressing rotary button
                          #ifdef BWClock  
  { 
    delay(200);
    RotaryPressTimer =  millis();                                     // If time < 60 sec then adjust light intensity factor
    Toggle_HetWasIsUit++;
                          #ifdef DCFMOD    
    if (Toggle_HetWasIsUit == 3)  { Toggle_HetWasIsUit = -1; SeeDCFsignalInDisplay = true; }
                          #endif DCFMOD
    if (Toggle_HetWasIsUit >= 3)  { Toggle_HetWasIsUit = 0 ; SeeDCFsignalInDisplay = false;}
    if (Toggle_HetWasIsUit == 0)  { Toggle_HetWasIs = 1;} // On
    if (Toggle_HetWasIsUit == 1)  { Toggle_HetWasIs = 0;} // Off
    if (Toggle_HetWasIsUit == 2)  { Toggle_HetWasIs = 0; Play_Lights(); } // Off after 10 sec
    
    Serial.print(F("Toggle_HetWasIsUit: "));   Serial.println(Toggle_HetWasIsUit);
    Serial.print(F("Toggle_HetWasIs: "));      Serial.println(Toggle_HetWasIs);    
    Displaytime();
   }
                          #endif BWClock
                          #ifdef ColourClock
   { 
    delay(300);
    RotaryPressAction();                                               // increases RotaryPress and selects next DisplayChoice
    myEnc.write(0);                                                    // Set encoder pos back to 0  
    Looptime = millis();                                             // Set encoder pos back to 0       
  }
                          #endif ColourClock 
 myEnc.write(0);

 }
                          #endif ROTARYMOD
                          #ifdef ColourClock
//---------------------------------------------
// CLOCK change display mode in colour clock
//--------------------------------------------
void RotaryPressAction(void)
{
    if (RotaryPress > OWNHETISCLR) RotaryPress=0;
    DisplayChoice = RotaryPress;
    switch (RotaryPress++) 
    {
      case 0:  DisplayChoice = DEFAULTCOLOUR; break;
      case 1:  DisplayChoice = HOURLYCOLOUR;  break;        
      case 2:  DisplayChoice = NOFUSSCOLOUR;  break;
      case 3:  DisplayChoice = DEMOCOLOUR;    break;
      case 4:  DisplayChoice = OWNHETISCLR;   break;
      case 5:  RotaryPress   = 0;             break; 
     default:  RotaryPress   = 0;   
    }
    GetTijd(0);                                                        // synchronize time with RTC clock
    SetMinuteColour();                                                 // Set the colour per minute of 'HET'
    Displaytime();
}
                           #endif ColourClock 
//--------------------------------------------
// CLOCK Self test sequence
//--------------------------------------------
void Selftest(void)
{ 
 //  Serial.print(F("Self test in : "));
 // start by clearing the display to a known state
  GetTijd(1); //Prints time in Serial monitor
  LedsOff(); 
  HET;     Laatzien();
  IS;      Laatzien();
  WAS;     Laatzien();
  MVIJF;   Laatzien();
  MTIEN;   Laatzien();
  KWART;   Laatzien(); 
  VOOR;    Laatzien();
  OVER;    Laatzien();
  PRECIES; Laatzien(); 
  HALF;    Laatzien();
  ELF;     Laatzien();  
  VIJF;    Laatzien();
  TWEE;    Laatzien();  
  EEN;     Laatzien(); 
  VIER;    Laatzien();
  TIEN;    Laatzien();
  TWAALF;  Laatzien();
  DRIE;    Laatzien();
  NEGEN;   Laatzien(); 
  ACHT;    Laatzien();
  ZES;     Laatzien(); 
  ZEVEN;   Laatzien();  
  UUR;     Laatzien();
// EDSOFT;  Laatzien();
  Tekstprintln("*"); 
  Play_Lights();     
  Displaytime();
}
// -------------------------- End Selftest

//--------------------------- Time functions --------------------------
void Displaytime(void)
{
CheckColourStatus();                         // Check colours to display as chosen by toggle rotary switch        
LedsOff();                                   // start by clearing the display to a known state

 HET;                                        // HET lights are always on colour changes 
 switch (Iminute)
 {
  case  0: IS;  PRECIES; UUR; break;
  case  1: IS;  UUR;  break;
  case  2: 
  case  3: WAS; UUR; break;
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
  case 59: IS;  UUR; break;
}
//if (Ihour >=0 && hour <12) digitalWrite(AMPMpin,0); else digitalWrite(AMPMpin,1);

sayhour = Ihour;
if (Iminute > 18 ) sayhour = Ihour+1;
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
WriteLEDs();
}

//--------------------------------------------
// DS3231 Get time from DS3231
//--------------------------------------------
void GetTijd(byte printit)
{
 Inow =    RTC.now();
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
 RTC.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
 GetTijd(0);                               // synchronize time with RTC clock
 Displaytime();
 Print_tijd();
}
//--------------------------------------------
// DS3231 Get temperature from module
//--------------------------------------------
int get3231Temp(void)
{
 byte tMSB, tLSB;
 int temp3231;
  
  Wire.beginTransmission(DS3231_I2C_ADDRESS);    //temp registers (11h-12h) get updated automatically every 64s
  Wire.write(0x11);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
 
  if(Wire.available()) 
  {
    tMSB = Wire.read();                          //2's complement int portion
    tLSB = Wire.read();                          //fraction portion 
    temp3231 = (tMSB & B01111111);               //do 2's math on Tmsb
    temp3231 += ( (tLSB >> 6) * 0.25 ) + 0.5;    //only care about bits 7 & 8 and add 0.5 to round off to integer   
  }
  else {  temp3231 = -273; }   
  return (temp3231);
}

// ------------------- End  Time functions 

// --------------------Colour Clock Light functions -----------------------------------
//--------------------------------------------
//  LED Set colour for LED
//--------------------------------------------
void ColourLeds(int FirstLed, int LastLed, uint32_t RGBColor)
{
 int n;
 for (n = FirstLed; n <= LastLed; n++)
   {
      //strip.setPixelColor(n,RGBColor);
      RGBstrip.setRGB(n,RGBColor);
     // RGBstrip.setHSV(n,(byte)((Iminute+Ihour*60) / 5.65),255,255);
   }
}

//--------------------------------------------
//  LED Clear display settings of the LED's
//--------------------------------------------
void LedsOff(void) 
{ 
 int n;
 for (n = 0; n <= NUM_LEDS; n++)
   {
    //  strip.setPixelColor(n,0);
      RGBstrip.setRGB(n,0);
   }
//  FastLED.clear();
}
//--------------------------------------------
//  LED Turn On the LED's
//--------------------------------------------
void WriteLEDs(void) 
{ 
  ShowLeds();    //Push data in LED strip to commit the changes
}
//--------------------------------------------
// LED Turn On en Off the LED's
//--------------------------------------------
void Laatzien()
{ 
  WriteLEDs();
  delay(Delaytime);
  LedsOff(); 
}

//--------------------------------------------
//  LED Push data in LED strip to commit the changes
//--------------------------------------------
void ShowLeds(void)
{
 // FastLED.show();
 // strip.show();
   RGBstrip.sync();
}
//--------------------------------------------
//  LED Set brighness of LEDs
//--------------------------------------------  
void SetBrightnessLeds( byte Bright)
{
     RGBstrip.setBrightness(Bright);
     RGBstrip.sync();
//    strip.setBrightness(Bright);  
//    strip.show();
   //LEDS.setBrightness(Bright);
}

//--------------------------------------------
//  LED function to make RGB color
//-------------------------------------------- 
uint32_t FuncCRGB(uint32_t Red, uint32_t Green, uint32_t Blue)
{
return ((uint32_t)256 * (uint32_t)256 * Red + (uint32_t)256 * Green + Blue);
}

//--------------------------------------------
//  LED Set display colors of clock
//-------------------------------------------- 
void CheckColourStatus(void)
{
 if (DisplayChoice == DEFAULTCOLOUR) {LetterColor = DefaultColor;     }   
 if (DisplayChoice == NOFUSSCOLOUR ) {LetterColor = WhiteColour; MINColor = WhiteColour;  SECColor = WhiteColour;    } // all white
 if (DisplayChoice == HOURLYCOLOUR ) {LetterColor = HourColor[Ihour]; }                                                // A colour every hour
 if (DisplayChoice == DEMOCOLOUR )   {LetterColor = OwnColour;   MINColor = OwnColour;   SECColor = OwnColour;       } // all white 
 if (DisplayChoice == OWNHETISCLR )  {LetterColor = OwnColour;        }                                                // own colour except HET IS WAS  
}
//--------------------------------------------
//  LED Set minute color
//--------------------------------------------
void SetMinuteColour()                                 // Set the colour per minute of 'HET'
{
 MINColor = FuncCRGB((uint32_t)(15 + (Iminute * 4)), (uint32_t)(255 - Iminute * 4), 0);  
}
//--------------------------------------------
//  LED Set second color
//--------------------------------------------
void SetSecondColour()                                 // Set the colour per second of 'IS' and 'WAS'
{
 byte Bluecolor = 0 ;
 SECColor = (uint32_t) FuncCRGB( (uint32_t)(15 + ((uint32_t) Isecond * 4)),   (uint32_t)(255 - (uint32_t)(Isecond * 4)), (uint32_t)Bluecolor);
 CheckColourStatus();
 if (Is) { ColourLeds(4,5,SECColor);  ColourLeds(6,9,0);  }
   else    { ColourLeds(4,5,0);         ColourLeds(6,9,SECColor);  }  

 ShowLeds();
//   Serial.print(": ");
//   Serial.print(Isecond);
//   Serial.print(": ");
//   Serial.print((uint32_t)256 * (uint32_t)256* (uint32_t)(15 + (Isecond * 4)),HEX);
//   Serial.print(": ");
//   Serial.print((uint32_t)256 * (uint32_t)(255 - (uint32_t)Isecond * 4),HEX);
//   Serial.print(": ");
//   Serial.println(SECColor,HEX);
}
//--------------------------------------------
//  LED Dim the leds by PWM measured by the LDR and print values
//--------------------------------------------
void DimLeds(byte print) 
{
 int Temp;                                                                                                 
 if (SecPulse)                                   // if a second has passed 
 {
  int LDR_read = analogRead(PhotoCellPin);      // Read lightsensor
  int BrCalc, Temp;
  OutPhotocell = (int) (LightReducer * sqrt( (float) 63.5 * (float) constrain(LDR_read,1,1023))); // Linear --> hyperbolic with sqrt
  MinPhotocell = MinPhotocell > LDR_read ? LDR_read : MinPhotocell;
  MaxPhotocell = MaxPhotocell < LDR_read ? LDR_read : MaxPhotocell;
  BrightnessCalcFromLDR = constrain(OutPhotocell, LowerBrightness , 255);                   // filter out of strange results
  BrCalc = (int) (BrightnessCalcFromLDR/2.55);
  if(print)
  {
   Temp = get3231Temp()-2; 
   sprintf(sptext,"LDR:%d (%d-%d)->%d=%d%% T=%dC ",LDR_read, MinPhotocell, MaxPhotocell, OutPhotocell,BrCalc,Temp);
   Tekstprint(sptext);
   Print_tijd();
  }
                                       #ifdef BWClock
  analogWrite(PWMpin, BrightnessCalcFromLDR);  // write PWM                  *** naar functie SetBrightnessLeds
                                       #endif BWClock
                                       #ifdef ColourClock
  SetBrightnessLeds(BrightnessCalcFromLDR);  //LEDS.setBrightness(BRIGHTNESS);    *** naar functie SetBrightnessLeds 
                                         #endif ColourClock                                  
 }
 SecPulse = 0;
}
//--------------------------------------------
//  LED Turn On en Off the LED's
//--------------------------------------------
void Play_Lights()
{
  for (int n=0; n<12; n++) 
  {
    for (int i=0; i<NUM_LEDS; i++) { ColourLeds(i,i+1,HourColor[n]); ShowLeds();   }
      // FastLED.show();
  }
  LedsOff();
}

void FillLEDsFromPaletteColors( uint8_t colorIndex)
{
//  uint8_t brightness = BrightnessCalcFromLDR; 
//  for( int i = NUM_LEDS; i >=0; --i) {
//    leds[i] = ColorFromPalette( currentPalette, colorIndex, brightness, currentBlending);
//    colorIndex += 2; //3;
//  }
}

void ChangePalettePeriodically()
{
  SetBrightnessLeds(BRIGHTNESS);
// { currentPalette =  RainbowColors_p;           currentBlending = NOBLEND;   }
//    if( second ==  0)  { currentPalette = RainbowColors_p;         currentBlending = BLEND;   }
//    if( second == 20)  { currentPalette = RainbowStripeColors_p;   currentBlending = BLEND;   }
//    if( second == 10)  { currentPalette = CloudColors_p;           currentBlending = BLEND;   }
//    if( second == 20)  { currentPalette = PartyColors_p;           currentBlending = BLEND;   }
//    if( second == 50)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = NOBLEND; }
 //   if( second == 30)  { currentPalette = myRedWhiteBluePalette_p; currentBlending = BLEND;   }
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
void WriteLightReducerEeprom(float value)
{
 LightReducer = value;
 if (LightReducer < 0.01 ) LightReducer = 0.01;
 if (LightReducer > 2.50 ) LightReducer = 2.50;                      // May not be larger than 2.55 (value*100 = stored as byte 
 EEPROM.write(0, (int) (LightReducer * 100));                        // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3ld%%",(long)(LightReducer*100)) ;
 Tekstprintln(sptext);
// Serial.print(millis() - RotaryPressTimer); Serial.print(" msec ------- ");
// Serial.print(F("LightReducer: ")); Serial.print(LightReducer * 100); Serial.println("%");
}

//--------------------------------------------
//  LED Write lowest allowable light intensity to EEPROM
//--------------------------------------------
void WriteLowerBrightness(int LowerBrightness)
{
 if (LowerBrightness < 1 ) LowerBrightness =  1;                     // Range between 1 and 100
 if (LowerBrightness > 255) LowerBrightness = 255;
 EEPROM.write(1, LowerBrightness);                                   // Default Lower Brightness for this clock
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
 char strhx[8];
 float ff;
 int Jaar;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64 )                                           // Does the string start with a letter?
  {
  int val = InputString[0];
  int FMfreq;
  
  Tekstprintln(sptext);
  switch (val)
   {
    case 'A':
    case 'a':   
             Toggle_HetWasIsUit = 0; Toggle_HetWasIs = 1;             // All tekst displayed  
             Tekstprintln("All tekst displayed");
             break;
    case 'B':
    case 'b':    
             Toggle_HetWasIsUit = 1; Toggle_HetWasIs = 0;             // Het Is Was turned off
             Tekstprintln("Het Is Was turned off");
             break;
    case 'C':
    case 'c':    
            Toggle_HetWasIsUit = 2; Toggle_HetWasIs = 0;              // Het Is Was Off after 10 sec
            Play_Lights();                     
            Tekstprintln("Het Is Was Off after 10 sec");                          
            break;
    case 'D':
    case 'd':  
            if (InputString.length() == 9 )
             {
              temp   = InputString.substring(1,3);     Iday = (byte) temp.toInt(); 
              temp   = InputString.substring(3,5);   Imonth = (byte) temp.toInt(); 
              temp   = InputString.substring(5,9);     Jaar =  temp.toInt(); 
              Iday   = constrain(Iday  , 0, 31);
              Imonth = constrain(Imonth, 0, 12); 
              Jaar  = constrain(Jaar , 1000, 9999); 
              RTC.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
              Tekstprintln(sptext);
             }
             else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");

            break;
    case 'E':
    case 'e':
            Tekstprintln("Entered an E");
            break;       
                     #ifdef FMRADIO                        
    case 'F':
    case 'f':
            //set FM frequency
             temp = InputString.substring(1);
             FMfreq = temp.toInt();
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
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             temp = InputString.substring(1);
             LowerBrightness = temp.toInt();
             WriteLowerBrightness(LowerBrightness);
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm':    
             temp = InputString.substring(1);
             ff = (float)(temp.toInt()) / 100;
             WriteLightReducerEeprom(ff);
             break;
    case 'P':
    case 'p':  
            if (InputString.length() == 7 )
              {
               temp = InputString.substring(1,7);
               OwnColour = hexToDec(temp) ;                         // display letter color
               LetterColor = OwnColour;                             // display letter color 
               DisplayChoice = DEMOCOLOUR;
               Displaytime();
               }
             else Tekstprintln("**** Length fault. Enter PFFFFFF ****");            
            break;
    case 'q':
    case 'Q':  
            if (InputString.length() == 2 )
              {
                temp   = InputString.substring(1,3);     
                RotaryPress = (byte) temp.toInt(); 
                RotaryPress--; 
                RotaryPressAction();                                  // increases RotaryPress and selects next DisplayChoice
               }
             else Tekstprintln("**** Length fault. Enter PFFFFFF ****");            
            break;    
    case 'I':
    case 'i':   
            SWversion();
//            Display1=255;   Display2=255;   Display3=255;  Laatzien();
//            Display1=0;     Display2=0;     Display3=0;    Laatzien();
            break;
                     #ifdef FMRADIO
    case 'R':
    case 'r':
            RDA5807_Report();
            break;
    case 'S':
    case 's':
            RDA5807_ReadStatus();            
            break;
    case 'T':
    case 't':    
            RDA5807_RDS_Dump();                                          
            break;             
                     #endif FMRADIO 
    case 'X':
    case 'x':    
            Demo = 1 - Demo;                                          // toggle Demo mode
            Play_Lights();
            GetTijd(0);  
            Displaytime();
            sprintf(sptext,"Demo mode: %d",Demo);
            Tekstprintln(sptext);
            break; 
    case 'Z':
    case 'z':
            Zelftest = 1 - Zelftest;                                       
            break;          
    default:
            break;
   }
   Displaytime();
   InputString = "";
 }
 else if (InputString.length() > 3 && InputString.length() <7 )
 {
  temp = InputString.substring(0,2);   
  Ihour = temp.toInt(); 
  if (InputString.length() > 3) { temp = InputString.substring(2,4); Iminute = temp.toInt(); }
  if (InputString.length() > 5) { temp = InputString.substring(4,6); Isecond = temp.toInt(); }
  
  SetRTCTime();
 }
 InputString = "";
 temp = "";
}

//--------------------------------------------
//  CLOCK Convert Hex to uint32
//--------------------------------------------
uint32_t hexToDec(String hexString) 
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
