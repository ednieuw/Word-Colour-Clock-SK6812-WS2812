// =============================================================================================================================
/* 
This Arduino code controls the ATMEGA328 ot ARMEGA1284 chip on the PCB board that controls the LED strips of the Word Clock
This source contains code for the following modules:  
- RTC DS3231 ZS-042 clock module
- KY-040 Keyes Rotary Encoder
- LDR light sensor 5528
- Bluetooth RF Transceiver Module HC05
- DCF77 module DCF-2
- I2C LCD display

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
************************************************************************************
The HC05 or HM-10 Bluetooth module is used to read and write information or instructions to the clock
The DCF77 module can be attached to adjust the time to the second with German longwave time signal received by the module

 Author .: Ed Nieuwenhuys
 Changes.: 0.24a Added locking rotary after 60 seconds  
 Changes.: 0.24b Store own colour in EEPROM
 Changes.: Character_Clock_WS2812_Sep2020  Version with WS2812 library special for WS2812 LEDs
 Changes.: Character_Clock_WS2812_Nov2020  Typos
 Changes.: Character_Clock_WS2812_Dec2020  Removed ATMEGA1284. Added Print own colour uint32_t as hex in menu.sizeof(menu) / sizeof(menu[0]). Added PrintLine sub routine
 */
// ===============================================================================================================================

//--------------------------------------------
// ARDUINO Definition of installed modules
//--------------------------------------------
#define BLUETOOTHMOD
//#define DCFMOD
#define ROTARYMOD
//#define LCDMOD

//--------------------------------------------
// ARDUINO Includes defines and initialysations
//--------------------------------------------
#include <Wire.h>
                    #ifdef LCDMOD
#include <LCD.h>
#include <LiquidCrystal_I2C.h>
                    #endif  LCDMOD
#include <RTClib.h>
#include <EEPROM.h>
#include "TimeLib.h"       
#include <WS2812.h>
                     #ifdef BLUETOOTHMOD
#include <SoftwareSerial.h>     // for Bluetooth communication
                     #endif BLUETOOTHMOD
                     #ifdef ROTARYMOD
#include <Encoder.h>
                     #endif ROTARYMOD
                     #ifdef DCFMOD
#include "DCF77.h"
                     #endif DCFMOD

#define NUM_LEDS    92            // How many leds in  strip?  
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
#define EDSOFT  Serial.print(F("Edsoft ")); 
/* 
#define  NUM_LEDS    113            // How many leds in  strip?
#define HET     Serial.print(F("Het "));    ColourLeds(0,3,MINColor);   
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
#define EDSOFT  Serial.print(F("Edsoft "));
*/

//--------------------------------------------
// PIN Assigments
//-------------------------------------------- 

// Digital hardware constants
enum DigitalPinAssignments {
 DCF_PIN      = 2,                // DCFPulse on interrupt  pin
 encoderPinA  = 3,                // right (labeled DT on decoder)on interrupt pin
 encoderPinB  = 4,                // left (labeled CLK on decoder)no interrupt pin  
 LED_PIN      = 5,                // Pin to control colour 2811/2812 leds
 BT_TX        = 6,                // Connects to Bluetooth RX
 BT_RX        = 7,                // Connects to Bluetooth TX
 clearButton  = 8,                // switch (labeled SW on decoder)
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


//--------------------------------------------
// LED
//--------------------------------------------

#define  BRIGHTNESS  200            // BRIGHTNESS 0 - 255
#define  UPDATES_PER_SECOND 20
WS2812   RGBstrip(NUM_LEDS);        // Initialyse RGB strip
byte     BrightnessCalcFromLDR = BRIGHTNESS;
int      Previous_LDR_read = 512;
int      ToggleEdsoft = 1;          // Flash led every hour
uint32_t MINColor      = 0XFFDD00;
uint32_t SECColor      = 0XFFDD00;
uint32_t LetterColor   = 0XFFDD00;
uint32_t DefaultColor  = 0XFFDD00;  // Yellow
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
Encoder myEnc(encoderPinA, encoderPinB);// Use digital pin  for encoder
                          #endif ROTARYMOD      
long     Looptime          = 0;
byte     RotaryPress       = 0;         // Keeps track displaychoice and how often the rotary is pressed.
uint32_t RotaryPressTimer  = 0;
byte     NoofRotaryPressed = 0;

//--------------------------------------------
// LDR PHOTOCELL
//--------------------------------------------
byte     LightReducer        = 80;         // Factor to dim ledintensity with. Between 0.1 and 1 in steps of 0.05
byte     LowerBrightness     = 10;         // Lower limit of Brightness ( 0 - 255)
int      OutPhotocell;                     // Stores reading of photocell;
int      MinPhotocell        = 999;        // Stores minimum reading of photocell;
int      MaxPhotocell        = 1;          // Stores maximum reading of photocell;

//--------------------------------------------
// CLOCK
//--------------------------------------------                                 
#define  MAXTEXT 80
static   unsigned long msTick;            // The number of millisecond ticks since we last incremented the second counter
int      count; 
int      Delaytime = 200;
byte     Isecond, Iminute, Ihour, Iday, Imonth, Iyear; 
byte     lastminute = 0, lasthour = 0, sayhour = 0;
byte     ChangeTime           = false;
byte     ChangeLightIntensity = false;
byte     SecPulse             = 0;          // give a pulse to the Isecond led
byte     Demo = false;
byte     Zelftest             = false;
String   SerialString;
byte     Is = true;                         // toggle of displaying Is or Was

const char menu[][MAXTEXT] PROGMEM =  {
 "Woordklok",
 "Voer in:",
 "Tijd uumm (1721) of uummss (172145)",
 "Dddmmjjjj (D15122017 voor 15 December 2017)",
 "Lnn     (L5) Min lichtintensiteit ( 1 - 255)",
 "Mnn     (M90)Max lichtintensiteit (1% - 250%)",
 "Pnnnnnn (P2345DD) voor eigen RGB-kleur (n=0-F)",
 "Qn(Q2) voor palettekeuze (Q0-Q4)",
 "        Q0 Geel, HET ISWAS verkleurt",
 "        Q1 Uurkleur, HET ISWAS verkleurt",
 "        Q2 Wit",
 "        Q3 Eigen kleur",
 "        Q4 Eigen kleur HET ISWAS verkleurt",
 "I       info",
 "X       Demo mode",
 "Z       Zelftest",
 "Ed Nieuwenhuys  Character_Clock_WS2812 Dec 2020" };

//--------------------------------------------
// DS3231 CLOCK MODULE
//--------------------------------------------
#define DS3231_I2C_ADDRESS          0x68
#define DS3231_TEMPERATURE_MSB      0x11
#define DS3231_TEMPERATURE_LSB      0x12
RTC_DS3231 RTCklok;    //RTC_DS1307 RTCklok;  
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

                    #ifdef DCFMOD           // DCF77 ------------------------------ 
#define DCF_INTERRUPT 0                     // DCF Interrupt number associated with DCF_PIN
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
#define DEFAULTCOLOUR 0
#define HOURLYCOLOUR  1           
#define WHITECOLOR    2
#define OWNCOLOUR     3
#define OWNHETISCLR   4
//byte DisplayChoice = DEFAULTCOLOUR;
//byte DisplayChoice = WHITECOLOR;
//byte DisplayChoice = OWNCOLOUR;
byte DisplayChoice   = HOURLYCOLOUR;
 // End Definitions  ---------------------------------------------------------


//--------------------------------------------
// ARDUINO Loop
//--------------------------------------------
void loop(void)
{
SerialCheck();
if(Demo)          Demomode();
else if(Zelftest) Selftest(); 
else
 { 
  EverySecondCheck();  
  EveryMinuteUpdate();
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
 Serial.begin(9600);                                              // Setup the serial port to 9600 baud  
 Serial.println("Serial 9600 baud enabled");   
                                  #ifdef BLUETOOTHMOD   
 Bluetooth.begin(9600);                                           // Set up the Bluetooth port to 9600 baud 
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
 RGBstrip.setOutput(LED_PIN);                                       // Set strip dat output to pin LED pin
 RGBstrip.sync();                                                   // Initialize all pixels to 'off'
 RGBstrip.setBrightness(BRIGHTNESS);  

                          #ifdef DCFMOD
 DCF.Start();                                                       // Start the DCF-module
 Serial.println("DCF enabled");
                          #endif DCFMOD
 Wire.begin();                                                      // Start the wire communication I2C
 RTCklok.begin();                                                       // Start the RTC-module
                          #ifdef LCDMOD
 lcd.begin (16,2); // for 16 x 2 LCD module                         // Activate LCD module
 lcd.setBacklightPin(3,POSITIVE);
 lcd.setBacklight(HIGH);
 Serial.println("LCD enabled");
                          #endif LCDMOD
 GetTijd(1);                                                        // Get the time and print it to serial 
 DateTime now = RTCklok.now();
 DateTime compiled = DateTime(__DATE__, __TIME__);
 if (now.unixtime() < compiled.unixtime()) 
  {
   Serial.println(F("RTC is older than compile time! Updating"));   // Following line sets the RTC to the date & time this sketch was compiled
   RTCklok.adjust(DateTime(F(__DATE__), F(__TIME__))); 
  }
 if (EEPROM.read(0) <3 || EEPROM.read(0) > 250)    
           EEPROM.write(0, LightReducer);                           // Default intensity for this clock
 if (EEPROM.read(1) <1 || EEPROM.read(1) > 100)   
           EEPROM.write(1, LowerBrightness);                        // Default Lower Brightness for this clock
 if (EEPROM.read(2) > OWNHETISCLR)                 
           EEPROM.write(2, DEFAULTCOLOUR);                          // Default Display setting
 LightReducer    = EEPROM.read(0);                                  // Store it is the work variable
 LowerBrightness = EEPROM.read(1);                                  // Store it is the work variable
 DisplayChoice   = EEPROM.read(2);                                  // Store it is the work variable
 OwnColour       = GetEEPROM_RGBW();                                // Get the stored RGBW value
 if (OwnColour == 0 ) OwnColour = 0X002345DD;                       // If memory is empty then store default value, blue
 Looptime = millis();                                               // Used in KY-040 rotary
 msTick = millis(); 
 SWversion();                                                       // Display the version number of the software
 GetTijd(0);                                                        // Get the time and print it to serial
 SetMinuteColour();                                                 // Set the colour per minute of 'HET'
                     #ifdef ROTARYMOD
 myEnc.write(0);                                                    // Clear Rotary encode buffer
                     #endif ROTARYMOD
// Selftest();                                                      // Play the selftest
} 
//--------------------------------------------
// CLOCK Version info
//--------------------------------------------
void SWversion(void) 
{ 
 PrintLine(52);
 for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)   {strcpy_P(sptext, menu[i]);                     Tekstprintln(sptext);  }
 PrintLine(52);
  sprintf(sptext,"Brightness Min: %3d bits  Max: %3d%%",LowerBrightness, LightReducer) ;
 Tekstprintln(sptext);
 sprintf(sptext,"  LDR read Min:%4d bits  Max: %3d bits",MinPhotocell, MaxPhotocell);
 Tekstprintln(sptext); 
 char HEXtxt[16];
 unsigned char bytes[4];
 bytes[0] = (OwnColour >> 24) & 0xFF;
 bytes[1] = (OwnColour >> 16) & 0xFF;
 bytes[2] = (OwnColour >> 8)  & 0xFF;
 bytes[3] =  OwnColour        & 0xFF;
 sprintf(HEXtxt,"0x%0.2X%0.2X%0.2X", bytes[1],bytes[2],bytes[3]);
 sprintf(sptext,"Display choice: %1d  Eigen kleur: %s",DisplayChoice,HEXtxt);
 Tekstprintln(sptext);
 PrintLine(52);
}
void PrintLine(byte Lengte)
{
 for (int n=0; n<Lengte; n++) {Serial.print(F("_"));} Serial.println(); 
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
   digitalWrite(secondsPin,HIGH);                                   // Turn ON the second on pin 13
   ++SecPulse;                                                      // For second routine in function DimLeds
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
  if ( millis() - msTick > 50)   digitalWrite(secondsPin,LOW);      // Turn OFF the second on pin 13
  if ( millis() - msTick >999)                                      // Flash the onboard Pin 13 Led so we know something is happening
   {    
    msTick = millis();                                              // second++; 
    digitalWrite(secondsPin,HIGH);                                  // Turn ON the second on pin 13
    ++SecPulse;                                                     // Second routine in function DimLeds
    GetTijd(0);                                                     // Synchronize time with RTC clock      
                    #ifdef LCDMOD
   Print_tijd_LCD();
                    #endif LCDMOD  
   if(Isecond % 30 == 0) DimLeds(true);                             // Led Intensity Control + seconds tick print every 30 seconds   
     else                DimLeds(false);
   SetSecondColour();                                               // Set the colour per second of 'IS' and 'WAS' 
   if(Iminute == 0 && Isecond <9)
    { 
     ToggleEdsoft = Isecond % 2;                                    // ToggleEdsoft bocomes 0 or 1 and turn on and off the first seconds at minute 0 the Edsoft light on pin 24
     Serial.println(ToggleEdsoft);
     Displaytime();                                                 // Dit moet iets laten knipperen
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
      ReworkInputString(BluetoothString);                          // Rework ReworkInputString();
 BluetoothString = "";
}
                           #endif BLUETOOTHMOD
                           
                           #ifdef DCFMOD
//--------------------------------------------
// CLOCK check for DCF input
//--------------------------------------------
void DCF77Check(void)
{
 time_t DCFtime = DCF.getTime();                                   // Check if new DCF77 time is available
 if (DCFtime!=0)
  {
   Tekstprint("DCF: Time is updated ----->  ");
   DCF_signal+=2;
   setTime(DCFtime); 
   RTCklok.adjust(DCFtime);
  }
 bool LHbit = digitalRead(DCF_PIN);
 digitalWrite(DCF_LED_Pin, 1 - LHbit );                            // write inverted DCF pulse to LED on board 
 if (SeeDCFsignalInDisplay == true)
  {
   Toggle_HetWasIs = LHbit;
   Display1 |= (Toggle_HetWasIs<<0);                               // Turn off the  HET IS WAS LEDs
   Displaytime();
  }
  DCF_signal = constrain(DCF_signal,0,99);                         // DCF_signal <100
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
   if (c>31 && c<128) SerialString += c;                            // Allow input from Space - Del
  }
 if (SerialString.length()>0)     ReworkInputString(SerialString);  // Rework ReworkInputString();
 SerialString = "";
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
    if ( encoderPos && ( (millis() - Looptime) > 200))              // If rotary turned avoid debounce within 0.20 sec
     {   
     Serial.print(F("----> Index:"));   Serial.println(encoderPos);
     if (encoderPos >0)                                             // Increase  
       {     
        if (ChangeLightIntensity)  { WriteLightReducer(5); }        // If time < 60 sec then adjust light intensity factor
        if (ChangeTime) 
          {
           if (NoofRotaryPressed == 1)                              // Change hours
              {
               Ihour++; 
               if( Ihour >23) { Ihour = 0; }
              }      
           if (NoofRotaryPressed == 2)                              // Change minutes
              { 
               Isecond = 0;
               Iminute++;
               if( Iminute >59) { Iminute = 0; Ihour++; if( Ihour >23) { Ihour = 0; } }   
              }
           } 
        }    
      if (encoderPos <0)                                            // Decrease
       {
       if (ChangeLightIntensity)   { WriteLightReducer(-5); }    // If time < 60 sec then adjust light intensity factor
       if (ChangeTime)     
          {
           if (NoofRotaryPressed == 1)                              // Change hours
            {
             Ihour--;
             if( Ihour <=0) { Ihour = 23; }
            }      
           if (NoofRotaryPressed == 2)                              // Change minutes
            { 
             Isecond = 0;
             Iminute--;
             if( Iminute == 0) { Iminute = 59; Ihour--; if( Ihour <= 0) { Ihour = 23; } }   
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
    if(++NoofRotaryPressed >8 ) NoofRotaryPressed = 1;
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
       default: NoofRotaryPressed = 0; 
                SeeDCFsignalInDisplay = ChangeTime = ChangeLightIntensity  = false;  
                Selftest();        
                break;                         
      } 
    Serial.print(F("NoofRotaryPressed: "));   Serial.println(NoofRotaryPressed); 
    if(DisplayChoice >2 && DisplayChoice <9 )
      {
        EEPROM.write(2, DisplayChoice);                                   // Store the display choice in EEPROM
        sprintf(sptext,"Display choice stored: Q%d",DisplayChoice);
        Tekstprintln(sptext);  
      }
    myEnc.write(0);
    Looptime = millis(); 
    SetSecondColour();                                                    // Set the colour per second of 'IS WAS'      
    SetMinuteColour();                                                    // Set the colour per minute of 'HET'
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
 for (int n=0 ; n<=NoofBlinks; n++) { LedsOff(); Laatzien(); delay(Delayms); ColourLeds(36, 59, 0XFF0000FF);  Laatzien(); delay(Delayms);} 
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
 case  1: EEN; break;
 case 14:
 case  2: TWEE; break;
 case 15:
 case  3: DRIE; break;
 case 16:
 case  4: VIER; break;
 case 17:
 case  5: VIJF; break;
 case 18:
 case  6: ZES; break;
 case 19:
 case  7: ZEVEN; break;
 case 20:
 case  8: ACHT; break;
 case 21:
 case  9: NEGEN; break;
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
 Inow =    RTCklok.now();
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
 RTCklok.adjust(DateTime(Inow.year(), Inow.month(), Inow.day(), Ihour, Iminute, Isecond));
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
//  LED function to make RGBW color
//-------------------------------------------- 
uint32_t FuncCRGBW( uint32_t Red, uint32_t Green, uint32_t Blue, uint32_t White)
{ 
 return ( (White<<24) + (Red << 16) + (Green << 8) + Blue );
}
//--------------------------------------------
//  LED functions to extract RGBW colors
//-------------------------------------------- 
 uint8_t Cwhite(uint32_t c) { return (c >> 24);}
 uint8_t Cred(  uint32_t c) { return (c >> 16);}
 uint8_t Cgreen(uint32_t c) { return (c >> 8); }
 uint8_t Cblue( uint32_t c) { return (c);      }

//--------------------------------------------
//  LED Store RGBW colors in EEPROM 
//-------------------------------------------- 
void StoreEEPROM_RGBW(uint32_t RGBWvalue)
{
 EEPROM.write(4, Cred(  RGBWvalue));
 EEPROM.write(5, Cgreen(RGBWvalue));
 EEPROM.write(6, Cblue( RGBWvalue));
 EEPROM.write(7, Cwhite(RGBWvalue));
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
 if (DisplayChoice == DEFAULTCOLOUR) {LetterColor = DefaultColor;    }   
 if (DisplayChoice == WHITECOLOR )   {LetterColor = WhiteColour; MINColor = WhiteColour;  SECColor = WhiteColour;    } // all white
 if (DisplayChoice == HOURLYCOLOUR ) {LetterColor = HourColor[Ihour];  }                                               // A colour every hour
 if (DisplayChoice == OWNCOLOUR )    {LetterColor = OwnColour;   MINColor = OwnColour;   SECColor = OwnColour;       } // all white 
 if (DisplayChoice == OWNHETISCLR )  {LetterColor = OwnColour;      }                                                  // own colour except HET IS WAS  
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
// byte Bluecolor = 0 ;
 SECColor = (uint32_t) FuncCRGB( (uint32_t)(15 + ((uint32_t) Isecond * 4)),   (uint32_t)(255 - (uint32_t)(Isecond * 4)), 0); //(uint32_t)Bluecolor);
 CheckColourStatus();
 if(NUM_LEDS == 92)
 {
  if (Is) { ColourLeds(3,4,SECColor); ColourLeds(5,7,0);  }
   else   { ColourLeds(3,4,0);        ColourLeds(5,7,SECColor);  }  
 }
 else
 { 
 if (Is) { ColourLeds(4,5,SECColor);  ColourLeds(6,9,0);  }
   else  { ColourLeds(4,5,0);         ColourLeds(6,9,SECColor);  }  
 }
 
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
  int LDR_read = (4 * Previous_LDR_read + analogRead(PhotoCellPin)) / 5;               // Read lightsensor 
  int BrCalc, Temp;
  Previous_LDR_read = LDR_read;
  OutPhotocell = (int)((LightReducer * sqrt(63.5*LDR_read))/100);   
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
  SetBrightnessLeds(BrightnessCalcFromLDR);  //LEDS.setBrightness(BRIGHTNESS);    *** naar functie SetBrightnessLeds                               
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

//--------------------------------------------
//  LED In- or decrease light intensity value
//--------------------------------------------
void WriteLightReducer(int amount)
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
 EEPROM.write(0, LightReducer);                // Store the value (0-250) in permanent EEPROM memory at address 0
 sprintf(sptext,"Max brightness: %3d%%",LightReducer);
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
 float  ff;
 int    Jaar;
 InputString.toCharArray(sptext, MAXTEXT-1);

 if ( InputString[0] > 64 )                                           // Does the string start with a letter?
  {
  int val = InputString[0];
  int FMfreq;
  
  Tekstprintln(sptext);
  switch (val)
   {
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
              RTCklok.adjust(DateTime(Jaar, Imonth, Iday, Inow.hour(), Inow.minute(), Inow.second()));
              sprintf(sptext,"%0.2d:%0.2d:%0.2d %0.2d-%0.2d-%0.4d",Inow.hour(),Inow.minute(),Inow.second(),Iday,Imonth,Jaar);
              Tekstprintln(sptext);
             }
             else Tekstprintln("**** Length fault. Enter ddmmyyyy ****");

            break; 
                     #ifdef DCFMOD   
    case 'G':                                                         // Toggle DCF Signal on Display
    case 'g':
             SeeDCFsignalInDisplay = 1 - SeeDCFsignalInDisplay;
             sprintf(sptext,"SeeDCFsignal: %d",SeeDCFsignalInDisplay);
             Tekstprintln(sptext);
             break;
                     #endif DCFMOD
    case 'L':                                                         // Lowest value for Brightness
    case 'l':    
             if (InputString.length() < 5)
               {      
                temp = InputString.substring(1);
                LowerBrightness = temp.toInt();
                LowerBrightness   = constrain(LowerBrightness,1,255);
                WriteLowerBrightness(LowerBrightness);
                sprintf(sptext,"Lower brightness changed to: %d bits",LowerBrightness);
                Tekstprintln(sptext);
               }
             else Tekstprintln("**** Length fault. Enter Lnnn ****");
             break;
    case 'M':                                                         // factor ( 0 - 1) to multiply brighness (0 - 255) with 
    case 'm': 
             if (InputString.length() < 5)
               {    
                temp = InputString.substring(1);
                byte Intensityfactor = temp.toInt();
                Intensityfactor = constrain(Intensityfactor,1,255);
                WriteLightReducerEeprom(Intensityfactor);
                sprintf(sptext,"Max brightness changed to: %d%%",Intensityfactor);
                Tekstprintln(sptext);
               }
             else Tekstprintln("**** Length fault. Enter Mnnn ****");
              break;     
    case 'P':
    case 'p':  
            if (InputString.length() == 7 )
              {
               temp = InputString.substring(1,7);
               OwnColour = hexToDec(temp);                            // Display letter color
               LetterColor = OwnColour;                               // Display letter color 
               StoreEEPROM_RGBW(OwnColour);                           // Store the RGBW value in EEPROM
               DisplayChoice = OWNCOLOUR;
               Tekstprintln("**** Palette changed ****");
               EEPROM.write(2, DisplayChoice);                        // Store the display choice in EEPROM
               sprintf(sptext,"Display choice stored: Q%d",DisplayChoice);
               Tekstprintln(sptext); 
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
                EEPROM.write(2, DisplayChoice);                        // Store the display choice in EEPROM
                sprintf(sptext,"Display choice: Q%d",DisplayChoice);
                Tekstprintln(sptext);
                lastminute = 99;                                      // Force a minute update
               }
             else Tekstprintln("**** Display choice length fault. Enter Q0 - Q6"); 
             Displaytime();                                           // Turn on the LEDs with proper time             
            break;    
    case 'I':
    case 'i':   
            SWversion();
            break;

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
