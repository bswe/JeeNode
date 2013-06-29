// TM2025_Logger (R 1.0):  This program logs the data from a 
// TriMetric TM-2025 battery monitor (Bogart Engineering), and allows 
// access to the logged data via a wireless interface.
//
// 5/14/2011 (copyright Bollenbacher Software Engineering)

#include <Ports.h>
#include <RF12.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define LED_P1D_PIN     4    // activity LED on port 1 pin DIO
#define LED_P1A_PIN     14   // activity LED on port 1 pin AIO
#define LED_P2D_PIN     5    // activity LED on port 2 pin DIO
#define LED_P2A_PIN     15   // activity LED on port 2 pin AIO
#define LED_P3D_PIN     6    // activity LED on port 3 pin DIO
#define LED_P3A_PIN     16   // activity LED on port 3 pin AIO
#define LED_P4D_PIN     7    // activity LED on port 4 pin DIO
#define LED_P4A_PIN     17   // activity LED on port 4 pin AIO
#define ON              1
#define OFF             0

#define INPUT_BFR_SIZE 100
#define LOG_BFR_SIZE   100

PortI2C EepromPort (1);                  // using port 1 on the JeeNode
DeviceI2C LowBank (EepromPort, 0x50);
DeviceI2C HighBank (EepromPort, 0x51);

 char HELP_TEXT[] PROGMEM = 
    "TM2025_Logger (R 1.0)\n"
    "Available commands:" "\n"
    "  d     - display logged data" "\n"
    "  l     - log data" "\n"
    "  s     - stop logging data" "\n"
    "  m     - display data from monitor" "\n"
;

static char SerialInputBfr[INPUT_BFR_SIZE];
static byte SerialBfrIndex = 0;
static char LogBfr[LOG_BFR_SIZE];
static byte LogBfrIndex = 0;


static void SetLed (byte Port, byte State) {
   digitalWrite (Port, !State);
   }


static void BlinkLed (byte Port, int Duration) {
   SetLed (Port, ON);
   delay (Duration);
   SetLed (Port, OFF);
   }


static void ShowHelpString (PGM_P s) {
   for (;;) {
      char c = pgm_read_byte (s++);
      if (0 == c)
         break;
      if ('\n' == c)
         Serial.print ('\r');
      Serial.print (c);
      }
   }


static void ShowHelp () {
   ShowHelpString (HELP_TEXT);
   }


static void ProcessSerialInput (char c) {
   char Digits[10];
   char Str[20];
   byte i, j, k;
   
   if (',' == c) {
      SerialInputBfr[SerialBfrIndex] = 0;
      Serial.println (SerialInputBfr);
      SerialBfrIndex = 0;
      } 
   else {
      SerialInputBfr[SerialBfrIndex++] = c;
      /*
      i = 9;
      while (c > 0) {
         Digits[i--] = c % 10;
         c = c/10;
         }
      Str[0] = '0';
      for (j=i+1, k=1; j <= 9; j++,k++)
         Str[k] = Digits[j] + '0';
      Str[k] = 0;
      /*
      Serial.print ('\'');
      Serial.print (Str);
      Serial.println ('\'');
      */
      //BlinkLed (LED_P1D_PIN, ON, 10);
      }
   }


void setup () {
   //Serial.begin(2400);
   //ShowHelp();
   pinMode(LED_P1D_PIN, OUTPUT);     
   SetLed (LED_P1D_PIN, OFF);
   pinMode(LED_P1A_PIN, OUTPUT);     
   SetLed (LED_P1A_PIN, OFF);
   pinMode(LED_P2D_PIN, OUTPUT);     
   SetLed (LED_P2D_PIN, OFF);
   pinMode(LED_P2A_PIN, OUTPUT);     
   SetLed (LED_P2A_PIN, OFF);
   pinMode(LED_P3D_PIN, OUTPUT);     
   SetLed (LED_P3D_PIN, OFF);
   pinMode(LED_P3A_PIN, OUTPUT);     
   SetLed (LED_P3A_PIN, OFF);
   pinMode(LED_P4D_PIN, OUTPUT);     
   SetLed (LED_P4D_PIN, OFF);
   pinMode(LED_P4A_PIN, OUTPUT);     
   SetLed (LED_P4A_PIN, OFF);
   }


void loop () {
   /*
   if (Serial.available())
      ProcessSerialInput (Serial.read());
   */
   BlinkLed (LED_P1D_PIN, 500);
   BlinkLed (LED_P1A_PIN, 500);
   BlinkLed (LED_P2D_PIN, 500);
   BlinkLed (LED_P2A_PIN, 500);
   BlinkLed (LED_P3D_PIN, 500);
   BlinkLed (LED_P3A_PIN, 500);
   BlinkLed (LED_P4D_PIN, 500);
   BlinkLed (LED_P4A_PIN, 500);
   }
