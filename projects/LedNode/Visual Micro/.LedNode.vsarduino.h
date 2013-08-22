//Board = Arduino Uno
#define __AVR_ATmega328P__
#define ARDUINO 105
#define F_CPU 16000000L
#define __AVR__
extern "C" void __cxa_pure_virtual() {;}

static void setLeds ();
static void useRamp (const void* ptr);
static void loadRamp (byte pos);
static void saveRamp (byte pos, const void* data);
static void ShowHelpString (PGM_P s);
static void ShowHelp ();
static void ProcessSerialInput (char c);
//
//

#include "c:\Program Files\Arduino\hardware\arduino\variants\standard\pins_arduino.h" 
#include "c:\Program Files\Arduino\hardware\arduino\cores\arduino\arduino.h"
#include "C:\Work\BSE\Atmel\JeeNode\projects\LedNode\LedNode.ino"
