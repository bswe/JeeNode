/// @dir ledNode
/// Programmable color ramps for the LED drivers on the LED Node.
// 2011-10-26 <jc@wippler.nl> http://opensource.org/licenses/mit-license.php

#include <JeeLib.h>
#include <EEPROM.h>
#include <avr/sleep.h>

#define EEPROM_BASE 0x100 // store ramps starting at this offset
#define RAMP_LIMIT 100    // room for ramps 0..99, stored in EEPROM

#define LED_R 6   // the PWM pin which drives the red LED
#define LED_G 9   // the PWM pin which drives the green LED
#define LED_B 5   // the PWM pin which drives the blue LED
#define Sprint Serial.print
#define Sprintln Serial.println

const char HELP_TEXT[] PROGMEM = 
   "\nLED tester 1.0\n"
   "<nn> d - display LED program nn\n"
   "    0 = instant off\n"
   "    1 = instant warm white\n"
   "    2 = instant cool white\n"
   "    3 = 5s off\n"
   "    4 = 5s warm white\n"
   "    5 = 5s cool white\n"
   "    6 = 5s red -> green -> blue\n"
   "    9 = instant faint red'ish yellow\n"
   "   10 = 5s red -> blue -> green\n"
   "   13 = 1s red -> green -> blue\n"
   "   16 = .5s instant red -> blue -> green\n"
   "   19 = .3s instant red -> blue -> green\n"
   "   22 = 1s red -> blue -> green\n"
   "   25 = .5s red -> blue -> green\n";

/// @struct Ramp
/// A "Ramp" is a target RGB color and the time in seconds to reach that color.
/// Ramps can be chained together with a non-zero ramp index in the chain field.
typedef struct {
  byte colors[3]; // red, green, blue, 0..255
  int steps;     // number of seconds used to reach given RGB colors
  byte chain;     // next ramp to use when done, or 0 to stay as is
} Ramp;

long now[3];      // current PWM values, as 9+23 bit fractional int
long delta[3];    // current PWM deltas, as 9+23 bit fractional int
int duration;    // number of 0.01s steps remaining in this ramp
byte nextRamp;    // which saved ramp to use next (or none if zero)
MilliTimer timer; // used as timer for the 0.01s steps
static unsigned long InputValue;

static Ramp stdRamps[] = {
  {   0,   0,   0,   0,  0 }, // 0:  instant off
  { 255,  85,  30,   0,  0 }, // 1:  instant warm white
  { 255, 150,  75,   0,  0 }, // 2:  instant cold white
  {   0,   0,   0, 500,  0 }, // 3:  5s off
  { 255,  85,  30, 500,  0 }, // 4:  5s warm white
  { 255, 150,  75, 500,  0 }, // 5:  5s cold white
  { 255,   0,   0, 500,  7 }, // 6:  5s red -> green -> blue
  {   0, 255,   0, 500,  8 }, // 7:  5s green -> blue -> red
  {   0,   0, 255, 500,  6 }, // 8:  5s blue -> red -> green
  {   7,   1,   0,   0,  0 }, // 9:  instant faint red'ish yellow
  { 255,   0,   0, 500, 12 }, // 10: 5s red -> blue -> green
  {   0, 255,   0, 500, 10 }, // 11: 5s green -> red -> blue
  {   0,   0, 255, 500, 11 }, // 12: 5s blue -> green -> red
  { 255,   0,   0, 100, 14 }, // 13: 1s red -> green -> blue
  {   0, 255,   0, 100, 15 }, // 14: 1s green -> blue -> red
  {   0,   0, 255, 100, 13 }, // 15: 1s blue -> red -> green
  { 255,   0,   0, -50, 18 }, // 16: .5s instant red -> blue -> green
  {   0, 255,   0, -50, 16 }, // 17: .5s instant green -> red -> blue
  {   0,   0, 255, -50, 17 }, // 18: .5s instant blue -> green -> red
  { 255,   0,   0, -30, 21 }, // 19: .3s instant red -> blue -> green
  {   0, 255,   0, -30, 19 }, // 20: .3s instant green -> red -> blue
  {   0,   0, 255, -30, 20 }, // 21: .3s instant blue -> green -> red
  { 255,   0,   0, 100, 24 }, // 22: 1s red -> blue -> green
  {   0, 255,   0, 100, 22 }, // 23: 1s green -> red -> blue
  {   0,   0, 255, 100, 23 }, // 24: 1s blue -> green -> red
  { 255,   0,   0,  50, 27 }, // 25: .5s red -> blue -> green
  {   0, 255,   0,  50, 25 }, // 26: .5s green -> red -> blue
  {   0,   0, 255,  50, 26 }, // 27: .5s blue -> green -> red
 };


static void setLeds () {
  // set to bits 30..23, but rounded by one extra bit (i.e. bit 22)
  analogWrite(LED_R, (byte) (((word) (now[0] >> 22) + 1) >> 1));
  analogWrite(LED_G, (byte) (((word) (now[1] >> 22) + 1) >> 1));
  analogWrite(LED_B, (byte) (((word) (now[2] >> 22) + 1) >> 1));
}

static void useRamp (const void* ptr) {
  const Ramp* ramp = (const Ramp*) ptr;
  nextRamp = ramp->chain;
  //duration = ramp->steps * 100;
  duration = ramp->steps;
  for (byte i = 0; i < 3; ++i) {
    long target = (long) ramp->colors[i] << 23;
    if (duration > 0)
      delta[i] = (target - now[i]) / duration;
    else
      now[i] = target;
  }
  setLeds();
}

static void loadRamp (byte pos) {
  if (pos < RAMP_LIMIT) {
    word addr = EEPROM_BASE + pos * sizeof (Ramp);
    Ramp ramp;
    for (byte i = 0; i < sizeof (Ramp); ++i)
      ((byte*) &ramp)[i] = EEPROM.read(addr+i);
    useRamp(&ramp);
  }
}

static void saveRamp (byte pos, const void* data) {
  if (pos < RAMP_LIMIT) {
    word addr = EEPROM_BASE + pos * sizeof (Ramp);
    for (byte i = 0; i < sizeof (Ramp); ++i)
      EEPROM.write(addr+i, ((const byte*) data)[i]);
  }
}

static void ShowHelpString (PGM_P s) {
   for (;;) {
      char c = pgm_read_byte (s++);
      if (0 == c)
         break;
      if ('\n' == c)
         Sprint ('\r');
      Sprint (c);
      }
   }


static void ShowHelp () {
   ShowHelpString (HELP_TEXT);
   }

static void ProcessSerialInput (char c) {
    if ('0' <= c && '9' >= c)
        InputValue = 10 * InputValue + c - '0';
    else if ('a' <= c && c <='z') {
        switch (c) {
			case 'd': // turn activity LED on or off
			   loadRamp(InputValue);
               break;
			default:
				ShowHelp();
        }
        InputValue = 0;
	}
}

void setup () {
  // fix timer 1 so it also runs in fast PWM mode, to match timer 0
  bitSet(TCCR1B, WGM12);
  // set up the default ramps
  for (byte i = 0; i < sizeof stdRamps / sizeof *stdRamps; ++i)
    saveRamp(i, stdRamps + i);
  // intialize wireless
  rf12_initialize(1, RF12_868MHZ, 19);
  // test code: start up with ramp #1
  loadRamp(2);
  Serial.begin (57600);
  ShowHelp ();
}

void loop () {
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_mode();

  if  (Serial.available ()) {
    ProcessSerialInput (Serial.read());
  }
  if (timer.poll(10)) {
    if (duration > 0) {
      --duration;
      for (byte i = 0; i < 3; ++i)
        now[i] += delta[i];
      setLeds();
	} else if (duration < 0) {
	  ++duration;
      setLeds();
    } else if (nextRamp != 0)
      loadRamp(nextRamp);
  }
  
  if (rf12_recvDone() && rf12_crc == 0) {
    const byte* p = (const byte*) rf12_data;
    if (rf12_len == 1) {
      // a single byte loads the ramp from EEPROM
      loadRamp(rf12_data[0]);
    } else if (rf12_len == 1 + sizeof (Ramp)) {
      // 6 bytes, either a save or an immediate command
      // make sure that slot zero, i.e. "all-off", is never overwritten
      if (rf12_data[0] > 0) // save the date to EEPROM, if slot is not zero
        saveRamp(rf12_data[0], (const Ramp*) (rf12_data + 1));
      else // use the ramp as is without saving, if slot is zero
        useRamp((const Ramp*) (rf12_data + 1));
    }
  }
}
