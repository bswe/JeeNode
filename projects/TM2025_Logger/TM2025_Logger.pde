// TM2025_Logger (R 1.0):  This program logs the data from a 
// TriMetric TM-2025 battery monitor (Bogart Engineering), and allows 
// access to the logged data via a wireless interface.  Serial port
// runs at 2400 buad.
//
// 11/12/2011 (copyright Bollenbacher Software Engineering)

#include <Ports.h>
#include <RF12.h>
#include <util/crc16.h>
#include <util/parity.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <SD.h>

#define SW_VERSION "2.0.1"

#define HEARTBEAT_PERIOD      2000                           // 2 second
#define ACK_TIMEOUT           100                            // .1 second
#define DISCONNECTED_FROM_MONITOR_TIMEOUT_IN_HEARTBEATS 2    // 4 seconds
#define WRITE_LOG_INFO_TIMEOUT_IN_HEARTBEATS 150             // 5 minutes

#define SERIAL_INPUT_BFR_SIZE 20
#define MAX_PACKET_PAYLOAD_SIZE 62

// 125 kbyte EEPROM constants
#define LOG_BFR_SIZE          128000   // EEPROM size in bytes
#define EEPROM_PORT           1        // using port 1 on the JeeNode for EEPROM
#define LOW_BANK_ADDR         0x50     // address for accessing low 64k of M24M01
#define HIGH_BANK_ADDR        0x51     // address for accessing high 64k of M24M01
#define LOG_BFR_START         16       // leave 1 byte for RunMode,
                                       // 1 byte for wrap flag,
                                       // 4 bytes for TimeStamp,
                                       // 4 bytes for WrapTimeDelta,
                                       // 2 bytes for NumberOfLogEntries,
                                       // and 4 bytes for storing LogBfrEnd
#define WRITE_DELAY           5        // 5 ms delay for m24m01 to complete write
#define NUMBER_TO_AVERAGE     2        // number of values to average for log value
#define MAX_LOG_ENTRY_SIZE    3        // maximum number of bytes in a log entry

#define LED_P2D_PIN     5    // activity LED on port 2 pin DIO
#define GREEN_LED       LED_P2D_PIN
#define LED_P2A_PIN     15   // activity LED on port 2 pin AIO
#define RED_LED         LED_P2A_PIN
#define ON              1
#define OFF             0
#define LED_ON_TIME     100  // number of milli seconds that heartbeat led is on per blink
#define LED_OFF_TIME    300  // number of milli seconds that heartbeat led is off per blink
#define NONE            -1   // value to indicate that no heartbeat led state change is scheduled

#define WIRELESS_LINK_ID 1
#define UNKNOWN_CMD      "*U"

#define Sprint Serial.print
#define Sprintln Serial.println

PortI2C EepromPort (EEPROM_PORT);                  
DeviceI2C LowBank (EepromPort, 0x50);
DeviceI2C HighBank (EepromPort, 0x51);

char HELP_TEXT[] PROGMEM = 
    "\nTM2025_Logger (R "
    SW_VERSION
    ")\n"
    "Available commands:" "\n"
    "  <n> a    - turn activity LED on PB1 on or off" "\n"
    "  <nn> i   - set node ID (standard node ids are 1..26)" "\n"
    "  <n> b    - set MHz band (4 = 433, 8 = 868, 9 = 915)" "\n"
    "  <nnn> g  - set network group (0 = any)" "\n"
    "  c        - clear log" "\n"
    "  d        - display logged data" "\n"
    "  f        - fill the log buffer" "\n"
    "  l        - Start logging data w/o displaying it" "\n"
    "  m        - Start logging data & display it" "\n"
    "  <n> q    - set quiet mode (1 = don't report bad packets)" "\n"
    "  !        - Stop logging" "\n"
    "  ?        - Dump program info" "\n"

;

// Run mode enum 
typedef enum {NotLogging, LoggingWithMonitor, LoggingWithoutMonitor, UploadingData} RunModes;
static RunModes RunMode = NotLogging;

typedef enum {LookingForAmps, CapturingAmps, LookingForVolts, CapturingVolts} CaptureStates;
static CaptureStates CaptureState;
static unsigned char CharToFind;

static char SerialInputBfr[SERIAL_INPUT_BFR_SIZE];
static unsigned long SerialBfrOverflowCount;
static unsigned char SerialBfrIndex;
static unsigned long LogBfrEnd, InputValue;
static unsigned long TimeStamp = 0, WrapTimeDelta = 0, TimeSyncTime = 0;
static unsigned int NumberOfLogEntries;
static unsigned char BufferHasWrapped;
static unsigned int AmpValues[NUMBER_TO_AVERAGE];
static unsigned char VoltValues[NUMBER_TO_AVERAGE], LastVoltValue = 100;
static char NextValue;
static unsigned long LastTime, ReceiveTime, TxTime = 0;
static unsigned int DisconnectedFromMonitorTimeout;
static unsigned int WriteLogInfoTimeout;
static unsigned char AckRequested, SerialEnabled = 0;

typedef struct {
    unsigned char LedState;
    unsigned int When;
    } NextLedStateType;

static NextLedStateType NextLedState;
static unsigned char NumberOfHeartbeatBlinks;

BlinkPlug blinkPlug (2);

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// RF12 configuration setup code

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

typedef struct {
    byte nodeId;
    byte group;
    char msg[RF12_EEPROM_SIZE-4];
    word crc;
} RF12Config_type;

static RF12Config_type RF12Config;

static char cmd;
static byte stack[RF12_MAXDATA], top, sendLen, dest, quiet = 1;
static byte testbuf[RF12_MAXDATA];

static void addCh (char* msg, char c) {
    byte n = strlen(msg);
    msg[n] = c;
}

static void addInt (char* msg, word v) {
    if (v >= 10)
        addInt(msg, v / 10);
    addCh(msg, '0' + v % 10);
}

static void saveConfig () {
   byte id;

   // set up a nice config string to be shown on startup
   memset(RF12Config.msg, 0, sizeof RF12Config.msg);
    
   id = RF12Config.nodeId & 0x1F;
   strcat(RF12Config.msg, "ID=");
   addCh(RF12Config.msg, '@' + id);
   strcat(RF12Config.msg, "(");
   addInt(RF12Config.msg, id);
   strcat(RF12Config.msg, "),");
   if (RF12Config.nodeId & COLLECT)
      addCh(RF12Config.msg, '*');
    
   strcat(RF12Config.msg, " G");
   addInt(RF12Config.msg, RF12Config.group);
    
   strcat(RF12Config.msg, ", @");
   static word bands[4] = { 315, 433, 868, 915 };
   word band = RF12Config.nodeId >> 6;
   addInt(RF12Config.msg, bands[band]);
   strcat(RF12Config.msg, "MHz");
    
   RF12Config.crc = ~0;
   for (byte i = 0; i < sizeof RF12Config - 2; ++i)
      RF12Config.crc = _crc16_update(RF12Config.crc, ((byte*) &RF12Config)[i]);

   // save to EEPROM
   for (byte i = 0; i < sizeof RF12Config; ++i) {
      byte b = ((byte*) &RF12Config)[i];
      eeprom_write_byte(RF12_EEPROM_ADDR + i, b);
      }
    
   byte Response = rf12_config(0);
   if (!Response) {
      Sprint ("config save failed: rf12_config() returned ");
      Sprintln (Response, DEC);
      }
   }


static void SetLed (byte Port, byte On) {
   pinMode (Port, OUTPUT);
   digitalWrite (Port, !On);
   }


static void DumpProgramInfoToWireless () {
   char Offset = 1;
   char Bfr [sizeof(long)*2+1];
   unsigned long DeltaTime;

   /*
   ltoa (LogBfrEnd, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   itoa (BufferHasWrapped, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   itoa (SerialBfrIndex, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   ltoa (SerialBfrOverflowCount, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   */
   itoa (NumberOfLogEntries, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);

   testbuf[Offset++] = ':';
   ltoa (TimeStamp, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);

   testbuf[Offset++] = ':';
   ltoa (WrapTimeDelta, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);

   testbuf[Offset++] = ':';
   DeltaTime = millis();
   DeltaTime = DeltaTime - TimeSyncTime;
   ltoa (DeltaTime, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   /*
   testbuf[Offset++] = ':';
   itoa (CaptureState, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   itoa (quiet, Bfr, 16);
   for (byte i=0; i < strlen (Bfr); i++)
      testbuf[i+Offset] = Bfr[i];
   Offset += strlen (Bfr);
   testbuf[Offset++] = ':';
   testbuf[Offset++] = CharToFind;
   */
   //testbuf[Offset++] = ':';
   //rf12_config();
   testbuf[Offset] = 0;
   //Sprintln ((char*) testbuf);
   }


static void DumpProgramInfo () {
   Sprint ("\nLogBfrEnd = ");
   Sprintln (LogBfrEnd, DEC);
   Sprint ("BufferHasWrapped = ");
   Sprintln (BufferHasWrapped, DEC);
   Sprint ("SerialBfrIndex = ");
   Sprintln (SerialBfrIndex, DEC);
   Sprint ("SerialBfrOverflowCount = ");
   Sprintln (SerialBfrOverflowCount, DEC);
   Sprint ("NumberOfLogEntries = ");
   Sprintln (NumberOfLogEntries, DEC);
   Sprint ("CaptureState = ");
   Sprintln (CaptureState, DEC);
   Sprint ("CharToFind = ");
   Sprintln (CharToFind);
   Sprint ("Quiet = ");
   Sprintln (quiet, DEC);
   Sprint ("Current radio configuration: ");
   rf12_config();
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


unsigned char ReadLowBank (unsigned long Address) {
   unsigned char Value;

	LowBank.send();
	LowBank.write(Address >> 8);
	LowBank.write(Address & 0xFF);	
	LowBank.receive();
	Value = LowBank.read(1);
	LowBank.stop();
   return Value;
   }
unsigned char ReadHighBank (unsigned long Address) {
   unsigned char Value;



	HighBank.send();
	HighBank.write(Address >> 8);
	HighBank.write(Address & 0xFF);	
	HighBank.receive();
	Value = HighBank.read(1);
	HighBank.stop();
   return Value;
   }


void WriteLowBank (unsigned long Address, unsigned char Value) {
	LowBank.send();
	LowBank.write(Address >> 8);
	LowBank.write(Address & 0xFF);	
	LowBank.write(Value);
	LowBank.stop();
   }


void WriteHighBank (unsigned long Address, unsigned char Value) {
	HighBank.send();
	HighBank.write(Address >> 8);
	HighBank.write(Address & 0xFF);	
	HighBank.write(Value);
	HighBank.stop();
   }


unsigned char ReadEeprom (unsigned long Address) {
   if (Address >= 0x10000)
      return (ReadHighBank (Address-0x10000));
   else
      return (ReadLowBank (Address));
   }


void WriteEeprom (unsigned long Address, unsigned char Value) {
   if (Address >= 0x10000)
      WriteHighBank (Address-0x10000, Value);
   else
      WriteLowBank (Address, Value);
	delay (WRITE_DELAY);
   }


static void ClearLog () {
   int i;

   NumberOfLogEntries = 0;
   BufferHasWrapped = 0;
   LogBfrEnd = LOG_BFR_START-1;
   for (i=1; LOG_BFR_START-1 > i; i++) 
      WriteEeprom (i, 0);
   WriteEeprom (LOG_BFR_START-1, LogBfrEnd);
   LastVoltValue = 100;
   Sprintln ("Cleared log");
   }


static void ClearSerialBfr () {
   SerialBfrOverflowCount = 0;
   SerialBfrIndex = 0;
   CaptureState = LookingForAmps;
   CharToFind = ',';
   InputValue = 0;
   NextValue = 0;
   LastVoltValue = 100;
   }


static char* ConvertLogDataToStr (int Amps, int Volts) {
   static char StrBfr[SERIAL_INPUT_BFR_SIZE];
   char Index = 0;
   char Digit;

   // format the amps
   StrBfr[Index++] = 'A';
   StrBfr[Index++] = ' ';
   Amps -= 1500;
   if (Amps < 0) {
      StrBfr[Index++] = '-';
      Amps *= -1;
      }
   if (1000 <= Amps) {
      StrBfr[Index++] = '1';
      Amps -= 1000;
      }
   Digit = Amps / 100;
   StrBfr[Index++] = Digit + '0';
   StrBfr[Index++] = '.';
   Amps %= 100;
   Digit = Amps / 10;
   StrBfr[Index++] = Digit + '0';
   Amps %= 10;
   if (0 < Amps)
      StrBfr[Index++] = Amps + '0';

   // format the volts
   StrBfr[Index++] = ' ';
   StrBfr[Index++] = ' ';
   StrBfr[Index++] = ' ';
   StrBfr[Index++] = 'V';
   StrBfr[Index++] = ' ';
   StrBfr[Index++] = '1';
   Digit = Volts / 10;
   StrBfr[Index++] = Digit + '0';
   Volts %= (Digit * 10);
   StrBfr[Index++] = '.';
   StrBfr[Index++] = Volts + '0';

   // terminate string
   StrBfr[Index++] = 0;

   return StrBfr;
   }


static void DumpLog (long StartAtAddress) {
   unsigned char BytePosition = 0;
   long Start, Stop, LogEntryIndex;
   char LookingForFirstLogEntry = 1;
   unsigned char c;
   char AllDone = 0;
   int Amps, Volts;
   char* LogStr;

   if (0 == NumberOfLogEntries) {
      Sprintln ("Log is empty");
      return;
      }
   if (0 != StartAtAddress)
      Start = StartAtAddress;     // use starting address passed in from caller
   else if (BufferHasWrapped) {   
      Start = LogBfrEnd + 1;      // if bfr has wrapped find start of buffer from the end pointer
      if (LOG_BFR_SIZE == Start)
         Start = LOG_BFR_START;
      }
   else 
      Start = LOG_BFR_START;      // else if bfr hasn't wrapped, use the beginning of the bfr
   Stop = LogBfrEnd;
   if (Start == Stop)
      return;
   LastVoltValue = 100;
   while (1) {
      c = ReadEeprom (Start);
      Sprint ("LogBfr[");
      Sprint (Start, DEC);
      Sprint ("] = ");
      Sprint ((unsigned char) c, DEC);
      Sprint (" (");
      Sprint ((unsigned char) c, HEX);
      Sprintln ("h)");
      if ((c & 1) == 1) {
         // found the first byte of a log entry
         if (LookingForFirstLogEntry) {
            LookingForFirstLogEntry = 0;
            LogEntryIndex = Start;
            }
         else {   // found next log entry while procesing a log entry, so log entry is complete
            if (BytePosition == 2) {
               if (100 == LastVoltValue) {
                  Sprint ("No required voltage value at ");
                  Sprintln (LogEntryIndex ,DEC);
                  }
               else
                  Volts = LastVoltValue;
               }
            else
               LastVoltValue = Volts;
            LogStr = ConvertLogDataToStr (Amps, Volts);
            Sprint ("LogEntry at ");
            Sprint (LogEntryIndex ,DEC);
            Sprint (" is  ");
            Sprintln (LogStr);
            BytePosition = 0;
            LogEntryIndex = Start;
            }
         }
      if (!LookingForFirstLogEntry) {
         if (MAX_LOG_ENTRY_SIZE < BytePosition) {
            Sprintln ("ERROR: log entry longer than 3 bytes");
            LookingForFirstLogEntry = 1;
            BytePosition = 1;
            }
         switch (++BytePosition) {
            case 1:
               Amps = c >> 1;
               break;
            case 2:
               Amps = (Amps << 8) + c;
               Amps = Amps >> 1;
               break;
            case 3:
               Volts = c >> 1;
            }
         if (Start == Stop) {
            break;
            }
         }
      if (LOG_BFR_SIZE == ++Start)
         Start = LOG_BFR_START;
      }
   LogStr = ConvertLogDataToStr (Amps, Volts);
   Sprint ("LogEntry at ");
   Sprint (LogEntryIndex ,DEC);
   Sprint (" is  ");
   Sprintln (LogStr);
   }


char SendPacket (char* Payload, byte Length) {
   byte Header;

   if (!rf12_recvDone() && rf12_canSend()) {
      sendLen = Length;
      /*
      Sprint ("sending ");
      Sprint ((int) sendLen);
      Sprint (" bytes (");
      for (byte i = 0; i < sendLen; ++i) {
        if (i != 0)
           Sprint (' ');
        Sprint ((int) Payload[i]);
        }
      Sprint (") to ");
      Sprintln ((int) WIRELESS_LINK_ID);
      */
      Header = RF12_HDR_ACK;
      Header |= RF12_HDR_DST | WIRELESS_LINK_ID;
      rf12_sendStart (Header, Payload, sendLen);
      TxTime = millis();
      return 1;
      }
   else {
      //Sprintln ("Can't send packet");
      return 0;
      }
   }


static void UploadLogToWireless (unsigned char Action) {
   static long Start, Stop;
   static char SequenceNumber, Done;
   static char PacketIndex;

   if (1 == Action) {
      if (0 == NumberOfLogEntries) {
         // nothing to upload, so send end packet
         testbuf[0] = 'D';
         SendPacket ((char*) testbuf, 1);
         Sprintln ("Log is empty");
         RunMode = NotLogging;
         return;
         }
      if (BufferHasWrapped) {   
         Start = LogBfrEnd + 1;      // if bfr has wrapped find start of buffer from the end pointer
         if (LOG_BFR_SIZE == Start)
            Start = LOG_BFR_START;
         }
      else 
         Start = LOG_BFR_START;      // else if bfr hasn't wrapped, use the beginning of the bfr
      Stop = LogBfrEnd;
      SequenceNumber = 1;
      Done = 0;
      }
   if (2 == Action) { // ack not received, retransmit packet
      while (0 == SendPacket ((char*) testbuf, PacketIndex));
      return;
      }
   if (Done) {
      // all done, so send end packet
      Sprintln ("Upload completed");
      testbuf[0] = 'D';
      while (0 == SendPacket ((char*) testbuf, 1));
      RunMode = NotLogging;
      }
   PacketIndex = 2;
   while (1) {
      testbuf[PacketIndex++] = ReadEeprom (Start);
      if (Start == Stop) 
         Done = 1;
      else if (LOG_BFR_SIZE == ++Start)
         Start = LOG_BFR_START;
      if ((MAX_PACKET_PAYLOAD_SIZE <= PacketIndex) || Done) {
         // send packet
         testbuf[1] = SequenceNumber++;
         while (0 == SendPacket ((char*) testbuf, PacketIndex));
         break;
         }
      }
   //LogStr = ConvertLogDataToStr (Amps, Volts);
   //Sprint ("LogEntry at ");
   //Sprint (LogEntryIndex ,DEC);
   //Sprint (" is  ");
   //Sprintln (LogStr);??
   }


static void FillLogBfr (long NumberOfEntries) {
   long i;
   static char* DummyInput[] = {"A0.75V12.6",
                                "A-0.13V12.6",
                                "A-0.56V12.6",
                                "A-0.80V12.5",
                                "A-1.05V12.5",                                
                                "A-2.18V12.4",
                                "A-3.56V12.4",
                                "A-3.02V12.3",
                                "A-2.50V12.3",                                
                                "A-1.48V12.4",
                                "A-1.04V12.5",
                                "A-0.68V12.6",
                                "A-0.12V12.6",
                                "A0.27V12.7"};

   ClearLog ();
   for (i=0; NumberOfEntries*2 > i; i++) {
      strcpy ((char*) SerialInputBfr, DummyInput[i % 14]);
      ProcessSample ();
      if (((i % 2000) == 0) && (i != 0))
         Sprintln (i/2, DEC);
      }
   }


static void WriteLogBfrInfo () {
   char i;
   long Temp;

   // write the log buffer end index to eeprom
   Temp = LogBfrEnd;
   for (i=LOG_BFR_START-1; LOG_BFR_START-4 <= i; i--) {
      WriteEeprom (i, Temp & 0xFF);
      Temp = Temp >> 8;
      }
   // write the number of log entries to eeprom
   Temp = NumberOfLogEntries;
   for (i=LOG_BFR_START-5; LOG_BFR_START-6 <= i; i--) {
      WriteEeprom (i, Temp & 0xFF);
      Temp = Temp >> 8;
      }
   // write the wrap time delta to eeprom
   Temp = WrapTimeDelta;
   for (i=LOG_BFR_START-7; LOG_BFR_START-10 <= i; i--) {
      WriteEeprom (i, Temp & 0xFF);
      Temp = Temp >> 8;
      }
   // write the time stamp to eeprom
   Temp = TimeStamp;
   for (i=LOG_BFR_START-11; LOG_BFR_START-14 <= i; i--) {
      WriteEeprom (i, Temp & 0xFF);
      Temp = Temp >> 8;
      }
   }


static void LogData (unsigned char* Values, char Count) {
   char i;
   char BufferJustWrapped = 0;

   SetLed (RED_LED, ON);
   //Sprint ("Loggging data: ");
   //Sprint ((unsigned char) Values[0], DEC);
   //Sprint (", ");
   //Sprint ((unsigned char) Values[1], DEC);
   //Sprint (", ");
   //Sprintln ((unsigned char) Values[2], DEC);
   if (LastVoltValue == Values[2])
      Count--;
   else
      LastVoltValue = Values[2];
   // write the log entry to eeprom
   for (i=0; i < Count; i++) {
      if (LOG_BFR_SIZE == ++LogBfrEnd) {
         LogBfrEnd = LOG_BFR_START;
         BufferJustWrapped = 1;
         }
      WriteEeprom (LogBfrEnd, Values[i]);
      }
   // if buffer just wrapped for first time then write that to eeprom
   if (BufferJustWrapped && ! BufferHasWrapped) {
      BufferHasWrapped = 1;
      WriteEeprom (1, BufferHasWrapped);
      }
   if (!BufferHasWrapped) 
      NumberOfLogEntries++;
   SetLed (RED_LED, OFF);
   }


static void ProcessSample () {
   int Value;
   char Index = -1, Length, i, TensDecimalPointPosition = 3;
   int Sign, Range, AmpsRoundingValue;
   unsigned int Amps, Volts;
   unsigned char Values[3], HighByte;

   //Sprint ("Processing sample: ");
   //Sprintln (SerialInputBfr);
   // convert amp & volt strings to integers and store values in arrays
   // find the V in the string to use as end of amps value, (strpos() is not available)
   for (i=0; 0 != SerialInputBfr[i]; i++)
      if ('V' == SerialInputBfr[i]) {
         Index = i;
         break;
         }
   if (Index == -1) {
      Sprintln ("No V in input bfr");
      return;
      }
   // read in the amps value
   Length = strlen (SerialInputBfr);
   Value = 0;
   Sign = 1;
   Range = 1;
   AmpsRoundingValue = 0;
   for (i=1; i < Index; i++)
      if ('-' == SerialInputBfr[i]) {
         Sign = -1;
         TensDecimalPointPosition++;
         }
      else if (('.' == SerialInputBfr[i]) && (TensDecimalPointPosition == i))
         Range = 10;
      else if ('0' <= SerialInputBfr[i] && '9' >= SerialInputBfr[i])
         Value = 10 * Value + SerialInputBfr[i] - '0';
   if ((10 == Range) && (Value & 1))
      AmpsRoundingValue = 10 * Sign;
   Value = (Value * Sign * Range) + AmpsRoundingValue;
   AmpValues[NextValue] = Value + 1500;    // convert to 0..3000 range
   // read in the volts value
   Value = 0;
   for (i=Index+1; i < Length; i++)
      if ('0' <= SerialInputBfr[i] && '9' >= SerialInputBfr[i])
         Value = 10 * Value + SerialInputBfr[i] - '0';
   VoltValues[NextValue++] = Value - 100;  // convert to 0..50 range
   if (NUMBER_TO_AVERAGE > NextValue)
      return;
   // average the values, init sums to zero before adding all the values 
   Amps = Volts = 0;
   for (i=0; NUMBER_TO_AVERAGE > i; i++) {
      Amps += AmpValues[i];
      Volts += VoltValues[i];
      }
   Value = Amps & 1;            // save whether amps is odd or even
   Amps = (Amps / 2) + Value;   // divide sum by number of values and round up if sum was odd
   Value = Volts & 1;           // save whether volts is odd or even
   Volts = (Volts / 2) + Value; // divide sum by number of values and round up if sum was odd
   //Sprint ("A=");
   //Sprint (Amps, DEC);
   //Sprint (", V=");
   //Sprintln (Volts, DEC);
   // left shift values by 1 and 
   // add 1 to high byte of amps value to mark first byte of log entry
   Amps = Amps << 1;
   HighByte = highByte (Amps);
   Values[0] = (HighByte << 1) + 1;
   Values[1] = Amps & 0xFF;
   Values[2] = Volts << 1;
   // Log the data 
   LogData(Values, 3);
   NextValue = 0;
   }

#define RF_GOTO_SLEEP 0
#define RF_WAKE_UP -1

static void ProcessSerialInput (char c) {
   unsigned int rm;
   //Sprint ("rcvd ");
   //Sprintln (c);
   if (NotLogging == RunMode) {
      if ('0' <= c && '9' >= c)
         InputValue = 10 * InputValue + c - '0';
      else if ('a' <= c && c <='z') {
         switch (c) {
            case 'a': // turn activity LED on or off
               SetLed (RED_LED, InputValue);
               if (InputValue == 1) {
                  rf12_sleep(RF_GOTO_SLEEP);
                  Sprintln ("put the radio to sleep");
                  }
               else
                  rf12_sleep(RF_WAKE_UP);
               break;
            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
               InputValue = InputValue == 8 ? RF12_868MHZ :
                            InputValue == 9 ? RF12_915MHZ : RF12_433MHZ;
               RF12Config.nodeId = (InputValue << 6) + (RF12Config.nodeId & 0x3F);
               saveConfig();
               break;
            case 'c':
               ClearLog ();
               break;
            case 'd':
               DumpLog (InputValue);
               break;
            case 'f':
               FillLogBfr (InputValue);
               break;
            case 'g': // set network group
               RF12Config.group = InputValue;
               saveConfig();
               break;
            case 'i': // set node id
               RF12Config.nodeId = (RF12Config.nodeId & 0xE0) + (InputValue & 0x1F);
               saveConfig();
               break;
            case 'l':
               ClearSerialBfr ();
               RunMode = LoggingWithoutMonitor;
               WriteEeprom (0, RunMode);
               DisconnectedFromMonitorTimeout = WriteLogInfoTimeout = 0;
               Sprintln ("\nLogging w/o monitor (use '!' to get command mode)");
               break;
            case 'm':
               ClearSerialBfr ();
               RunMode = LoggingWithMonitor;
               WriteEeprom (0, RunMode);
               DisconnectedFromMonitorTimeout = WriteLogInfoTimeout = 0;
               Sprintln ("\nLogging with monitor (use '!' to get command mode)");
               break;
            case 'q': // turn quiet mode on or off (don't report bad packets)
                quiet = InputValue;
                break;
            case 's':
               Sprint ("Setting baud rate to ");
               Sprintln (InputValue, DEC);
               delay (100);
               Serial.end();
               Serial.begin (InputValue);
               break;
            default:
               ShowHelp();
            }
         InputValue = 0;
         }
      else if (c == '?')
         DumpProgramInfo();
      else if (c > ' ')
         ShowHelp();
      }
   else if ('!' == c) {
      RunMode = NotLogging;
      WriteEeprom (0, RunMode);
      WriteLogBfrInfo ();
      Sprintln ("\nStopped logging (in command mode)");
      }
   else if (UploadingData != RunMode) {  // in logging mode, so process the input from the TM2025
      if (SERIAL_INPUT_BFR_SIZE == SerialBfrIndex) {
         SerialBfrOverflowCount++;
         Sprintln ("ERROR: serial input buffer overflow");
         // TODO: reset input bfr and state machine and return
         }
      //Sprint ("c=");
      //Sprint (c);
      //Sprint (", CS=");
      //Sprint (CaptureState, DEC);
      //Sprint (", CTF=");
      //Sprintln (CharToFind);
      switch (CaptureState) {
         case LookingForAmps: 
            switch (CharToFind) {
               case ',':
                  if (c == CharToFind)
                     CharToFind = 'A';
                  break;
               case 'A': 
                  if (c == CharToFind)
                     CharToFind = '=';
                  else
                     CharToFind = ',';
                  break;
               case '=': 
                  if (c == CharToFind) {
                     SerialInputBfr[SerialBfrIndex++] = 'A';
                     CaptureState = CapturingAmps;
                     }
                  CharToFind = ',';
                  break;
               default:
                  Sprintln ("ERROR: unknown character to find in LookingForAmps state");
                  // TODO: reset everything
               }
            break;
         case CapturingAmps:
            if (c == CharToFind)
               CaptureState = LookingForVolts;  //fall thru to LookingForVolts case
            else {
               SerialInputBfr[SerialBfrIndex++] = c;
               break;
               }
         case LookingForVolts:
            switch (CharToFind) {
               case ',':
                  if (c == CharToFind)
                     CharToFind = 'V';
                  break;
               case 'V': 
                  if (c == CharToFind)
                     CharToFind = '=';
                  else
                     CharToFind = ',';
                  break;
               case '=': 
                  if (c == CharToFind)
                     SerialInputBfr[SerialBfrIndex++] = 'V';
                     CaptureState = CapturingVolts;
                  CharToFind = ',';
                  break;
               default:
                  Sprintln ("ERROR: unknown character to find in LookingForVolts state");
                  // TODO: reset everything
               }
            break;
         case CapturingVolts:
            if (c == CharToFind) {
               SerialInputBfr[SerialBfrIndex] = 0;
               ProcessSample ();
               SerialBfrIndex = 0;
               CaptureState = LookingForAmps;
               }
            else 
               SerialInputBfr[SerialBfrIndex++] = c;
            break;
         default:
            Sprintln ("ERROR: unknown capture state");
            // TODO: reset everything
         }
      //Sprint ("CS=");
      //Sprint (CaptureState, DEC);
      //Sprint (", CTF=");
      //Sprintln (CharToFind);
      }
   }


void SetNextLedState (byte State, int Time) {
   NextLedState.LedState = State;
   NextLedState.When = Time;
   }


void PrintPacket (int Group, int Header, int Length) {
   if (Group != -1) {
      Sprint ("G ");
      Sprint ((int) Group);
      }
   Sprint ("HDR=");
   Sprint ((int) Header, HEX);
   Sprint ("h LEN=");
   Sprint ((int) Length, DEC);
   Sprint (" (");
   for (byte i = 0; i < Length; ++i) {
      if (i != 0)
         Sprint (' ');
      Sprint ((int) testbuf[i]);
      }
   Sprint (")");
   if (Header & RF12_HDR_ACK)
      Sprintln (" Ack requested");
   else
      Sprintln (" Ack not requested");
   }


char SendPacket (char* Payload) {
   byte Header;

   if (!rf12_recvDone() && rf12_canSend()) {
      sendLen = strlen (Payload);
      /*
      Sprint ("sending ");
      Sprint ((int) sendLen);
      Sprint (" bytes (");
      for (byte i = 0; i < sendLen; ++i) {
        if (i != 0)
           Sprint (' ');
        Sprint ((int) Payload[i]);
        }
      Sprint (") to ");
      Sprintln ((int) WIRELESS_LINK_ID);
      */
      if (AckRequested) {
         Header = RF12_HDR_CTL;
         Header |= RF12Config.nodeId & 0x1F;
         }
      else {
         Header = RF12_HDR_ACK;
         Header |= RF12_HDR_DST | WIRELESS_LINK_ID;
         }
      rf12_sendStart (Header, Payload, sendLen);
      return 1;
      }
   else {
      //Sprintln ("Can't send packet");
      return 0;
      }
   }


void ProcessPacketCharacter (char c) {
   if ('v' == c)   // return version string
      strcpy ((char*) &testbuf[1], SW_VERSION);    
   else if ('r' == c) {   // return runmode
      switch (RunMode) {
         case NotLogging :
            strcpy ((char*) testbuf, "r0");    
            break;
         case LoggingWithMonitor :
            strcpy ((char*) testbuf, "r1");    
            break;
         case LoggingWithoutMonitor :
            strcpy ((char*) testbuf, "r2");    
            break;
         }
      }
   else if ('?' == c) {
      DumpProgramInfoToWireless();
      }
   else if (NotLogging == RunMode) {
      if ('0' <= c && '9' >= c) {
         InputValue = 10 * InputValue + c - '0';
         Sprint ("InputValue = ");
         Sprintln (InputValue);
         }
      else if ('a' <= c && c <='z') {
         switch (c) {
            case 'a': // turn activity LED on or off
                SetLed (RED_LED, InputValue);
                break;
            case 'b': // set band: 4 = 433, 8 = 868, 9 = 915
               InputValue = InputValue == 8 ? RF12_868MHZ :
                            InputValue == 9 ? RF12_915MHZ : RF12_433MHZ;
               RF12Config.nodeId = (InputValue << 6) + (RF12Config.nodeId & 0x3F);
               saveConfig();
               break;
            case 'c':
               ClearLog ();
               break;
            case 'd':
               RunMode = UploadingData;
               break;
            case 'f':
               FillLogBfr (InputValue);
               break;
            case 'g': // set network group
               RF12Config.group = InputValue;
               saveConfig();
               break;
            case 'i': // set node id
               RF12Config.nodeId = (RF12Config.nodeId & 0xE0) + (InputValue & 0x1F);
               saveConfig();
               break;
            case 'l':
               if (InputValue != 0)
                  TimeStamp = InputValue;
               WrapTimeDelta = 0;
               TimeSyncTime = millis();
               ClearSerialBfr ();
               RunMode = LoggingWithoutMonitor;
               WriteEeprom (0, RunMode);
               WriteLogBfrInfo ();
               DisconnectedFromMonitorTimeout = WriteLogInfoTimeout = 0;
               SerialEnabled = 1;
               break;
            case 'q': // turn quiet mode on or off (don't report bad packets)
               quiet = InputValue;
               break;
            case 't':
               DumpProgramInfoToWireless();
               TimeStamp = InputValue;
               WrapTimeDelta = 0;
               TimeSyncTime = millis();
               break;
            default:
               strcpy ((char*) testbuf, UNKNOWN_CMD);
            }
         InputValue = 0;
         }
      else 
         strcpy ((char*) testbuf, UNKNOWN_CMD);
      }
   else if ('!' == c) {
      RunMode = NotLogging;
      SerialEnabled = 0;
      WriteEeprom (0, RunMode);
      WriteLogBfrInfo ();
      }
   else 
      strcpy ((char*) testbuf, UNKNOWN_CMD);
   }


void ParsePacket (int Length) {
   testbuf[Length] = 0;
   for (byte i = 0; i < Length; ++i) 
      ProcessPacketCharacter (testbuf[i]);
   SendPacket ((char*) testbuf);
   if (UploadingData == RunMode)
      UploadLogToWireless (1);  // kick off upload
   //Sprintln ("leaving ParsePacket()");
   }


void setup () {
   char i;
   unsigned char Value;
   byte ButtonState;

   Serial.begin (2400);
   ClearSerialBfr ();
   NextLedState.When = NONE;
   // recover state of log buffer and RunMode from EEPROM
   RunMode = (RunModes) ReadEeprom (0);
   NumberOfHeartbeatBlinks = RunMode + 1;
   BufferHasWrapped = ReadEeprom (1);
   for (i=LOG_BFR_START-4; LOG_BFR_START > i; i++) {
      Value = ReadEeprom (i);
      LogBfrEnd = (LogBfrEnd << 8) + Value;
      }
   for (i=LOG_BFR_START-6; LOG_BFR_START-4 > i; i++) {
      Value = ReadEeprom (i);
      NumberOfLogEntries = (NumberOfLogEntries << 8) + Value;
      }
   for (i=LOG_BFR_START-10; LOG_BFR_START-6 > i; i++) {
      Value = ReadEeprom (i);
      WrapTimeDelta = (WrapTimeDelta << 8) + Value;
      }
   for (i=LOG_BFR_START-14; LOG_BFR_START-10 > i; i++) {
      Value = ReadEeprom (i);
      TimeStamp = (TimeStamp << 8) + Value;
      }
   if (rf12_config(0)) {
      RF12Config.nodeId = eeprom_read_byte(RF12_EEPROM_ADDR);
      RF12Config.group = eeprom_read_byte(RF12_EEPROM_ADDR + 1);
      }
   else {
      RF12Config.nodeId = 0x42; // node B(2) @ 433 MHz
      RF12Config.group = 0xD4;  // 212
      saveConfig();
      }

   ButtonState = blinkPlug.buttonCheck();
   //Sprint ("Backdoor Button State = ");
   //Sprintln (ButtonState, DEC);
   if (ButtonState == BlinkPlug::ON2) {
      Sprintln ("Backdoor button pressed");
      RunMode = NotLogging;
      SerialEnabled = 1;
      }
   else if (NotLogging == RunMode)
      SerialEnabled = 0;

   switch (RunMode) {
      case NotLogging:
         ShowHelp ();
         break;
      case LoggingWithMonitor:
         Sprintln ("Logging with monitor (use '!' to get command mode)");
         break;
      case LoggingWithoutMonitor:
         Sprintln ("Logging w/o monitor (use '!' to get command mode)");
         break;
      }
   /*
   if (!SD.begin(9)) {
      Serial.println ("SD card initialization failed!");
      SetLed (RED_LED, ON);
      }
   */
   Sprint ("\nCurrent radio configuration: ");
   rf12_config();
   }


void loop () {
   unsigned long CurrentTime, DeltaTime;
   unsigned int Group, Header, Length;

   if  (Serial.available ()) {
      DisconnectedFromMonitorTimeout = 0;
      if (SerialEnabled)
         ProcessSerialInput (Serial.read());
      else
         Serial.read();   // throw away incoming TM2025 data if not logging
      }
   CurrentTime = millis();
   // check for missed ack
   if (UploadingData == RunMode) 
      if (0 != TxTime) {
         if (CurrentTime < TxTime)
            // deal with clock rollover
            TxTime = 1; 
         DeltaTime = CurrentTime - TxTime;
         if (ACK_TIMEOUT < DeltaTime) {
            TxTime = 0;
            UploadLogToWireless (2);
            }
         }
   if (CurrentTime < LastTime)
      // deal with clock rollover
      LastTime = 0;                    
   DeltaTime = CurrentTime - LastTime;
   if (DeltaTime > HEARTBEAT_PERIOD) {
      // reset LastTime for next heartbeat
      LastTime = CurrentTime;
      // turn on the heartbeat led
      SetLed (GREEN_LED, ON);
      // schedule the next state to be 'off' and the transition time to the length of 'on' time
      SetNextLedState (OFF, LED_ON_TIME);
      // set the number of blinks to 1 for 'not logging', atmega328
      // 2 for 'logging with monitor', and 3 for 'logginh w/o monitor'
      NumberOfHeartbeatBlinks = RunMode + 1;
      if (WRITE_LOG_INFO_TIMEOUT_IN_HEARTBEATS <= ++WriteLogInfoTimeout) {
         // write the log buffer info to flash every 5 minutes
         WriteLogBfrInfo ();
         WriteLogInfoTimeout = 0;
         }
      if (DISCONNECTED_FROM_MONITOR_TIMEOUT_IN_HEARTBEATS <= ++DisconnectedFromMonitorTimeout) {
         // save log buffer info if no characters received from battery monitor for 4 seconds
         WriteLogBfrInfo ();
         DisconnectedFromMonitorTimeout = 0;
         }
      }
   else if (NextLedState.When != NONE) {
      // something was scheduled for the heartbeat led
      if (DeltaTime >= NextLedState.When) {
         // set the led to the next state
         SetLed (GREEN_LED, NextLedState.LedState);
         if (NextLedState.LedState == OFF) {
            // record that led finished another blink
            --NumberOfHeartbeatBlinks;
            if (NumberOfHeartbeatBlinks == 0)
               // all done blinking the led for this heartbeat cycle
               SetNextLedState (OFF, NONE);
            else
               // schedule the next 'on' transition
               SetNextLedState (ON, DeltaTime + LED_OFF_TIME);
            }
         else
            // schedule the next 'off' transition
            SetNextLedState (OFF, DeltaTime + LED_ON_TIME);
         }
      }

   if (rf12_recvDone()) {
      Sprintln ("Rcvd new wireless packet");
      if (rf12_crc != 0) 
         if (quiet)
            return;
         else {
            Sprint ("CRC BAD, ");
            if (Length > 20)         // print at most 20 bytes if crc is wrong
               Length = 20;
            }
      SetLed (RED_LED, ON);
      if (RF12Config.group == 0) 
         Group = rf12_grp;
      else
         Group = -1;
      Header = rf12_hdr;
      Length = rf12_len;
      for (byte i = 0; i < Length; ++i) 
         testbuf[i] = rf12_data[i];
      PrintPacket (Group, Header, Length);
      if ((rf12_crc == 0) && (Header & RF12_HDR_ACK)) 
         AckRequested = 1;
      else 
         AckRequested = 0;
      if (Header & RF12_HDR_CTL) {
         if (UploadingData == RunMode)
            UploadLogToWireless (0);
         }
      else
         ParsePacket (Length);
      SetLed (RED_LED, OFF);
      }
   }
