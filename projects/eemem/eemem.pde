// Hooking up a 125 kbyte (128000 bytes or 1 mbit) M24M01 serial EEPROM 
// (from STMicroelectronics) via I2C.
//
// see http://www.jeelabs.org/2009/07/06/external-memory/
// 5/14/2011 (Bollenbacher Software Engineering)

// the M24M01 serial EEPROM is connected as follows (JeeNode v3 pinout):
//		pin 1, 2, 3, 7: not connected
//		pin 4: GND, port pin 3
//		pin 5: SDA, DIO, port pin 2
//		pin 6: SCL, AIO, port pin 5
//		pin 8: +3V, port pin 4

#include <Ports.h>
#include <RF12.h> // needed to avoid a linker error 

PortI2C EepromPort (1);                  // using port 1 on the JeeNode
DeviceI2C LowBank (EepromPort, 0x50);
DeviceI2C HighBank (EepromPort, 0x51);
unsigned long Address = 127000;
unsigned char NewValueDelta = 50;
unsigned char WriteDelay = 5;            // 5 ms delay for m24m01 to complete write
char RunTest = 0;
unsigned long ErrorCount = 0;


void setup() {
	Serial.begin (57600);
	Serial.print( "[eeprom test - delta value = ");
   Serial.print (int (NewValueDelta));
   Serial.print (", WriteDelay = ");
   Serial.print (int (WriteDelay));
   Serial.println ("]");
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
   /*
   Serial.print (",  writing ");
	Serial.print ((unsigned char) Value, DEC);
   Serial.print (" to LBA ");
	Serial.print ((unsigned long) Address, DEC);
   */
	LowBank.send();
	LowBank.write(Address >> 8);
	LowBank.write(Address & 0xFF);	
	LowBank.write(Value);
	LowBank.stop();
   }


void WriteHighBank (unsigned long Address, unsigned char Value) {
   /*
   Serial.print (",  writing ");
	Serial.print ((unsigned char) Value, DEC);
   Serial.print (" to HBA ");
	Serial.print ((unsigned long) Address, DEC);
   */
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
	delay (WriteDelay);
   }


void loop() {
   unsigned char OldValue, NewValue;

   if (RunTest == 0)
      if (Serial.available()) {
         RunTest = 1;
         Serial.println ("running test...");
         }
      else
         return;

   OldValue = ReadEeprom (Address);

   WriteEeprom (Address, (OldValue + NewValueDelta) % 0xFF);

   NewValue = ReadEeprom (Address);

   if (NewValue != ((OldValue + NewValueDelta) % 0xFF)) {
      Serial.print ("Failed to change addr ");
      Serial.print ((unsigned long) Address, DEC);
      Serial.print (" from ");
	   Serial.print ((unsigned char) OldValue, DEC);
      Serial.print (" to ");
	   Serial.println ((unsigned char) ((OldValue + NewValueDelta) % 0xFF), DEC);
      ErrorCount++;
      }

   Address = (Address + 1) % 0x1F400;
   if (Address%0x1000 == 0) {
      Serial.print ("Addres = ");
      Serial.print ((unsigned long) Address, DEC);
      Serial.print (",  ErrorCount = ");
      Serial.println ((unsigned long) ErrorCount, DEC);
      }
   }
