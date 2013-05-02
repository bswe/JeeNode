/*
  SD card read/write
 
 This example shows how to read and write data to and from an SD card file 	
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK  - pin 13
 ** CS   - pin 9    (for JeeNode)
 
 created   Nov 2010
 by David A. Mellis
 updated 2 Dec 2010
 by Tom Igoe
 modified 27 Feb 2012
 by William Bollenbacher
 
 This example code is in the public domain.
 */
 
#include <SD.h>

File myFile;
unsigned long StartTime, StopTime;
bool SetupFailed = false;
char OutputBfr[50];
Sd2Card card;
SdVolume volume;
SdFile root;
uint32_t volumesize;

void setup() {
   // Note that even if it's not used as the CS pin, the hardware SS pin 
   // (10 on most Arduino boards, 53 on the Mega) must be left as an output 
   // or the SD library functions will not work. 
   pinMode (10, OUTPUT);
   Serial.begin (57600);
   Serial.println ("Initializing SD card...");
   StartTime = micros();
   if (! SD.begin(9)) {
      Serial.println ("initialization failed!");
      SetupFailed = true;
      return;
      }
   StopTime = micros();
   sprintf (OutputBfr, "initialization done: it took %ld us", StopTime - StartTime);
   Serial.println (OutputBfr);
   // print the type of card
   card = SD.Card();
   Serial.print ("Card type: ");
   switch (card.type()) {
      case SD_CARD_TYPE_SD1:
         Serial.println ("SD1");
         break;
      case SD_CARD_TYPE_SD2:
         Serial.println ("SD2");
         break;
      case SD_CARD_TYPE_SDHC:
      Serial.println ("SDHC");
         break;
      default:
         Serial.println ("Unknown");
      }
  
   // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
   if (! volume.init (card)) {
      Serial.println ("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
      return;
      }
   
   // print the type and size of the first FAT-type volume
   sprintf (OutputBfr, "Volume type is FAT%d", volume.fatType());
   Serial.println (OutputBfr);
  
   volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
   volumesize *= volume.clusterCount();       // we'll have a lot of clusters
   volumesize *= 512;                         // SD card blocks are always 512 bytes
   sprintf (OutputBfr, "Volume size (bytes):  %ld", volumesize);
   Serial.println (OutputBfr);
   volumesize /= 1024;
   sprintf (OutputBfr, "Volume size (Kbytes): %ld", volumesize);
   Serial.println (OutputBfr);
   volumesize /= 1024;
   sprintf (OutputBfr, "Volume size (Mbytes): %ld", volumesize);
   Serial.println (OutputBfr);
  
   Serial.println ("\nFiles found on the card (name, date and size in bytes): ");
   root.openRoot (volume);
   // list all files in the card with date and size
   root.ls (LS_R | LS_DATE | LS_SIZE);
   }

void loop() {
   int Char;
   char FileName[20];
   int FileNumber = 0;

   if (SetupFailed) {
      while (true)
         delay (10000);
      return;
      }

   Serial.println ("SD Card read/write tests: press any key to run tests>");
   while (! Serial.available())
      delay (1000);
   Serial.read();

   do
      sprintf (FileName, "tst%d.txt", FileNumber++);
   while (SD.exists (FileName));

   // open the file. note that only one file can be open at a time,
   // so you have to close this one before opening another.
   sprintf (OutputBfr, "Opening %s for writing...", FileName);
   Serial.println (OutputBfr);
   StartTime = micros();
   myFile = SD.open (FileName, FILE_WRITE);
   StopTime = micros();
   sprintf (OutputBfr, "open took %ld us", StopTime - StartTime);
   Serial.println (OutputBfr);
     
   // if the file opened okay, write to it and close it:
   if (myFile) {
      sprintf (OutputBfr, "Writing to %s...", FileName);
      Serial.println (OutputBfr);
      StartTime = micros();
      myFile.println ("testing 1, 2, 3.");
      StopTime = micros();
      sprintf (OutputBfr, "write took %ld us", StopTime - StartTime);
      Serial.println (OutputBfr);
      sprintf (OutputBfr, "Closing %s...", FileName);
      Serial.println (OutputBfr);
      StartTime = micros();
      myFile.close();
      StopTime = micros();
      sprintf (OutputBfr, "close took %ld us", StopTime - StartTime);
      Serial.println (OutputBfr);
      } 
   else {
      // if the file didn't open, print an error:
      sprintf (OutputBfr, "error opening %s for writing!", FileName);
      Serial.println (OutputBfr);
      }
     
   // re-open the file for reading:
   sprintf (OutputBfr, "Opening %s for reading...", FileName);
   Serial.println (OutputBfr);
   StartTime = micros();
   myFile = SD.open (FileName);
   StopTime = micros();
   sprintf (OutputBfr, "open took %ld us", StopTime - StartTime);
   Serial.println (OutputBfr);
   if (myFile) {
      sprintf (OutputBfr, "Reading %s:", FileName);
      Serial.println (OutputBfr);
      // read from the file until EOF:
      while (myFile.available()) {
         StartTime = micros();
    	   Char = myFile.read();
         StopTime = micros();
         sprintf (OutputBfr, "<%c> read took %ld us", Char, StopTime - StartTime);
         Serial.println (OutputBfr);
         }
      // close the file:
      myFile.close();
      } 
   else {
  	   // if the file didn't open, print an error:
      sprintf (OutputBfr, "error opening %s for reading!", FileName);
      Serial.println (OutputBfr);
      }
   }


