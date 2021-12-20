/* Open Sound Control for Audio Library for Teensy 3.x, 4.x
 * Copyright (c) 2021, Jonathan Oakley, teensy-osc@0akley.co.uk
 *
 * Development of this library was enabled by PJRC.COM, LLC by sales of
 * Teensy and Audio Adaptor boards, implementing libraries, and maintaining
 * the forum at https://forum.pjrc.com/ 
 *
 * Please support PJRC's efforts to develop open source software by 
 * purchasing Teensy or other PJRC products.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/* 
 *  Empty audio design for use with Audio Design Tool++
 *  
 *   "C:\Program Files (x86)\Arduino\hardware\tools\arm\bin\arm-none-eabi-addr2line" -e 
 */

#include <OSCBundle.h>

#include <SLIPEncodedUSBSerial1.h>
//#include <usb_serial.h>

#include <Audio.h>
#include <Wire.h>
//#include <SPI.h>
#include <SD.h>
//#include <SerialFlash.h>
#define DBG_SERIAL Serial

#include "OSCAudioBase.h"

// set this to the hardware serial port you wish to use
#define HWSERIALPORT Serial7

//SLIPEncodedSerial HWSERIAL(HWSERIALPORT);

SLIPEncodedUSBSerial1 OSC_SERIAL(SerialUSB1);

const int ledPin = 13;
int ledState = LOW;             // ledState used to set the LED
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long currentMillis = 0;
unsigned long currentInterval = 0;
unsigned long ledBlinkOnInterval = 100;
unsigned long ledBlinkOffInterval = 2000;

void printDirectory(File dir, File prevDir);
void printFiles(File dir);

void setup() {
	DBG_SERIAL.begin(115200);
    unsigned long ms = millis();
    while (!DBG_SERIAL) { if ((millis() - ms) > 10000) break; }
    Serial.println("starting stuff...");
  //-------------------------------
  AudioMemory(50); // no idea what we'll need, so allow plenty
  //-------------------------------
  if (CrashReport && DBG_SERIAL)
  {
    DBG_SERIAL.println(CrashReport);
    CrashReport.clear();
  }
  //testSanitise();
 // listObjects();

 while (!(SD.begin(BUILTIN_SDCARD))) 
  {
      Serial.println("Unable to access the SD card");
      delay(500);
  }
  File root = SD.open("/");
  
  Serial.println("----------------\nFiles:");
  printFiles(root);
  Serial.println("----------------");
  root.close();
}
void printFiles(File dir) {
while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

OSCBundle* replyStack;
void routeAudio(OSCMessage& msg, int addressOffset)
{
  DBG_SERIAL.println("audio message!");
  OSCAudioBase::routeAll(msg,addressOffset,*replyStack);
}


void routeDynamic(OSCMessage& msg, int addressOffset)
{
#if defined(SAFE_RELEASE)  
  DBG_SERIAL.println("dynamic objects message!");
  OSCAudioBase::routeDynamic(msg,addressOffset,*replyStack);
#else
  DBG_SERIAL.println("dynamic objects not available!");
#endif // defined(SAFE_RELEASE)  
}


void listObjects(void)
{
  OSCAudioBase* obj=OSCAudioBase::getFirst();

  while (NULL != obj)
  {
    DBG_SERIAL.printf("%s is at %08X\n",obj->name,(uint32_t) obj);
    DBG_SERIAL.flush();
    obj = obj->getNext();
  }
}


void testSanitise()
{
  char buf[20];
  OSCAudioBase::sanitise("Hello world!",buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::sanitise("[Hello world!]",buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::trimUnderscores(buf,buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::sanitise("*[H]e{}llo#, world!?/",buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::trimUnderscores(buf,buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::sanitise("#*,/? []{}",buf); DBG_SERIAL.println(buf);  
  OSCAudioBase::trimUnderscores(buf,buf); DBG_SERIAL.printf("<%s>\n",buf);  
}

//-----------------------------------------------------------------------------------------------------------------
// save blob to filesystem
void saveFS(OSCMessage& msg, int addressOffset)
{
  char fn[50];
  uint8_t buf[65];
  int remain = msg.getBlobLength(1);
  int idx = 0;
  OSCAudioBase::error retval = OSCAudioBase::OK;
  OSCMessage& repl = OSCAudioBase::staticPrepareReplyResult(msg,*replyStack);
  File saveFile;
  
  msg.getString(0,fn,50);
  saveFile = SD.open(fn,FILE_WRITE_BEGIN);
  if (saveFile)
  {
    Serial.print("Save ");
    while (remain > 0)
    {
      int toGet = remain>64?64:remain;
      int got = msg.getBlob(1,buf,64,idx,toGet);
      int wrote = saveFile.write(buf,got);
      buf[64] = 0;
      if (remain < 64)
        buf[remain] = 0;
      remain -= got;
      Serial.printf("<%s> @ %d: wrote %d, %d left\n",buf,idx,wrote,remain); Serial.flush();
      idx += got;
    }
    saveFile.close();
    repl.add("saved");
    Serial.printf(" (length %d) to %s\n",msg.getBlobLength(1),fn); Serial.flush();
  }
  else
  {
    retval = OSCAudioBase::NOT_FOUND;
    repl.add("failed");
  }    
  repl.add(retval);
}

// retrieve blob from filesystem
// Annoyingly, CNMAT / OSC doesn't appear to be able to build a blob's content incrementally,
// so this operation has to load the entire file in one go, then create another copy of it
// in the messaging structure, resulting in heap fragmentation.
void sendFS(OSCMessage& msg, int addressOffset)
{
  char fn[50];
  uint8_t* buf;
  int remain;
  bool success = false;
  OSCAudioBase::error retval = OSCAudioBase::OK;
  OSCMessage& repl = OSCAudioBase::staticPrepareReplyResult(msg,*replyStack);
  File sendFile;
  
  msg.getString(0,fn,50);
  sendFile = SD.open(fn);
  if (sendFile)
  {
    remain = sendFile.size();
    Serial.print("Send ");
  
    if (NULL != (buf = (uint8_t*) malloc(remain+1)))
    {
      sendFile.read(buf,remain);
      repl.add(buf,remain); 
      buf[remain] = 0;
      //Serial.println((char*) buf);
      Serial.printf("File size:%d", remain);
      free(buf);
      success = true;
    }
    else
    {
      retval = OSCAudioBase::NO_MEMORY;
      repl.add("failed");
    }
    sendFile.close();
  }
  else
  {
    retval = OSCAudioBase::NOT_FOUND;
    repl.add("failed");
  }  
  repl.add(retval);
}


// Delete file from filesystem
void deleteFS(OSCMessage& msg, int addressOffset)
{
  char fn[50];
  bool success;
  OSCMessage& repl = OSCAudioBase::staticPrepareReplyResult(msg,*replyStack);
  
  msg.getString(0,fn,50);
  
  Serial.print("Delete ");

  if (true == (success = SD.remove(fn)))
    repl.add(fn); 
  else
    repl.add("failed");
  
  repl.add(success
            ?OSCAudioBase::OK
            :OSCAudioBase::NOT_FOUND);
}


// Retrieve blob from filesystem and process it as an OSC-encoded packet
static int loadFSdepth = 0; // do a recursion check
void loadFS(OSCMessage& msg, int addressOffset)
{
  char fn[50];
  uint8_t* buf;
  int remain;
  OSCAudioBase::error retval = OSCAudioBase::OK;
  OSCMessage& repl = OSCAudioBase::staticPrepareReplyResult(msg,*replyStack);
  OSCBundle myReplies;
  File loadFile;

  loadFSdepth++;
  myReplies.add("/load");
  msg.getString(0,fn,50);
  loadFile = SD.open(fn);
  if (loadFile)
  {
    remain = loadFile.size();
    Serial.print("Load ");
  
    if (NULL != (buf = (uint8_t*) malloc(remain+1)))
    {
      loadFile.read(buf,remain);
      //repl.add(buf,remain); // attaches file contents as blob
      repl.add(fn); // attaches file name as string
      buf[remain] = 0;
      if ('#' == buf[0]) // got a bundle
      {
        OSCBundle bndl;
        bndl.fill(buf,remain);
        Serial.println("bundle"); Serial.flush();
        processBundle(&bndl,myReplies);
      }
      else if ('/' == buf[0]) // got a message
      {
        OSCMessage msg;
        msg.fill(buf,remain);
        Serial.println("message"); Serial.flush();
        processMessage(&msg,myReplies);      
      }
      Serial.println((char*) buf);  Serial.flush();
      free(buf);
    }
    else
    {
      retval = OSCAudioBase::NO_MEMORY;
      repl.add("failed");
    }
    loadFile.close();
  }
  else
  {
    retval = OSCAudioBase::NOT_FOUND;
    repl.add("failed");
  }

  repl.add(retval);
  
  // tack myReplies on here:
  {
    int msgN = myReplies.size(); // number of messages in reply to file execution
    for (int i=0;i<msgN;i++)
      replyStack->add(*myReplies.getOSCMessage(i));
  }
  loadFSdepth--;
}

// route messages to filing system:
//   /load<s>: string s is the name of a file containing an OSC bundle which should be fed back into the OSC decoder
//   /save<s><b>: save blob b to filename s; the blob can be anything but is likely to be either JSON or OSC
//   /send<s>: retrieve a file and send it in the reply to the GUI (or other client) as a blob
//   /delete<s>: delete a file
void routeFS(OSCMessage& msg, int addressOffset)
{
  Serial.println("filesystem message!");
  if (OSCAudioBase::isStaticTarget(msg,addressOffset,"/save","sb"))
    saveFS(msg,addressOffset);
  else if (OSCAudioBase::isStaticTarget(msg,addressOffset,"/send","s"))
    sendFS(msg,addressOffset);
  else if (OSCAudioBase::isStaticTarget(msg,addressOffset,"/load","s"))
    loadFS(msg,addressOffset);
  else if (OSCAudioBase::isStaticTarget(msg,addressOffset,"/delete","s"))
    deleteFS(msg,addressOffset);
}


void processMessage(OSCMessage* msg,OSCBundle& reply)
{
  char prt[200];
  OSCBundle* replyPush = replyStack;
  replyStack = &reply;
  
  if (!msg->hasError())
  {
    msg->getAddress(prt);  
    DBG_SERIAL.println(prt);
    DBG_SERIAL.flush();
  
    msg->route("/teensy*/audio",routeAudio); // see if this object can use the message
    msg->route("/teensy*/dynamic",routeDynamic); // see if this object can use the message
    msg->route("/teensy*/fs",routeFS);           // or this one
  }
  else
    DBG_SERIAL.println("error in msg");
  
  replyStack = replyPush;
}

void processBundle(OSCBundle* bndl,OSCBundle& reply)
{
  int bndlSize = bndl->size();

  if (!bndl->hasError())  
  {
    for (int i=0;i<bndlSize;i++)
    {
      OSCMessage* msg = bndl->getOSCMessage(i); 
      DBG_SERIAL.printf("Message %d\n",i);
      processMessage(msg,reply);   
    }  
  }
  else
  {
    DBG_SERIAL.printf("error %d in bundle\n",(int) bndl->getError());
    
    for (int i=0;i<bndlSize;i++)
    {
      OSCMessage* msg = bndl->getOSCMessage(i); 
      DBG_SERIAL.printf("error %d in message %d\n",(int) msg->getError(),i);
    }
  }  
}

void sendReply(OSCBundle& reply)
{
  // for debug
  //reply.send(Serial); 
  Serial.printf("\nReply has %d messages, %d errors\n",reply.size(),reply.hasError());  

  // for real!
  OSC_SERIAL.beginPacket();
  reply.send(SerialUSB1); 
  OSC_SERIAL.endPacket();
}

void oscstuff()
{
    OSCBundle bndl;
  OSCBundle reply;
  OSCMessage msg;
  long long tt = 0; //0x4546474841424344; // for debug: ABCDEFGH
  char firstCh = 0;
  int msgLen;
  
  
  while (!OSC_SERIAL.endofPacket())
  {
    
    msgLen = OSC_SERIAL.available();
    while (msgLen--)
    {
      char c = OSC_SERIAL.read();
      // figure out if it's a message or a bundle
      if (0 == firstCh)
        firstCh = c;
      if ('#' == firstCh)
        bndl.fill((uint8_t) c); // simple messages should result in a 1-message "bundle", but don't
      else
        msg.fill((uint8_t) c); // so process them specifically
    }
  }


  reply.setTimetag((uint8_t*) &tt).add("/reply"); // create first message with reply address: used for all messages
  
  if ('#' == firstCh)
  {
    if (!bndl.hasError())  
    {
      int bndlSize = bndl.size();
      
      for (int i=0;i<bndlSize;i++)
      {
        OSCMessage* msg = bndl.getOSCMessage(i); 
        DBG_SERIAL.printf("Message %d\n",i);
        processMessage(msg,reply);   
      }  
    }
    else
    {
      DBG_SERIAL.printf("error %d in bundle\n",(int) bndl.getError());
      int bndlSize = bndl.size();
      
      for (int i=0;i<bndlSize;i++)
      {
        OSCMessage* msg = bndl.getOSCMessage(i); 
        DBG_SERIAL.printf("error %d in message %d\n",(int) msg->getError(),i);
      }
    }
    sendReply(reply);
    listObjects();
  }
  else 
  {
    if ('/' == firstCh) 
    {
      processMessage(&msg,reply);   
      sendReply(reply);
      listObjects();
    }
  }
  DBG_SERIAL.println();
}

void blinkLedTask(void)
{
    currentMillis = millis();
    currentInterval = currentMillis - previousMillis;
    
    if (ledState == LOW)
    {
        if (currentInterval > ledBlinkOffInterval)
        {
            previousMillis = currentMillis;
            ledState = HIGH;
            digitalWrite(ledPin, HIGH);
        }
    }
    else
    {
        if (currentInterval > ledBlinkOnInterval)
        {
            previousMillis = currentMillis;
            ledState = LOW;
            digitalWrite(ledPin, LOW);
        }
    }
}


//-----------------------------------------------------------------------------------------------------------------
// work with SLIP-protocol serial port:
void updateOSC()
{
  static enum {boot,reading,processing} state = boot;
  static OSCBundle bndl;
  static OSCBundle reply;
  static OSCMessage msg;
  long long tt = 0; //0x4546474841424344; // for debug: ABCDEFGH
  static char firstCh = 0;
  int msgLen;

  switch (state)
  {
    case boot:
      Serial.print("Waiting...");
      bndl.empty();
      reply.empty();
      msg.empty();
      firstCh = 0;
      state = reading;
      break;
    
    case reading:
      if (!OSC_SERIAL.endofPacket())
      {    
        msgLen = OSC_SERIAL.available(); // only ever returns 0 or 1, actually
        while (msgLen--)
        {
          char c = OSC_SERIAL.read();
          // figure out if it's a message or a bundle
          if (0 == firstCh)
            firstCh = c;
          if ('#' == firstCh)
            bndl.fill((uint8_t) c); // simple messages should result in a 1-message "bundle", but don't
          else
            msg.fill((uint8_t) c); // so process them specifically
        }
      }
      else
        state = processing;
      break;

    case processing:  
      Serial.println("processing!");
      reply.setTimetag((uint8_t*) &tt).add("/reply"); // create first message with reply address: used for all messages
      
      if ('#' == firstCh)
      {
        processBundle(&bndl,reply);
        sendReply(reply);
        listObjects();
      }
      else 
      {
        if ('/' == firstCh) 
        {
          processMessage(&msg,reply);   
          sendReply(reply);
          listObjects();
        }
      }
      Serial.println();
      state = boot;
      break;
  }
}

// work with SLIP-protocol serial port:
void loop()
{
    oscstuff();
  //updateOSC();
  blinkLedTask();

  
}

