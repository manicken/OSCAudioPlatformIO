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
#include <MD_MIDIFile.h>

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

MD_MIDIFile midiFile;


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

  Serial.printf("midi file load: %d", midiFile.load("furelise.mid"));
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
  }
  else
    DBG_SERIAL.println("error in msg");
  
  replyStack = replyPush;
}


void sendReply(OSCBundle& reply)
{
  // for debug
  //reply.send(Serial); 
  DBG_SERIAL.printf("\nReply has %d messages\n",reply.size());  

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

// work with SLIP-protocol serial port:
void loop()
{
    
  oscstuff();
  blinkLedTask();

  
}

void serialEventUSB1() {
  //Serial.print(lastMSec / 1000.0 );
  Serial.print("==secs last lps calc USB1::\t");
  //dualTrigger = 1;
  //while ( SerialUSB1.available() ) SerialUSB1.print( (char)SerialUSB1.read());
  //logEvent( 2 );
}
