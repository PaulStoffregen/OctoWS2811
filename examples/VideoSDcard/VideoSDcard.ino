/*  OctoWS2811 VideoSDcard.ino - Video on LEDs, played from SD Card
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2014 Paul Stoffregen, PJRC.COM, LLC

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

Update: The programs to prepare the SD card video file have moved to "extras"
https://github.com/PaulStoffregen/OctoWS2811/tree/master/extras

  The recommended hardware for SD card playing is:

    Teensy 3.1:     http://www.pjrc.com/store/teensy31.html
    Octo28 Apaptor: http://www.pjrc.com/store/octo28_adaptor.html
    SD Adaptor:     http://www.pjrc.com/store/wiz820_sd_adaptor.html
    Long Pins:      http://www.digikey.com/product-search/en?keywords=S1082E-36-ND

  See the included "hardware.jpg" image for suggested pin connections,
  with 2 cuts and 1 solder bridge needed for the SD card pin 3 chip select.
 
  Required Connections
  --------------------
    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
    pin 14: LED strip #2    All 8 are the same length.
    pin 7:  LED strip #3
    pin 8:  LED strip #4    A 100 to 220 ohm resistor should used
    pin 6:  LED strip #5    between each Teensy pin and the
    pin 20: LED strip #6    wire to the LED strip, to minimize
    pin 21: LED strip #7    high frequency ringining & noise.
    pin 5:  LED strip #8
    pin 15 & 16 - Connect together, but do not use
    pin 4:  Do not use

    pin 3:  SD Card, CS
    pin 11: SD Card, MOSI
    pin 12: SD Card, MISO
    pin 13: SD Card, SCLK
*/

#include <OctoWS2811.h>
#include <SPI.h>
#include <SD.h>
#include <Audio.h>
#include <Wire.h>

#define LED_WIDTH    60   // number of LEDs horizontally
#define LED_HEIGHT   32   // number of LEDs vertically (must be multiple of 8)

#define FILENAME     "VIDEO.BIN"

const int ledsPerStrip = LED_WIDTH * LED_HEIGHT / 8;
DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6];
elapsedMicros elapsedSinceLastFrame = 0;
bool playing = false;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, WS2811_800kHz);
File videofile;

AudioPlayQueue     audio;
AudioOutputAnalog  dac;
AudioConnection    patchCord1(audio, dac);


void setup() {
  AudioMemory(40);
  //while (!Serial) ;
  delay(50);
  Serial.println("VideoSDcard");
  leds.begin();
  leds.show();
  if (!SD.begin(3)) stopWithErrorMessage("Could not access SD card");
  Serial.println("SD card ok");
  videofile = SD.open(FILENAME, FILE_READ);
  if (!videofile) stopWithErrorMessage("Could not read " FILENAME);
  Serial.println("File opened");
  playing = true;
  elapsedSinceLastFrame = 0;
}

// read from the SD card, true=ok, false=unable to read
// the SD library is much faster if all reads are 512 bytes
// this function lets us easily read any size, but always
// requests data from the SD library in 512 byte blocks.
//
bool sd_card_read(void *ptr, unsigned int len)
{
  static unsigned char buffer[512];
  static unsigned int bufpos = 0;
  static unsigned int buflen = 0;
  unsigned char *dest = (unsigned char *)ptr;
  unsigned int n;

  while (len > 0) {
    if (buflen == 0) {
      n = videofile.read(buffer, 512);
      if (n == 0) return false;		
      buflen = n;
      bufpos = 0;
    }
    unsigned int n = buflen;
    if (n > len) n = len;
    memcpy(dest, buffer + bufpos, n);
    dest += n;
    bufpos += n;
    buflen -= n;
    len -= n;
  }
  return true;
}

// skip past data from the SD card
void sd_card_skip(unsigned int len)
{
  unsigned char buf[256];

  while (len > 0) {
    unsigned int n = len;
    if (n > sizeof(buf)) n = sizeof(buf);
    sd_card_read(buf, n);
    len -= n;
  }
}


void loop()
{
  unsigned char header[5];

  if (playing) {
    if (sd_card_read(header, 5)) {
      if (header[0] == '*') {
        // found an image frame
        unsigned int size = (header[1] | (header[2] << 8)) * 3;
        unsigned int usec = header[3] | (header[4] << 8);
        unsigned int readsize = size;
	// Serial.printf("v: %u %u", size, usec);
        if (readsize > sizeof(drawingMemory)) {
          readsize = sizeof(drawingMemory);
        }
        if (sd_card_read(drawingMemory, readsize)) {
          // Serial.printf(", us = %u", (unsigned int)elapsedSinceLastFrame);
          // Serial.println();
          while (elapsedSinceLastFrame < usec) ; // wait
          elapsedSinceLastFrame -= usec;
          leds.show();
        } else {
          error("unable to read video frame data");
          return;
        }
        if (readsize < size) {
          sd_card_skip(size - readsize);
        }
      } else if (header[0] == '%') {
        // found a chunk of audio data
        unsigned int size = (header[1] | (header[2] << 8)) * 2;
        // Serial.printf("a: %u", size);
	// Serial.println();
	while (size > 0) {
	  unsigned int len = size;
	  if (len > 256) len = 256;
	  int16_t *p = audio.getBuffer();
	  if (!sd_card_read(p, len)) {
	    error("unable to read audio frame data");
            return;
	  }
	  if (len < 256) {
            for (int i=len; i < 256; i++) {
              *((char *)p + i) = 0;  // fill rest of buffer with zero
            }
	  }
          audio.playBuffer();
	  size -= len;
	}
      } else {
        error("unknown header");
        return;
      }
    } else {
      error("unable to read 5-byte header");
      return;
    }
  } else {
    delay(2000);
    videofile = SD.open(FILENAME, FILE_READ);
    if (videofile) {
      Serial.println("File opened");
      playing = true;
      elapsedSinceLastFrame = 0;
    }
  }
}

// when any error happens during playback, close the file and restart
void error(const char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  videofile.close();
  playing = false;
}

// when an error happens during setup, give up and print a message
// to the serial monitor.
void stopWithErrorMessage(const char *str)
{
  while (1) {
    Serial.println(str);
    delay(1000);
  }
}

