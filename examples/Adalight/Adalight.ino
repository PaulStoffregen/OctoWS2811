/*  OctoWS2811 Adalight.ino - Receive Adalight and display on WS2811 LEDs
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


  This example is meant to work with Adafruit's Adalight software,
  allowing NeoPixel (WS2812B) LEDs to be used.

  https://learn.adafruit.com/adalight-diy-ambient-tv-lighting

  For good performance, you may need to use AmbiBox.  Running with
  Processing has been reported to give only 7 frames/sec.  Details here:
  https://forums.adafruit.com/viewtopic.php?f=8&t=68444&p=434808#p434808

  Options for connecting the LEDs is fewer than 8 sections:
  https://forum.pjrc.com/threads/53974-Ambilight-help


  Required Connections    http://www.pjrc.com/store/octo28_adaptor.html
  --------------------
    pin 2:  LED Strip #1    OctoWS2811 drives 8 LED Strips.
    pin 14: LED strip #2    All 8 are the same length.
    pin 7:  LED strip #3
    pin 8:  LED strip #4    A 100 ohm resistor should used
    pin 6:  LED strip #5    between each Teensy pin and the
    pin 20: LED strip #6    wire to the LED strip, to minimize
    pin 21: LED strip #7    high frequency ringining & noise.
    pin 5:  LED strip #8
    pin 15 & 16 - Connect together, but do not use
    pin 4 - Do not use
    pin 3 - Do not use as PWM.  Normal use is ok.
*/

#include <OctoWS2811.h>

const int ledsPerStrip = 60;

DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6];

const int config = WS2811_GRB | WS2811_800kHz;

OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);


void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2000);
  leds.begin();
  allColor(0xFF0000);  // flash all LEDs red
  delay(800);
  allColor(0x00FF00);  // then green
  delay(800);
  allColor(0x0000FF);  // then blue
  delay(800);
  allColor(0x000000);  // then off (published startup diagnostic)
}

void allColor(unsigned int c) {
  for (int i=0; i < ledsPerStrip*8; i++) {
    leds.setPixel(i, c);
  }
  leds.show();
}


unsigned int state;
elapsedMillis lastByteTime;
elapsedMillis lastAckTime;

unsigned int count=0;
unsigned int total=0;

void loop() {
  unsigned char buf[3];

  if (!Serial) {
    while (!Serial) /* wait */ ;
    delay(20);
    Serial.print("Ada\n");
    lastByteTime = 0;
    lastAckTime = 0;
    state = 0;
    count = 0;
  }

  if (state == 0) {
    if (Serial.available() == 0) goto wait;
    state = (Serial.read() == 'A') ? 1 : 0;

  } else if (state == 1) {
    if (Serial.available() == 0) goto wait;
    state = (Serial.read() == 'd') ? 2 : 0;

  } else if (state == 2) {
    if (Serial.available() == 0) goto wait;
    state = (Serial.read() == 'a') ? 3 : 0;

  } else if (state == 3) {
    if (Serial.available() < 3) goto wait;
    Serial.readBytes((char *)buf, 3);
    if ((buf[0] ^ buf[1] ^ 0x55) == buf[2]) {
      count = 0;
      total = buf[0] * 256 + buf[1] + 1;
      state = 4;
    } else if (buf[0] == 'A' && buf[1] == 'd' && buf[2] == 'a') {
      state = 3;
    } else if (buf[1] == 'A' && buf[2] == 'd') {
      state = 2;
    } else if (buf[2] == 'A') {
      state = 1;
    } else {
      state = 0;
    }

  } else if (state == 4) {
    if (Serial.available() < 3) goto wait;
    Serial.readBytes((char *)buf, 3);
    if (count < ledsPerStrip*8) {
      leds.setPixel(count, (buf[0] << 16) | (buf[1] << 8) | buf[2]);
    }
    count++;
    if (count >= total) {
      leds.show();
      state = 0;
    }

  } else {
wait:
    if (lastAckTime > 1000) {
      lastAckTime = 0;
      while (Serial.available()) Serial.read();
      Serial.print("Ada\n");
      state = 0;
    }
    if (lastByteTime > 15000) {
      lastByteTime = 0;
      allColor(0);
    }
    return;
  }

  lastAckTime = 0;
  lastByteTime = 0;
}

