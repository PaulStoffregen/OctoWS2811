/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC

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
*/

#ifndef OctoWS2811_h
#define OctoWS2811_h

#include <Arduino.h>

#ifdef __AVR__
#error "Sorry, OctoWS2811 only works on 32 bit Teensy boards.  AVR isn't supported."
#endif

#if TEENSYDUINO < 121
#error "Teensyduino version 1.21 or later is required to compile this library."
#endif

#include "DMAChannel.h"

#define WS2811_RGB	0	// The WS2811 datasheet documents this way
#define WS2811_RBG	1
#define WS2811_GRB	2	// Most LED strips are wired this way
#define WS2811_GBR	3
#define WS2811_BRG	4
#define WS2811_BGR	5
#define WS2811_RGBW	6
#define WS2811_RBGW	7
#define WS2811_GRBW	8
#define WS2811_GBRW	9
#define WS2811_BRGW	10
#define WS2811_BGRW	11
#define WS2811_WRGB	12
#define WS2811_WRBG	13
#define WS2811_WGRB	14
#define WS2811_WGBR	15
#define WS2811_WBRG	16
#define WS2811_WBGR	17
#define WS2811_RWGB	18
#define WS2811_RWBG	19
#define WS2811_GWRB	20
#define WS2811_GWBR	21
#define WS2811_BWRG	22
#define WS2811_BWGR	23
#define WS2811_RGWB	24
#define WS2811_RBWG	25
#define WS2811_GRWB	26
#define WS2811_GBWR	27
#define WS2811_BRWG	28
#define WS2811_BGWR	29

#define WS2811_800kHz 0x00	// Nearly all WS2811 are 800 kHz
#define WS2811_400kHz 0x40	// Adafruit's Flora Pixels
#define WS2813_800kHz 0x80	// WS2813 are close to 800 kHz but has 300 us frame set delay


class OctoWS2811 {
public:
#if defined(__IMXRT1062__)
	// Teensy 4.x can use any arbitrary group of pins!
	OctoWS2811(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config = WS2811_GRB, uint8_t numPins = 8, const uint8_t *pinList = defaultPinList);
	void begin(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config = WS2811_GRB, uint8_t numPins = 8, const uint8_t *pinList = defaultPinList);
	int numPixels(void);
#else
	// Teensy 3.x is fixed to 8 pins: 2, 14, 7, 8, 6, 20, 21, 5
	OctoWS2811(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config = WS2811_GRB);
	void begin(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config = WS2811_GRB);
	int numPixels(void) {
		return stripLen * 8;
	}
#endif
	void begin(void);

	void setPixel(uint32_t num, int color);
	void setPixel(uint32_t num, uint8_t red, uint8_t green, uint8_t blue) {
		setPixel(num, color(red, green, blue));
	}
	void setPixel(uint32_t num, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
		setPixel(num, Color(red, green, blue, white));
	}
	int getPixel(uint32_t num);

	void show(void);
	int busy(void);

	int color(uint8_t red, uint8_t green, uint8_t blue) {
		return (red << 16) | (green << 8) | blue;
	}

	// Functions for compatibility with Adafruit_NeoPixel
	void setPixelColor(uint16_t num, uint32_t color) {
		setPixel(num, color);
	}
	void setPixelColor(uint16_t num, uint8_t red, uint8_t green, uint8_t blue) {
		setPixel(num, red, green, blue);
	}
	void setPixelColor(uint16_t num, uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
		setPixel(num, red, green, blue, white);
	}
	//void setBrightness(uint8_t n) {
		//brightness = n;
	//}
	//uint8_t getBrightness() {
		//return brightness;
	//}
	uint32_t Color(uint8_t red, uint8_t green, uint8_t blue) {
		return (red << 16) | (green << 8) | blue;
	}
	uint32_t Color(uint8_t red, uint8_t green, uint8_t blue, uint8_t white) {
		return (white << 24) | (red << 16) | (green << 8) | blue;
	}

private:
	static uint16_t stripLen;
	//static uint8_t brightness;
	static void *frameBuffer;
	static void *drawBuffer;
	static uint8_t params;
	static DMAChannel dma1, dma2, dma3;
	static void isr(void);
	static uint8_t defaultPinList[8];
};

#endif
