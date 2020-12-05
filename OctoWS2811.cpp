/*  OctoWS2811 - High Performance WS2811 LED Display Library
    http://www.pjrc.com/teensy/td_libs_OctoWS2811.html
    Copyright (c) 2013 Paul Stoffregen, PJRC.COM, LLC
    Some Teensy-LC support contributed by Mark Baysinger.
    https://forum.pjrc.com/threads/40863-Teensy-LC-port-of-OctoWS2811

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

#include <Arduino.h>
#include "OctoWS2811.h"

#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__) || defined(__MKL26Z64__)

uint16_t OctoWS2811::stripLen;
//uint8_t OctoWS2811::brightness = 255;
void * OctoWS2811::frameBuffer;
void * OctoWS2811::drawBuffer;
uint8_t OctoWS2811::params;
DMAChannel OctoWS2811::dma1;
DMAChannel OctoWS2811::dma2;
DMAChannel OctoWS2811::dma3;

static uint8_t ones = 0xFF;
static volatile uint8_t update_in_progress = 0;
static uint32_t update_completed_at = 0;

OctoWS2811::OctoWS2811(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config)
{
	stripLen = numPerStrip;
	frameBuffer = frameBuf;
	drawBuffer = drawBuf;
	params = config;
}

// Waveform timing: these set the high time for a 0 and 1 bit, as a fraction of
// the total 800 kHz or 400 kHz clock cycle.  The scale is 0 to 255.  The Worldsemi
// datasheet seems T1H should be 600 ns of a 1250 ns cycle, or 48%.  That may
// erroneous information?  Other sources reason the chip actually samples the
// line close to the center of each bit time, so T1H should be 80% if TOH is 20%.
// The chips appear to work based on a simple one-shot delay triggered by the
// rising edge.  At least 1 chip tested retransmits 0 as a 330 ns pulse (26%) and
// a 1 as a 660 ns pulse (53%).  Perhaps it's actually sampling near 500 ns?
// There doesn't seem to be any advantage to making T1H less, as long as there
// is sufficient low time before the end of the cycle, so the next rising edge
// can be detected.  T0H has been lengthened slightly, because the pulse can
// narrow if the DMA controller has extra latency during bus arbitration.  If you
// have an insight about tuning these parameters AND you have actually tested on
// real LED strips, please contact paul@pjrc.com.  Please do not email based only
// on reading the datasheets and purely theoretical analysis.
#define WS2811_TIMING_T0H  60
#define WS2811_TIMING_T1H  176

// Discussion about timing and flicker & color shift issues:
// http://forum.pjrc.com/threads/23877-WS2812B-compatible-with-OctoWS2811-library?p=38190&viewfull=1#post38190

void OctoWS2811::begin(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config)
{
	stripLen = numPerStrip;
	frameBuffer = frameBuf;
	drawBuffer = drawBuf;
	params = config;
	begin();
}

void OctoWS2811::begin(void)
{
	uint32_t bufsize, frequency;

	if ((params & 0x1F) < 6) {
		bufsize = stripLen * 24; // RGB formats
	} else {
		bufsize = stripLen * 32; // RGBW formats
	}

	// set up the buffers
	memset(frameBuffer, 0, bufsize);
	if (drawBuffer) {
		memset(drawBuffer, 0, bufsize);
	} else {
		drawBuffer = frameBuffer;
	}

	// configure the 8 output pins
	GPIOD_PCOR = 0xFF;
	pinMode(2, OUTPUT);	// strip #1
	pinMode(14, OUTPUT);	// strip #2
	pinMode(7, OUTPUT);	// strip #3
	pinMode(8, OUTPUT);	// strip #4
	pinMode(6, OUTPUT);	// strip #5
	pinMode(20, OUTPUT);	// strip #6
	pinMode(21, OUTPUT);	// strip #7
	pinMode(5, OUTPUT);	// strip #8

	// create the two waveforms for WS2811 low and high bits
	switch (params & 0xC0) {
	case WS2811_400kHz:
		frequency = 400000;
		break;
	case WS2811_800kHz:
		frequency = 800000;
		break;
	case WS2813_800kHz:
		frequency = 800000;
		break;
	default:
		frequency = 800000;
	}


#if defined(__MK20DX128__)
	FTM1_SC = 0;
	FTM1_CNT = 0;
	uint32_t mod = (F_BUS + frequency / 2) / frequency;
	FTM1_MOD = mod - 1;
	FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	FTM1_C0SC = 0x69;
	FTM1_C1SC = 0x69;
	FTM1_C0V = (mod * WS2811_TIMING_T0H) >> 8;
	FTM1_C1V = (mod * WS2811_TIMING_T1H) >> 8;
	// pin 16 triggers DMA(port B) on rising edge
	CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);
	//CORE_PIN4_CONFIG = PORT_PCR_MUX(3); // testing only

#elif defined(__MK20DX256__)
	FTM2_SC = 0;
	FTM2_CNT = 0;
	uint32_t mod = (F_BUS + frequency / 2) / frequency;
	FTM2_MOD = mod - 1;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	FTM2_C0SC = 0x69;
	FTM2_C1SC = 0x69;
	FTM2_C0V = (mod * WS2811_TIMING_T0H) >> 8;
	FTM2_C1V = (mod * WS2811_TIMING_T1H) >> 8;
	// pin 32 is FTM2_CH0, PTB18, triggers DMA(port B) on rising edge
	// pin 25 is FTM2_CH1, PTB19
	CORE_PIN32_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);
	//CORE_PIN25_CONFIG = PORT_PCR_MUX(3); // testing only

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	FTM2_SC = 0;
	FTM2_CNT = 0;
	uint32_t mod = (F_BUS + frequency / 2) / frequency;
	FTM2_MOD = mod - 1;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	FTM2_C0SC = 0x69;
	FTM2_C1SC = 0x69;
	FTM2_C0V = (mod * WS2811_TIMING_T0H) >> 8;
	FTM2_C1V = (mod * WS2811_TIMING_T1H) >> 8;
	// FTM2_CH0, PTA10 (not connected), triggers DMA(port A) on rising edge
	PORTA_PCR10 = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);

#elif defined(__MKL26Z64__)
	FTM2_SC = 0;
	FTM2_CNT = 0;
	uint32_t mod = F_CPU / frequency;
	FTM2_MOD = mod - 1;
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0);
	FTM2_C0SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB;
	FTM2_C1SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB;
	TPM2_C0V = mod - ((mod * WS2811_TIMING_T1H) >> 8);
	TPM2_C1V = mod - ((mod * WS2811_TIMING_T1H) >> 8) + ((mod * WS2811_TIMING_T0H) >> 8);

#endif

	// DMA channel #1 sets WS2811 high at the beginning of each cycle
	dma1.source(ones);
	dma1.destination(GPIOD_PSOR);
	dma1.transferSize(1);
	dma1.transferCount(bufsize);
	dma1.disableOnCompletion();

	// DMA channel #2 writes the pixel data at 23% of the cycle
	dma2.sourceBuffer((uint8_t *)frameBuffer, bufsize);
	dma2.destination(GPIOD_PDOR);
	dma2.transferSize(1);
	dma2.transferCount(bufsize);
	dma2.disableOnCompletion();

	// DMA channel #3 clear all the pins low at 69% of the cycle
	dma3.source(ones);
	dma3.destination(GPIOD_PCOR);
	dma3.transferSize(1);
	dma3.transferCount(bufsize);
	dma3.disableOnCompletion();
	dma3.interruptAtCompletion();

#if defined(__MK20DX128__)
	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM1_CH0);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM1_CH1);
	DMAPriorityOrder(dma3, dma2, dma1);
#elif defined(__MK20DX256__)
	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);
	DMAPriorityOrder(dma3, dma2, dma1);
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);
	DMAPriorityOrder(dma3, dma2, dma1);
#elif defined(__MKL26Z64__)
	// route the timer interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_TPM2_CH0);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_TPM2_CH1);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_OV);
#endif

	// enable a done interrupts when channel #3 completes
	dma3.attachInterrupt(isr);
	//pinMode(9, OUTPUT); // testing: oscilloscope trigger
}

void OctoWS2811::isr(void)
{
	//digitalWriteFast(9, HIGH);
	//Serial1.print(".");
	//Serial1.println(dma3.CFG->DCR, HEX);
	//Serial1.print(dma3.CFG->DSR_BCR > 24, HEX);
	dma3.clearInterrupt();
#if defined(__MKL26Z64__)
	GPIOD_PCOR = 0xFF;
#endif
	//Serial1.print("*");
	update_completed_at = micros();
	update_in_progress = 0;
	//digitalWriteFast(9, LOW);
}

int OctoWS2811::busy(void)
{
	if (update_in_progress) return 1;
	// busy for 50 (or 300 for ws2813) us after the done interrupt, for WS2811 reset
	if (micros() - update_completed_at < 300) return 1;
	return 0;
}

void OctoWS2811::show(void)
{
	// wait for any prior DMA operation
	//Serial1.print("1");
	while (update_in_progress) ;
	//Serial1.print("2");
	// it's ok to copy the drawing buffer to the frame buffer
	// during the 50us WS2811 reset time
	if (drawBuffer != frameBuffer) {
		// TODO: this could be faster with DMA, especially if the
		// buffers are 32 bit aligned... but does it matter?
		if ((params & 0x1F) < 6) {
			memcpy(frameBuffer, drawBuffer, stripLen * 24);
		} else {
			memcpy(frameBuffer, drawBuffer, stripLen * 32);
		}
	}
	// wait for WS2811 reset
	while (micros() - update_completed_at < 300) ;
	// ok to start, but we must be very careful to begin
	// without any prior 3 x 800kHz DMA requests pending

#if defined(__MK20DX128__)
	uint32_t cv = FTM1_C0V;
	noInterrupts();
	// CAUTION: this code is timing critical.
	while (FTM1_CNT <= cv) ;
	while (FTM1_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM1_CNT < cv) ;
	FTM1_SC = 0;            // stop FTM1 timer (hopefully before it rolls over)
	FTM1_CNT = 0;
	update_in_progress = 1;
	//digitalWriteFast(9, HIGH); // oscilloscope trigger
	PORTB_ISFR = (1<<0);    // clear any prior rising edge
	uint32_t tmp __attribute__((unused));
	FTM1_C0SC = 0x28;
	tmp = FTM1_C0SC;        // clear any prior timer DMA triggers
	FTM1_C0SC = 0x69;
	FTM1_C1SC = 0x28;
	tmp = FTM1_C1SC;
	FTM1_C1SC = 0x69;
	dma1.enable();
	dma2.enable();          // enable all 3 DMA channels
	dma3.enable();
	FTM1_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM1 timer
	//digitalWriteFast(9, LOW);

#elif defined(__MK20DX256__)
	FTM2_C0SC = 0x28;
	FTM2_C1SC = 0x28;
	uint32_t cv = FTM2_C0V;
	noInterrupts();
	// CAUTION: this code is timing critical.
	while (FTM2_CNT <= cv) ;
	while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM2_CNT < cv) ;
	FTM2_SC = 0;             // stop FTM2 timer (hopefully before it rolls over)
	FTM2_CNT = 0;
	update_in_progress = 1;
	//digitalWriteFast(9, HIGH); // oscilloscope trigger
	PORTB_ISFR = (1<<18);    // clear any prior rising edge
	uint32_t tmp __attribute__((unused));
	FTM2_C0SC = 0x28;
	tmp = FTM2_C0SC;         // clear any prior timer DMA triggers
	FTM2_C0SC = 0x69;
	FTM2_C1SC = 0x28;
	tmp = FTM2_C1SC;
	FTM2_C1SC = 0x69;
	dma1.enable();
	dma2.enable();           // enable all 3 DMA channels
	dma3.enable();
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM2 timer
	//digitalWriteFast(9, LOW);

#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
	FTM2_C0SC = 0x28;
	FTM2_C1SC = 0x28;
	uint32_t cv = FTM2_C1V;
	noInterrupts();
	// CAUTION: this code is timing critical.
	while (FTM2_CNT <= cv) ;
	while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM2_CNT < cv) ;
	FTM2_SC = 0;             // stop FTM2 timer (hopefully before it rolls over)
	FTM2_CNT = 0;
	update_in_progress = 1;
	//digitalWriteFast(9, HIGH); // oscilloscope trigger
	#if defined(__MK64FX512__)
	asm("nop");
	#endif
	PORTA_ISFR = (1<<10);    // clear any prior rising edge
	uint32_t tmp __attribute__((unused));
	FTM2_C0SC = 0x28;
	tmp = FTM2_C0SC;         // clear any prior timer DMA triggers
	FTM2_C0SC = 0x69;
	FTM2_C1SC = 0x28;
	tmp = FTM2_C1SC;
	FTM2_C1SC = 0x69;
	dma1.enable();
	dma2.enable();           // enable all 3 DMA channels
	dma3.enable();
	FTM2_SC = FTM_SC_CLKS(1) | FTM_SC_PS(0); // restart FTM2 timer
	//digitalWriteFast(9, LOW);

#elif defined(__MKL26Z64__)
	uint32_t sc __attribute__((unused)) = FTM2_SC;
	uint32_t cv = FTM2_C1V;
	noInterrupts();
	while (FTM2_CNT <= cv) ;
	while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM2_CNT < cv) ;
	FTM2_SC = 0;		// stop FTM2 timer (hopefully before it rolls over)
	update_in_progress = 1;
	//digitalWriteFast(9, HIGH); // oscilloscope trigger
	dma1.clearComplete();
	dma2.clearComplete();
	dma3.clearComplete();
	const uint32_t bufsize = stripLen * (((params & 0x1F) < 6) ? 24 : 32);
	dma1.transferCount(bufsize);
	dma2.transferCount(bufsize);
	dma3.transferCount(bufsize);
	dma2.sourceBuffer((uint8_t *)frameBuffer, bufsize);
	// clear any pending event flags
	FTM2_SC = FTM_SC_TOF;
	FTM2_C0SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_DMA;
	FTM2_C1SC = FTM_CSC_CHF | FTM_CSC_MSB | FTM_CSC_ELSB | FTM_CSC_DMA;
	// clear any prior pending DMA requests
	dma1.enable();
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	FTM2_CNT = 0; // writing any value resets counter
	FTM2_SC = FTM_SC_DMA | FTM_SC_CLKS(1) | FTM_SC_PS(0);
	//digitalWriteFast(9, LOW);
#endif
	//Serial1.print("3");
	interrupts();
	//Serial1.print("4");
}

void OctoWS2811::setPixel(uint32_t num, int color)
{
	//Serial.printf("setPixel %u to color %08X\n", num, color);
	if ((params & 0x1F) < 6) {
		switch (params & 7) {
		  case WS2811_RBG:
			color = (color&0xFF0000) | ((color<<8)&0x00FF00) | ((color>>8)&0x0000FF);
			break;
		  case WS2811_GRB:
			color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);
			break;
		  case WS2811_GBR:
			color = ((color<<16)&0xFF0000) | ((color>>8)&0x00FFFF);
			break;
		  case WS2811_BRG:
			color = ((color<<8)&0xFFFF00) | ((color>>16)&0x0000FF);
			break;
		  case WS2811_BGR:
			color = ((color<<16)&0xFF0000) | (color&0x00FF00) | ((color>>16)&0x0000FF);
			break;
		  default:
			break;
		}
		uint32_t strip = num / stripLen;  // Cortex-M4 has 2 cycle unsigned divide :-)
		uint32_t offset = num % stripLen;

		uint32_t *p = ((uint32_t *) drawBuffer) + offset * 6;

		uint32_t mask32 = (0x01010101) << strip;

		// Set bytes 0-3
		*p &= ~mask32;
		*p |= (((color & 0x800000) >> 23) | ((color & 0x400000) >> 14) | ((color & 0x200000) >> 5) | ((color & 0x100000) << 4)) << strip;

		// Set bytes 4-7
		*++p &= ~mask32;
		*p |= (((color & 0x80000) >> 19) | ((color & 0x40000) >> 10) | ((color & 0x20000) >> 1) | ((color & 0x10000) << 8)) << strip;

		// Set bytes 8-11
		*++p &= ~mask32;
		*p |= (((color & 0x8000) >> 15) | ((color & 0x4000) >> 6) | ((color & 0x2000) << 3) | ((color & 0x1000) << 12)) << strip;

		// Set bytes 12-15
		*++p &= ~mask32;
		*p |= (((color & 0x800) >> 11) | ((color & 0x400) >> 2) | ((color & 0x200) << 7) | ((color & 0x100) << 16)) << strip;

		// Set bytes 16-19
		*++p &= ~mask32;
		*p |= (((color & 0x80) >> 7) | ((color & 0x40) << 2) | ((color & 0x20) << 11) | ((color & 0x10) << 20)) << strip;

		// Set bytes 20-23
		*++p &= ~mask32;
		*p |= (((color & 0x8) >> 3) | ((color & 0x4) << 6) | ((color & 0x2) << 15) | ((color & 0x1) << 24)) << strip;
	} else {
		uint8_t b = color;
		uint8_t g = color >> 8;
		uint8_t r = color >> 16;
		uint8_t w = color >> 24;
		uint32_t c = 0;
		switch (params & 0x1F) {
			case WS2811_RGBW: c = (r << 24) | (g << 16) | (b << 8) | w; break;
			case WS2811_RBGW: c = (r << 24) | (b << 16) | (g << 8) | w; break;
			case WS2811_GRBW: c = (g << 24) | (r << 16) | (b << 8) | w; break;
			case WS2811_GBRW: c = (g << 24) | (b << 16) | (r << 8) | w; break;
			case WS2811_BRGW: c = (b << 24) | (r << 16) | (g << 8) | w; break;
			case WS2811_BGRW: c = (b << 24) | (b << 16) | (r << 8) | w; break;
			case WS2811_WRGB: c = (w << 24) | (r << 16) | (g << 8) | b; break;
			case WS2811_WRBG: c = (w << 24) | (r << 16) | (b << 8) | g; break;
			case WS2811_WGRB: c = (w << 24) | (g << 16) | (r << 8) | b; break;
			case WS2811_WGBR: c = (w << 24) | (g << 16) | (b << 8) | r; break;
			case WS2811_WBRG: c = (w << 24) | (b << 16) | (r << 8) | g; break;
			case WS2811_WBGR: c = (w << 24) | (b << 16) | (g << 8) | r; break;
			case WS2811_RWGB: c = (r << 24) | (w << 16) | (g << 8) | b; break;
			case WS2811_RWBG: c = (r << 24) | (w << 16) | (b << 8) | g; break;
			case WS2811_GWRB: c = (g << 24) | (w << 16) | (r << 8) | b; break;
			case WS2811_GWBR: c = (g << 24) | (w << 16) | (b << 8) | r; break;
			case WS2811_BWRG: c = (b << 24) | (w << 16) | (r << 8) | g; break;
			case WS2811_BWGR: c = (b << 24) | (w << 16) | (g << 8) | r; break;
			case WS2811_RGWB: c = (r << 24) | (g << 16) | (w << 8) | b; break;
			case WS2811_RBWG: c = (r << 24) | (b << 16) | (w << 8) | g; break;
			case WS2811_GRWB: c = (g << 24) | (r << 16) | (w << 8) | b; break;
			case WS2811_GBWR: c = (g << 24) | (b << 16) | (w << 8) | r; break;
			case WS2811_BRWG: c = (b << 24) | (r << 16) | (w << 8) | g; break;
			case WS2811_BGWR: c = (b << 24) | (g << 16) | (w << 8) | r; break;
		}
		uint32_t strip = num / stripLen;
		uint32_t offset = num % stripLen;

		uint32_t *p = ((uint32_t *) drawBuffer) + offset * 8;
		uint32_t mask32 = (0x01010101) << strip;

		// Set bytes 0-3
		*p &= ~mask32;
		*p |= (((c & 0x80000000) >> 31) | ((c & 0x40000000) >> 22) | ((c & 0x20000000) >> 13) | ((c & 0x10000000) >> 4)) << strip;

		// Set bytes 4-7
		*++p &= ~mask32;
		*p |= (((c & 0x8000000) >> 27) | ((c & 0x4000000) >> 18) | ((c & 0x2000000) >> 9) | ((c & 0x1000000) << 0)) << strip;

		// Set bytes 8-11
		*++p &= ~mask32;
		*p |= (((c & 0x800000) >> 23) | ((c & 0x400000) >> 14) | ((c & 0x200000) >> 5) | ((c & 0x100000) << 4)) << strip;

		// Set bytes 12-15
		*++p &= ~mask32;
		*p |= (((c & 0x80000) >> 19) | ((c & 0x40000) >> 10) | ((c & 0x20000) >> 1) | ((c & 0x10000) << 8)) << strip;

		// Set bytes 16-19
		*++p &= ~mask32;
		*p |= (((c & 0x8000) >> 15) | ((c & 0x4000) >> 6) | ((c & 0x2000) << 3) | ((c & 0x1000) << 12)) << strip;

		// Set bytes 20-23
		*++p &= ~mask32;
		*p |= (((c & 0x800) >> 11) | ((c & 0x400) >> 2) | ((c & 0x200) << 7) | ((c & 0x100) << 16)) << strip;

		// Set bytes 24-27
		*++p &= ~mask32;
		*p |= (((c & 0x80) >> 7) | ((c & 0x40) << 2) | ((c & 0x20) << 11) | ((c & 0x10) << 20)) << strip;

		// Set bytes 28-31
		*++p &= ~mask32;
		*p |= (((c & 0x8) >> 3) | ((c & 0x4) << 6) | ((c & 0x2) << 15) | ((c & 0x1) << 24)) << strip;
	}
}

int OctoWS2811::getPixel(uint32_t num)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	int color=0;

	if ((params & 0x1F) >= 6) return 0; // TODO: implement getPixel() for RGBW modes
	strip = num / stripLen;
	offset = num % stripLen;
	bit = (1<<strip);
	p = ((uint8_t *)drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {
		if (*p++ & bit) color |= mask;
	}
	switch (params & 7) {
	  case WS2811_RBG:
		color = (color&0xFF0000) | ((color<<8)&0x00FF00) | ((color>>8)&0x0000FF);
		break;
	  case WS2811_GRB:
		color = ((color<<8)&0xFF0000) | ((color>>8)&0x00FF00) | (color&0x0000FF);
		break;
	  case WS2811_GBR:
		color = ((color<<8)&0xFFFF00) | ((color>>16)&0x0000FF);
		break;
	  case WS2811_BRG:
		color = ((color<<16)&0xFF0000) | ((color>>8)&0x00FFFF);
		break;
	  case WS2811_BGR:
		color = ((color<<16)&0xFF0000) | (color&0x00FF00) | ((color>>16)&0x0000FF);
		break;
	  default:
		break;
	}
	return color;
}

#endif // supported boards
