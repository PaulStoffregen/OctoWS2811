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

#include <string.h>
#include "OctoWS2811.h"


uint16_t OctoWS2811::stripLen;
void * OctoWS2811::frameBuffer;
void * OctoWS2811::drawBuffer;
uint8_t OctoWS2811::params;
DMAChannel OctoWS2811::dma1;
DMAChannel OctoWS2811::dma2;
DMAChannel OctoWS2811::dma3;

static uint8_t ones = 0xFF;
static volatile uint8_t update_in_progress = 0;
static uint32_t update_completed_at = 0;

OctoWS2811::OctoWS2811()
{
    // Unusable
}

void OctoWS2811::attach(uint32_t numPerStrip, void *frameBuf, void *drawBuf, uint8_t config)
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


void OctoWS2811::begin(void)
{
	uint32_t bufsize, frequency;
	bufsize = stripLen*24;

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
  switch (params & 0xF0) {
    case WS2811_400kHz: {
      frequency = 400000;
      frameSetDelay = 50;
    break; }
    case WS2811_800kHz: {
      frequency = 800000;
      frameSetDelay = 50;
    break; }
    case WS2813_800kHz: {
      frequency = 800000;
      frameSetDelay = 300;
    break; }
    default: {
      frequency = 800000;
      frameSetDelay = 50;
    break; }
  }
	analogWriteResolution(8);
	analogWriteFrequency(3, frequency);
	analogWriteFrequency(4, frequency);
	analogWrite(3, WS2811_TIMING_T0H);
	analogWrite(4, WS2811_TIMING_T1H);

#if defined(KINETISK)
	// pin 16 triggers DMA(port B) on rising edge (configure for pin 3's waveform)
	CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);
	pinMode(3, INPUT_PULLUP); // pin 3 no longer needed

	// pin 15 triggers DMA(port C) on falling edge of low duty waveform
	// pin 15 and 16 must be connected by the user: 16 is output, 15 is input
	pinMode(15, INPUT);
	CORE_PIN15_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(1);

	// pin 4 triggers DMA(port A) on falling edge of high duty waveform
	CORE_PIN4_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(3);

#elif defined(KINETISL)
	// on Teensy-LC, use timer DMA, not pin DMA
	//Serial1.println(FTM2_C0SC, HEX);
	//FTM2_C0SC = 0xA9;
	//FTM2_C0SC = 0xA9;
	//uint32_t t = FTM2_C0SC;
	//FTM2_C0SC = 0xA9;
	//Serial1.println(t, HEX);
	CORE_PIN3_CONFIG = 0;
	CORE_PIN4_CONFIG = 0;
	//FTM2_C0SC = 0;
	//FTM2_C1SC = 0;
	//while (FTM2_C0SC) ;
	//while (FTM2_C1SC) ;
	//FTM2_C0SC = 0x99;
	//FTM2_C1SC = 0x99;

	//MCM_PLACR |= MCM_PLACR_ARB;

#endif

	// DMA channel #1 sets WS2811 high at the beginning of each cycle
	dma1.source(ones);
	dma1.destination(GPIOD_PSOR);
	dma1.transferSize(1);
	dma1.transferCount(bufsize);
	dma1.disableOnCompletion();

	// DMA channel #2 writes the pixel data at 20% of the cycle
	dma2.sourceBuffer((uint8_t *)frameBuffer, bufsize);
	dma2.destination(GPIOD_PDOR);
	dma2.transferSize(1);
	dma2.transferCount(bufsize);
	dma2.disableOnCompletion();

	// DMA channel #3 clear all the pins low at 48% of the cycle
	dma3.source(ones);
	dma3.destination(GPIOD_PCOR);
	dma3.transferSize(1);
	dma3.transferCount(bufsize);
	dma3.disableOnCompletion();
	dma3.interruptAtCompletion();

#ifdef __MK20DX256__
	MCM_CR = MCM_CR_SRAMLAP(1) | MCM_CR_SRAMUAP(0);
	AXBS_PRS0 = 0x1032;
#endif

#if defined(KINETISK)
	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTC);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);
#elif defined(KINETISL)
	// route the timer interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_OV);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);
#endif

	// enable a done interrupts when channel #3 completes
	dma3.attachInterrupt(isr);
	//pinMode(9, OUTPUT); // testing: oscilloscope trigger
}

void OctoWS2811::isr(void)
{
	//Serial1.print(".");
	//Serial1.println(dma3.CFG->DCR, HEX);
	//Serial1.print(dma3.CFG->DSR_BCR > 24, HEX);
	dma3.clearInterrupt();
	//Serial1.print("*");
	update_completed_at = micros();
	update_in_progress = 0;
}

int OctoWS2811::busy(void)
{
	if (update_in_progress) return 1;
	// busy for 50 (or 300 for ws2813) us after the done interrupt, for WS2811 reset
	if (micros() - update_completed_at < frameSetDelay) return 1;
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
		memcpy(frameBuffer, drawBuffer, stripLen * 24);
	}
	// wait for WS2811 reset
	while (micros() - update_completed_at < frameSetDelay) ;

#if defined(KINETISK)
	// ok to start, but we must be very careful to begin
	// without any prior 3 x 800kHz DMA requests pending
	uint32_t sc = FTM1_SC;
	uint32_t cv = FTM1_C1V;
	noInterrupts();
	// CAUTION: this code is timing critical.  Any editing should be
	// tested by verifying the oscilloscope trigger pulse at the end
	// always occurs while both waveforms are still low.  Simply
	// counting CPU cycles does not take into account other complex
	// factors, like flash cache misses and bus arbitration from USB
	// or other DMA.  Testing should be done with the oscilloscope
	// display set at infinite persistence and a variety of other I/O
	// performed to create realistic bus usage.  Even then, you really
	// should not mess with this timing critical code!
	update_in_progress = 1;
	while (FTM1_CNT <= cv) ;
	while (FTM1_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM1_CNT < cv) ;
	FTM1_SC = sc & 0xE7;	// stop FTM1 timer (hopefully before it rolls over)
	//digitalWriteFast(9, HIGH); // oscilloscope trigger
	PORTB_ISFR = (1<<0);    // clear any prior rising edge
	PORTC_ISFR = (1<<0);	// clear any prior low duty falling edge
	PORTA_ISFR = (1<<13);	// clear any prior high duty falling edge
	dma1.enable();
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	FTM1_SC = sc;		// restart FTM1 timer
	//digitalWriteFast(9, LOW);
#elif defined(KINETISL)
	uint32_t sc = FTM2_SC;
	uint32_t cv = FTM2_C1V;
	noInterrupts();
	update_in_progress = 1;
	while (FTM2_CNT <= cv) ;
	while (FTM2_CNT > cv) ; // wait for beginning of an 800 kHz cycle
	while (FTM2_CNT < cv) ;
	FTM2_SC = 0;		// stop FTM2 timer (hopefully before it rolls over)
	//digitalWriteFast(9, HIGH); // oscilloscope trigger


	dma1.clearComplete();
	dma2.clearComplete();
	dma3.clearComplete();
	uint32_t bufsize = stripLen*24;
	dma1.transferCount(bufsize);
	dma2.transferCount(bufsize);
	dma3.transferCount(bufsize);
	dma2.sourceBuffer((uint8_t *)frameBuffer, bufsize);

	// clear any pending event flags
	FTM2_SC = 0x80;
	FTM2_C0SC = 0xA9;	// clear any previous pending DMA requests
	FTM2_C1SC = 0xA9;
	// clear any prior pending DMA requests
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_OV);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH0);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM2_CH1);
	//GPIOD_PTOR = 0xFF;
	//GPIOD_PTOR = 0xFF;
	dma1.enable();
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	FTM2_SC = 0x188;
	//digitalWriteFast(9, LOW);
#endif
	//Serial1.print("3");
	interrupts();
	//Serial1.print("4");
}

void OctoWS2811::setPixel(uint32_t num, int color)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;

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
	  default:
		break;
	}
	strip = num / stripLen;  // Cortex-M4 has 2 cycle unsigned divide :-)
	offset = num % stripLen;
	bit = (1<<strip);
	p = ((uint8_t *)drawBuffer) + offset * 24;
	for (mask = (1<<23) ; mask ; mask >>= 1) {
		if (color & mask) {
			*p++ |= bit;
		} else {
			*p++ &= ~bit;
		}
	}
}

int OctoWS2811::getPixel(uint32_t num)
{
	uint32_t strip, offset, mask;
	uint8_t bit, *p;
	int color=0;

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
	  default:
		break;
	}
	return color;
}
