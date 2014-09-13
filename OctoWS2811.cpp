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

static const uint8_t ones = 0xFF;
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
	frequency = (params & WS2811_400kHz) ? 400000 : 800000;
	analogWriteResolution(8);
	analogWriteFrequency(3, frequency);
	analogWriteFrequency(4, frequency);
	analogWrite(3, WS2811_TIMING_T0H);
	analogWrite(4, WS2811_TIMING_T1H);

	// pin 16 triggers DMA(port B) on rising edge (configure for pin 3's waveform)
	CORE_PIN16_CONFIG = PORT_PCR_IRQC(1)|PORT_PCR_MUX(3);
	pinMode(3, INPUT_PULLUP); // pin 3 no longer needed

	// pin 15 triggers DMA(port C) on falling edge of low duty waveform
	// pin 15 and 16 must be connected by the user: 16 is output, 15 is input
	pinMode(15, INPUT);
	CORE_PIN15_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(1);

	// pin 4 triggers DMA(port A) on falling edge of high duty waveform
	CORE_PIN4_CONFIG = PORT_PCR_IRQC(2)|PORT_PCR_MUX(3);

	// DMA channel #1 sets WS2811 high at the beginning of each cycle
	dma1.TCD->SADDR = &ones;
	dma1.TCD->SOFF = 0;
	dma1.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma1.TCD->NBYTES_MLNO = 1;
	dma1.TCD->SLAST = 0;
	dma1.TCD->DADDR = &GPIOD_PSOR;
	dma1.TCD->DOFF = 0;
	dma1.TCD->CITER_ELINKNO = bufsize;
	dma1.TCD->DLASTSGA = 0;
	dma1.TCD->CSR = DMA_TCD_CSR_DREQ;
	dma1.TCD->BITER_ELINKNO = bufsize;

	// DMA channel #2 writes the pixel data at 20% of the cycle
	dma2.TCD->SADDR = frameBuffer;
	dma2.TCD->SOFF = 1;
	dma2.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma2.TCD->NBYTES_MLNO = 1;
	dma2.TCD->SLAST = -bufsize;
	dma2.TCD->DADDR = &GPIOD_PDOR;
	dma2.TCD->DOFF = 0;
	dma2.TCD->CITER_ELINKNO = bufsize;
	dma2.TCD->DLASTSGA = 0;
	dma2.TCD->CSR = DMA_TCD_CSR_DREQ;
	dma2.TCD->BITER_ELINKNO = bufsize;

	// DMA channel #3 clear all the pins low at 48% of the cycle
	dma3.TCD->SADDR = &ones;
	dma3.TCD->SOFF = 0;
	dma3.TCD->ATTR = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
	dma3.TCD->NBYTES_MLNO = 1;
	dma3.TCD->SLAST = 0;
	dma3.TCD->DADDR = &GPIOD_PCOR;
	dma3.TCD->DOFF = 0;
	dma3.TCD->CITER_ELINKNO = bufsize;
	dma3.TCD->DLASTSGA = 0;
	dma3.TCD->CSR = DMA_TCD_CSR_DREQ | DMA_TCD_CSR_INTMAJOR;
	dma3.TCD->BITER_ELINKNO = bufsize;

#ifdef __MK20DX256__
	MCM_CR = MCM_CR_SRAMLAP(1) | MCM_CR_SRAMUAP(0);
	AXBS_PRS0 = 0x1032;
#endif

	// route the edge detect interrupts to trigger the 3 channels
	dma1.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTB);
	dma2.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTC);
	dma3.triggerAtHardwareEvent(DMAMUX_SOURCE_PORTA);

	// enable a done interrupts when channel #3 completes
	dma3.attachInterrupt(isr);
	//pinMode(1, OUTPUT); // testing: oscilloscope trigger
}

void OctoWS2811::isr(void)
{
	dma3.clearInterrupt();
	update_completed_at = micros();
	update_in_progress = 0;
}

int OctoWS2811::busy(void)
{
	//if (DMA_ERQ & 0xE) return 1;
	if (update_in_progress) return 1;
	// busy for 50 us after the done interrupt, for WS2811 reset
	if (micros() - update_completed_at < 50) return 1;
	return 0;
}

void OctoWS2811::show(void)
{
	uint32_t cv, sc;

	// wait for any prior DMA operation
	while (update_in_progress) ; 
	// it's ok to copy the drawing buffer to the frame buffer
	// during the 50us WS2811 reset time
	if (drawBuffer != frameBuffer) {
		// TODO: this could be faster with DMA, especially if the
		// buffers are 32 bit aligned... but does it matter?
		memcpy(frameBuffer, drawBuffer, stripLen * 24);
	}
	// wait for WS2811 reset
	while (micros() - update_completed_at < 50) ;

	// ok to start, but we must be very careful to begin
	// without any prior 3 x 800kHz DMA requests pending
	sc = FTM1_SC;
	cv = FTM1_C1V;
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
	//digitalWriteFast(1, HIGH); // oscilloscope trigger
	PORTB_ISFR = (1<<0);    // clear any prior rising edge
	PORTC_ISFR = (1<<0);	// clear any prior low duty falling edge
	PORTA_ISFR = (1<<13);	// clear any prior high duty falling edge
	dma1.enable();
	dma2.enable();		// enable all 3 DMA channels
	dma3.enable();
	FTM1_SC = sc;		// restart FTM1 timer
	//digitalWriteFast(1, LOW);
	interrupts();
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


