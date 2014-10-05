/*  OctoWS2811 addaudio.c - Merge image and sound data to VIDEO.BIN
      for Teensy 3.1 running OctoWS2811 VideoSDcard.ino

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
*/

// Basis Usage:
//
// First, run movie2sdcard.pde with Processing....
//
// Merge "myvideo.bin" image data, created by movie2sdcard.pde,
// and "myvideo.raw" audio data, created by ffmpeg & sox,
// to create "VIDEO.BIN" which can be played by VideoSDcard.ino
//
// Run with these commands:
//
// ./ffmpeg -i myvideo.mov -vn -f wav myvideo.wav
// ./sox myvideo.wav -c 1 -b 16 -r 44117.647 myvideo.raw
// ./addaudio
//
// Then copy VIDEO.BIN to a Micro SD card and play it on your LEDs :-)
//
// Compile with:
//
// gcc -O2 -Wall -o addaudio addaudio.c

#define VIDEO_INFILE  "myvideo.bin"
#define AUDIO_INFILE  "myvideo.raw"
#define OUTFILE       "VIDEO.BIN"

#define SAMPLE_RATE   44117.64706

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>

void die(const char *format, ...) __attribute__ ((format (printf, 1, 2)));

int main()
{
	FILE *fpa, *fpv, *fout;
	unsigned char header[5], header2[5];
	unsigned char vbuf[65536], abuf[65536];
	unsigned int usec;
	uint64_t elapsed_usec=0LL;
	//double elapsed_usec=0.0;
	unsigned int target_samples, elapsed_samples=0;
	unsigned int samples;
	size_t n, size;

	fpv = fopen(VIDEO_INFILE, "rb");
	if (!fpv) die("unable to read %s\n", VIDEO_INFILE);
	fpa = fopen(AUDIO_INFILE, "rb");
	if (!fpa) die("unable to read %s\n", AUDIO_INFILE);
	fout = fopen(OUTFILE, "wb");
	if (!fout) die("unable to write %s\n", OUTFILE);

	while (!feof(fpv)) {
		n = fread(header, 1, 5, fpv);
		if (n != 5) {
			if (feof(fpv)) break;
			die("unable to read video header");
		}
		if (header[0] == '*') {
			size = (header[1] | (header[2] << 8)) * 3;
			usec = header[3] | (header[4] << 8);
			if (size > sizeof(vbuf)) die("vbuf not big enough");
			n = fread(vbuf, 1, size, fpv);
			if (n != size) die("unable to read video data");

			printf("v: %u %u", (int)size, usec);

			if (usec == 0xFFFF) die("opps");

			elapsed_usec += usec;
			target_samples = ((uint64_t)elapsed_usec *
				(uint64_t)(SAMPLE_RATE * 1000.0) / 1000000000LL);
			//target_samples = elapsed_usec / 1e6 * SAMPLE_RATE;
			target_samples += 2560; // try to always buffer 2.5K
			samples = (target_samples - elapsed_samples + 127) / 128 * 128;
			elapsed_samples += samples;
			printf(", s=%u, n=%u", target_samples, samples);

			n = fread(abuf, 1, samples*2, fpa);
			if (n < samples*2) {
				printf(", n(audio) = %d", (int)n);
				n &= ~1;
			}
			header2[0] = '%';
			header2[1] = samples & 255;
			header2[2] = samples >> 8;
			header2[3] = 0;
			header2[4] = 0;
			fwrite(header2, 1, 5, fout);
			fwrite(abuf, 2, samples, fout);
			fwrite(header, 1, 5, fout);
			fwrite(vbuf, 1, size, fout);

		} else if (header[0] == '%') {
			// skip audio, if input file has any
			size = (header[1] | (header[2] << 8)) * 3;
			n = fread(vbuf, 1, size, fpv);
			if (n != size) die("unable to read video data");
		} else {
			die("unknown header type");
		}
		printf("\n");
	}
	return 0;
}

void die(const char *format, ...)
{
        va_list args;
        va_start(args, format);
        fprintf(stderr, "addaudio: ");
        vfprintf(stderr, format, args);
        fprintf(stderr, "\n");
        exit(1);
}

