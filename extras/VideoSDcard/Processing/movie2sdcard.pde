/*  OctoWS2811 movie2sdcard.pde - Convert video for SD card playing, with
      Teensy 3.1 running OctoWS2811 VideoSDcard.ino

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

// To configure this program, edit the following:
//
//  1: Change myMovie to open a video file of your choice    ;-)
//     Also change the output file name.
//
//  2: Edit ledWidth, ledHeight, ledLayout for your LEDs
//
//  3: Edit framerate.  This configures the speed VideoSDcard
//     will play your video data.  It's also critical for merging
//     audio to be played by Teensy 3.1's digital to analog output.
//

import processing.video.*;
import processing.serial.*;
import java.io.*;

int ledWidth =  60;          // size of LED panel
int ledHeight = 32;
boolean ledLayout = true;    // layout of rows, true = even is left->right

double framerate = 23.98;    // You MUST set this to the movie's frame rate
                             // Processing does not seem to have a way to detect it.

Movie myMovie = new Movie(this, "/Users/paul/myvideo.mov");
FileOutputStream myFile;     // edit output filename below...

float gamma = 1.8;
PImage ledImage;
int[] gammatable = new int[256];
long elapsed_picoseconds=0L;
long elapsed_microseconds=0L;
long picoseconds_per_frame = (long)(1e12 / framerate + 0.5);
boolean fileopen=true;

void setup() {
  for (int i=0; i < 256; i++) {
    gammatable[i] = (int)(pow((float)i / 255.0, gamma) * 255.0 + 0.5);
  }
  try {
    myFile = new FileOutputStream("/Users/paul/myvideo.bin");
  } catch (Exception e) {
    exit();
  }
  ledImage = createImage(ledWidth, ledHeight, RGB);
  size(720, 560);  // create the window
  myMovie.play();  // start the movie :-)
}
 
// movieEvent runs for each new frame of movie data
void movieEvent(Movie m) {
  // read the movie's next frame
  m.read();

  elapsed_picoseconds += picoseconds_per_frame;
  int usec = (int)((elapsed_picoseconds / 1000000L) - elapsed_microseconds);
  elapsed_microseconds += (long)usec;
  println("usec = " + usec);
  
  
  // copy the movie's image to the LED image
  ledImage.copy(m, 0, 0, m.width, m.height, 0, 0, ledWidth, ledHeight);
  // convert the LED image to raw data
  byte[] ledData =  new byte[(ledWidth * ledHeight * 3) + 5];
  image2data(ledImage, ledData, ledLayout);
  ledData[0] = '*';  // first Teensy is the frame sync master
  
  ledData[1] = (byte)(ledWidth * ledHeight);
  ledData[2] = (byte)((ledWidth * ledHeight) >> 8);
  ledData[3] = (byte)(usec);   // request the frame sync pulse
  ledData[4] = (byte)(usec >> 8); // at 75% of the frame time
  // send the raw data to the LEDs  :-)
  //ledSerial[i].write(ledData); 
  try {
    myFile.write(ledData);
  } catch (Exception e) {    
    exit();
  }
}

// image2data converts an image to OctoWS2811's raw data format.
// The number of vertical pixels in the image must be a multiple
// of 8.  The data array must be the proper size for the image.
void image2data(PImage image, byte[] data, boolean layout) {
  int offset = 5;
  int x, y, xbegin, xend, xinc, mask;
  int linesPerPin = image.height / 8;
  int pixel[] = new int[8];
  
  for (y = 0; y < linesPerPin; y++) {
    if ((y & 1) == (layout ? 0 : 1)) {
      // even numbered rows are left to right
      xbegin = 0;
      xend = image.width;
      xinc = 1;
    } else {
      // odd numbered rows are right to left
      xbegin = image.width - 1;
      xend = -1;
      xinc = -1;
    }
    for (x = xbegin; x != xend; x += xinc) {
      for (int i=0; i < 8; i++) {
        // fetch 8 pixels from the image, 1 for each pin
        pixel[i] = image.pixels[x + (y + linesPerPin * i) * image.width];
        pixel[i] = colorWiring(pixel[i]);
      }
      // convert 8 pixels to 24 bytes
      for (mask = 0x800000; mask != 0; mask >>= 1) {
        byte b = 0;
        for (int i=0; i < 8; i++) {
          if ((pixel[i] & mask) != 0) b |= (1 << i);
        }
        data[offset++] = b;
      }
    }
  } 
}

// translate the 24 bit color from RGB to the actual
// order used by the LED wiring.  GRB is the most common.
int colorWiring(int c) {
  int red = (c & 0xFF0000) >> 16;
  int green = (c & 0x00FF00) >> 8;
  int blue = (c & 0x0000FF);
  red = gammatable[red];
  green = gammatable[green];
  blue = gammatable[blue];
  return (green << 16) | (red << 8) | (blue); // GRB - most common wiring
}

// draw runs every time the screen is redrawn - show the movie...
void draw() {
  if (myMovie.time() < myMovie.duration()) {
    image(myMovie, 0, 80);
    image(ledImage, 240 - ledWidth / 2, 10);
  } else {
    if (fileopen) {
      println("movie stop, closing output file");
      try {
        myFile.close();
      } catch (Exception e) {    
        exit();
      }
      fileopen = false;
    }
  }
}

// respond to mouse clicks as pause/play
boolean isPlaying = true;
void mousePressed() {
  if (isPlaying) {
    myMovie.pause();
    isPlaying = false;
  } else {
    myMovie.play();
    isPlaying = true;
  }
}

