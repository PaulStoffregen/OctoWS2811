// Glediator example with OctoWS2811, by mortonkopf
//
// https://forum.pjrc.com/threads/33012-Gladiator-with-OctoWS2811-working-example

// You can also use Jinx to record Glediator format data to a SD card.
// To play the data from your SD card, use this modified program:
// https://forum.pjrc.com/threads/46229&viewfull=1#post153927

#include <OctoWS2811.h>

const int ledsPerStrip = 34;
const int NUM_LEDS = 272;

DMAMEM int displayMemory[ledsPerStrip*6];
int drawingMemory[ledsPerStrip*6];

const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(ledsPerStrip, displayMemory, drawingMemory, config);

void setup() {
  leds.begin();
  leds.show();
}

int serialGlediator() {
  while (!Serial.available()) {}
  return Serial.read();
}

void loop() {
  byte r,g,b;
  int i;

  while (serialGlediator() != 1) {}

  for (i=0; i < NUM_LEDS; i++) {
    b = serialGlediator();
    r = serialGlediator();
    g = serialGlediator();

    leds.setPixel(i, Color(r,g,b));
  }
  leds.show();
}

/* Helper functions */
// Create a 24 bit color value from R,G,B
unsigned int Color(byte r, byte g, byte b)
{
  return (((unsigned int)b << 16) | ((unsigned int)r << 8) | (unsigned int)g);
}
