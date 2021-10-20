// BasicTest example to demonstrate how to use FastLED with OctoWS2811

#include <OctoWS2811.h>
#define USE_OCTOWS2811
#include <FastLED.h>

#define NUM_LEDS  1920

CRGB leds[NUM_LEDS];

#define RED    0xFF0000
#define GREEN  0x00FF00
#define BLUE   0x0000FF
#define YELLOW 0xFFFF00
#define PINK   0xFF1088
#define ORANGE 0xE05800
#define WHITE  0xFFFFFF
  
void setup() {
  Serial.begin(9600);
  Serial.println("ColorWipe Using FastLED");
  LEDS.addLeds<OCTOWS2811,GRB>(leds,NUM_LEDS/8);
  LEDS.setBrightness(60);
}

void loop() {
  int microsec = 6000000 / NUM_LEDS;
  colorWipe(RED, microsec);
  colorWipe(GREEN, microsec);
  colorWipe(BLUE, microsec);
  colorWipe(YELLOW, microsec);
  colorWipe(PINK, microsec);
  colorWipe(ORANGE, microsec);
  colorWipe(WHITE, microsec);
}

void colorWipe(int color, int wait)
{
  for (int i=0; i < NUM_LEDS; i++) {
    leds[i] = color;
    FastLED.show();
    delayMicroseconds(wait);
  }
}
