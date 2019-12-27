#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN  D6

#define LED_SKIP 2
#define NUM_LEDS 4


void showStrip();
void setPixel(int Pixel, byte red, byte green, byte blue);
void setAll(byte red, byte green, byte blue);


Adafruit_NeoPixel strip(NUM_LEDS * LED_SKIP, LED_PIN, NEO_GRB + NEO_KHZ800);


void setup() { 
  strip.begin();
  strip.show();  // put your setup code here, to run once:


  setAll(100, 100, 100);
  setPixel(1, 100, 0, 0);
  showStrip();
}

void loop() {
  // put your main code here, to run repeatedly:
}


// Apply LED color changes
void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

// Set a LED color (not yet visible)
void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel * LED_SKIP, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

// Set all LEDs to a given color and apply it (visible)
void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}
