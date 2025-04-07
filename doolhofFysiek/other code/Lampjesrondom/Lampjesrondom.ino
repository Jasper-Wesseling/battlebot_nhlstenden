#include <Adafruit_NeoPixel.h>

#define NeoLED 8          // The single data pin for all NeoPixels
#define NUM_PIXELS 4      // Total number of NeoPixels (e.g., 4 LEDs)
#define DELAY_TIME 500    // Delay time between LED blinks

Adafruit_NeoPixel strip(NUM_PIXELS, NeoLED, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.clear();                  // Turn off all LEDs
    strip.setPixelColor(i, strip.Color(255, 165, 0)); // Orange color (R=255, G=165, B=0)
    strip.show();                    // Display the color
    delay(DELAY_TIME);                // Wait before switching to next LED
  }
}


// green color (128, 0, 0)

// red color (0, 128, 0)

// orange color (255, 165, 0)