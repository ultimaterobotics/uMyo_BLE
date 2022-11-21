/*
Obtaining muscle data from uMyo via BLE on ESP32 and showing 
activity level on a connected WS2812 LED strip
*/

#include <uMyo_BLE.h>
#include <FastLED.h>

#define NUM_LEDS 16
#define DATA_PIN 13

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  uMyo.begin();
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  for(int l = 0; l < NUM_LEDS; l++)
    leds[l] = CRGB(0, 0, 0);
  FastLED.show();
}

void loop() 
{
  int dev_count = uMyo.getDeviceCount(); //if more than one device is present, show all of them
  float muscle_level = 0;
  if(dev_count > 0) muscle_level = uMyo.getMuscleLevel(0); //take data from the 1st connected device if more than 1 is connected
  float max_level = 600; //defines sensitivity - the lower is maximum, the less effort is needed to fill all LEDs
  int active_leds = NUM_LEDS * muscle_level / max_level;
  int brightness = 150;
  for(int l = 0; l < NUM_LEDS; l++)
  {
    if(l < active_leds) leds[l] = CRGB(l * brightness / NUM_LEDS, 0, brightness/2); //blue -> purple red color scale
    else leds[l] = CRGB(0, 0, 0);
  }
  FastLED.show();
  delay(5);
}

