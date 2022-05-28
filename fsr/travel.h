#include <FastLED.h>
#define USE_LEDS    1
#define NUM_LEDS    23
#define INVERSE_VCC 0 //remap 1024->0 to 0->1024, if this is set to 1
#define LED_PIN     15

CRGB leds[NUM_LEDS];
#define WHITE_LIMIT  64
CRGB WHITE = CRGB(WHITE_LIMIT,WHITE_LIMIT,WHITE_LIMIT);
CRGB RED = CRGB(255,0,0);
CRGB BLUE = CRGB(0,0,255);

//LED settings.
CRGB defaultColors[4] = {BLUE, RED, BLUE, RED};
uint8_t ledOrder[4] = {3, 2, 0, 1};
uint8_t firstled[5] = {0, 6, 11, 17, NUM_LEDS}; 

void InitializeLEDs()
{
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

const int sensorMapping1[] = { A1, A0, A2, A3};
const int sensorMapping2[] = { 0, 1, 2, 3};
const int kNumSensors =  sizeof(sensorMapping1)/sizeof(int);
const int kNumSensorStates = sizeof(sensorMapping2) / sizeof(int);
  
