#include <FastLED.h>
#define USE_LEDS    1
#define NUM_LEDS    28
#define INVERSE_VCC 0 //remap 1024->0 to 0->1024, if this is set to 1
#define LED_DATA_PIN     16
#define LED_CLOCK_PIN   15

CRGB leds[NUM_LEDS];
#define WHITE_LIMIT  30 
CRGB WHITE = CRGB(WHITE_LIMIT,WHITE_LIMIT,WHITE_LIMIT);
CRGB RED = CRGB(128,0,0);
CRGB BLUE = CRGB(0,0,128);

//LED settings.
CRGB defaultColors[4] = {BLUE, RED, BLUE, RED};
uint8_t ledOrder[4] = {3, 2, 0, 1};
uint8_t firstled[5] = {0, 7, 14, 21, NUM_LEDS}; 

void InitializeLEDs()
{
  FastLED.addLeds<SK9822, LED_DATE_PIN, LED_CLOCK_PIN, BGR>(leds, NUM_LEDS);
}

//sensor pins to sensor mapping
const int sensorMapping1[] = { A1, A0, A2, A3};
const int sensorMapping2[] = { 0, 1, 2, 3};
const int kNumSensors =  sizeof(sensorMapping1)/sizeof(int);
const int kNumSensorStates = sizeof(sensorMapping2) / sizeof(int);
