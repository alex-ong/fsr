#define USE_LEDS    0
#define NUM_LEDS    0
#define INVERSE_VCC 1 //remap 1024->0 to 0->1024, if this is set to 1
#define LED_PIN     15

void InitializeLEDs()
{
  //pass
}

//sensor mapping. required.
const int sensorMapping1[] = { A10, A3, A2, A8, A0, A1, A7, A6};
const int sensorMapping2[] = { 0, 0, 1, 1, 2, 2, 3, 3};
const int kNumSensors =  sizeof(sensorMapping1)/sizeof(int);
const int kNumSensorStates = sizeof(sensorMapping2) / sizeof(int);
