#include "FastADC.h"

#include <Joystick.h>
#include <inttypes.h>
#include <EEPROMex.h>


#if !defined(__AVR_ATmega32U4__) && !defined(__AVR_ATmega328P__) && \
    !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #define CAN_AVERAGE //not avr
#endif


#define TRAVEL 0
#define FOLDING 1
#define JUNACIOUS 2

//choose pad here
#define CHOSEN_PAD (JUNACIOUS)
#if CHOSEN_PAD == TRAVEL
  #include "travel.h"
#elif CHOSEN_PAD == FOLDING
  #include "folding.h"
#elif CHOSEN_PAD == JUNACIOUS
  #include "junacious.h"
#endif

#define FASTADC 1
#if USE_LEDS
  #include <FastLED.h>
#endif

bool needLEDUpdate = false;
bool muteLEDs = false;
void UpdateLEDColor(uint8_t button_num, bool pressed)
{  
  #if USE_LEDS    
  button_num = ledOrder[button_num-1]; //remap to clockwise around pad.
  CRGB defaultColor = defaultColors[button_num];
  CRGB color = pressed ? WHITE : defaultColor;
  color = muteLEDs ? CRGB(0,0,0): color;
  int start_index = firstled[button_num];
  int end_index = firstled[button_num+1];
  for (int i = start_index; i < end_index; i++)
  {
    leds[i] = color;
  }
  #endif
}

Joystick_ Joystick; //create the joystick
// Use the Joystick library for Teensy
void ButtonStart() {
  // Use Joystick.begin() for everything that's not Teensy 2.0.
  Joystick.begin(false);
}

void ButtonPress(uint8_t button_num) {
  Joystick.pressButton(button_num);
  #if USE_LEDS
    UpdateLEDColor(button_num, true);
    needLEDUpdate=true;
  #endif
}
void ButtonRelease(uint8_t button_num) {
  Joystick.releaseButton(button_num);
  #if USE_LEDS
    UpdateLEDColor(button_num, false);
    needLEDUpdate=true;
  #endif
}



// Default threshold value for each of the sensors.
const int16_t kDefaultThreshold = 1000;
// Baud rate used for Serial communication. Technically ignored by Teensys.
const long kBaudRate = 115200;


// Button numbers should start with 1 (Button0 is not a valid Joystick input).
// Automatically incremented when creating a new SensorState.
uint8_t curButtonNum = 1;


/*===========================================================================*/

// The class that actually evaluates a sensor and actually triggers the button
// press or release event. If there are multiple sensors added to a
// SensorState, they will all be evaluated first before triggering the event.
class SensorState {
 public:
  SensorState(): num_sensors_(0), kButtonNum(curButtonNum++) 
  {
    for (size_t i = 0; i < kMaxSharedSensors; ++i) {
      sensor_ids_[i] = 0;
      individual_states_[i] = SensorState::OFF;
    }    
  }

  // Adds a new sensor to share this state with. If we try adding a sensor that
  // we don't have space for, it's essentially dropped.
  void AddSensor(uint8_t sensor_id) {
    if (num_sensors_ < kMaxSharedSensors) {
      sensor_ids_[num_sensors_++] = sensor_id;
    }
  }

  // Evaluates a single sensor as part of the shared state.
  void EvaluateSensor(uint8_t sensor_id,
                      int16_t cur_value,
                      int16_t user_threshold) {
    size_t sensor_index = GetIndexForSensor(sensor_id);

    // The sensor we're evaluating is not part of this shared state.
    // This should not happen.
    if (sensor_index == SIZE_MAX) {
      return;
    }
    
    // If we're above the threshold, turn the individual sensor on.
    if (cur_value >= user_threshold + kPaddingWidth) {
      individual_states_[sensor_index] = SensorState::ON;
    }

    // If we're below the threshold, turn the individual sensor off.
    if (cur_value < user_threshold - kPaddingWidth) {
      individual_states_[sensor_index] = SensorState::OFF;
    }
    
    // If we evaluated all the sensors this state applies to, only then
    // should we determine if we want to send a press/release event.
    bool all_evaluated = (sensor_index == num_sensors_ - 1);

    if (all_evaluated) {
      switch (combined_state_) {
        case SensorState::OFF:
          {
            // If ANY of the sensors triggered, then we trigger a button press.
            bool turn_on = false;
            for (size_t i = 0; i < num_sensors_; ++i) {
              if (individual_states_[i] == SensorState::ON) {
                turn_on = true;
                break;
              }
            }
            if (turn_on) {
              ButtonPress(kButtonNum);
              combined_state_ = SensorState::ON;              
            }
          }
          break;
        case SensorState::ON:
          {
            // ALL of the sensors must be off to trigger a release.
            // i.e. If any of them are ON we do not release.
            bool turn_off = true;
            for (size_t i = 0; i < num_sensors_; ++i) {
              if (individual_states_[i] == SensorState::ON) {
                turn_off = false;
              }
            }
            if (turn_off) {
              ButtonRelease(kButtonNum);
              combined_state_ = SensorState::OFF;
            }
          }
          break;
      }
    }
  }

  // Given a sensor_id, returns the index in the sensor_ids_ array.
  // Returns SIZE_MAX if not found.
  size_t GetIndexForSensor(uint8_t sensor_id) {
    for (size_t i = 0; i < num_sensors_; ++i) {
      if (sensor_ids_[i] == sensor_id) {
        return i;
      }
    }
    return SIZE_MAX;
  }

 private:
  static const size_t kMaxSharedSensors = 2;
  // The collection of sensors shared with this state.
  uint8_t sensor_ids_[kMaxSharedSensors];
  // The number of sensors this state combines with.
  size_t num_sensors_;

  // Used to determine the state of each individual sensor, as well as
  // the aggregated state.
  enum State { OFF, ON };
  // The evaluated state for each individual sensor.
  State individual_states_[kMaxSharedSensors];
  // The aggregated state.
  State combined_state_ = SensorState::OFF;

  // One-tailed width size to create a window around user_threshold to
  // mitigate fluctuations by noise. 
  // TODO(teejusb): Make this a user controllable variable.
  const int16_t kPaddingWidth = 1;

  
  // The button number this state corresponds to.
  const uint8_t kButtonNum;
};

/*===========================================================================*/

// Class containing all relevant information per sensor.
class Sensor {
 public:
  Sensor(uint8_t pin_value, SensorState* sensor_state = nullptr)
      : initialized_(false), pin_value_(pin_value),
        user_threshold_(kDefaultThreshold),
        #if defined(CAN_AVERAGE)
          moving_average_(kWindowSize),
        #endif
        offset_(0), sensor_state_(sensor_state),
        should_delete_state_(false) {}
  
  ~Sensor() {
    if (should_delete_state_) {
      delete sensor_state_;
    }
  }

  void Init(uint8_t sensor_id) {
    // Sensor should only be initialized once.
    if (initialized_) {
      return;
    }
    // Sensor IDs should be 1-indexed thus they must be non-zero.
    if (sensor_id == 0) {
      return;
    }

    // There is no state for this sensor, create one.
    if (sensor_state_ == nullptr) {
      sensor_state_ = new SensorState();
      // If this sensor created the state, then it's in charge of deleting it.
      should_delete_state_ = true;
    }

    // If this sensor hasn't been added to the state, then try adding it.
    if (sensor_state_->GetIndexForSensor(sensor_id) == SIZE_MAX) {
      sensor_state_->AddSensor(sensor_id);
    }
    sensor_id_ = sensor_id;
    initialized_ = true;
  }

  // Fetches the sensor value and maybe triggers the button press/release.
  void EvaluateSensor(bool willSend) {
    if (!initialized_) {
      return;
    }
    // If this sensor was never added to the state, then return early.
    if (sensor_state_->GetIndexForSensor(sensor_id_) == SIZE_MAX) {
      return;
    }

    int16_t sensor_value = analogRead(pin_value_);
    #if INVERSE_VCC
      sensor_value = 1024 - sensor_value;
    #endif
    
    #if defined(CAN_AVERAGE)
      // Fetch the updated Weighted Moving Average.
      cur_value_ = moving_average_.GetAverage(sensor_value) - offset_;
    #else
      // Don't use averaging for Arduino Leonardo, Uno, Mega1280, and Mega2560
      // since averaging seems to be broken with it. This should also include
      // the Teensy 2.0 as it's the same board as the Leonardo.
      // TODO(teejusb): Figure out why and fix. Maybe due to different integer
      // widths?
      cur_value_ = sensor_value - offset_;
    #endif

    if (willSend) {
      sensor_state_->EvaluateSensor(
        sensor_id_, cur_value_, user_threshold_);
    }
  }

  void UpdateThreshold(int16_t new_threshold) {
    user_threshold_ = new_threshold;
  }

  int16_t UpdateOffset() {
    // Update the offset with the last read value. UpdateOffset should be
    // called with no applied pressure on the panels so that it will be
    // calibrated correctly.
    offset_ = cur_value_;
    return offset_;
  }

  int16_t GetCurValue() {
    return cur_value_;
  }

  int16_t GetThreshold() {
    return user_threshold_;
  }

 private:
  // Ensures that Init() has been called at exactly once on this Sensor.
  bool initialized_;
  // The pin on the Teensy/Arduino corresponding to this sensor.
  uint8_t pin_value_;

  // The user defined threshold value to activate/deactivate this sensor at.
  int16_t user_threshold_;
  
  #if defined(CAN_AVERAGE)
  // The smoothed moving average calculated to reduce some of the noise. 
  HullMovingAverage moving_average_;
  #endif

  // The latest value obtained for this sensor.
  int16_t cur_value_;
  // How much to shift the value read by during each read.
  int16_t offset_;

  // Since many sensors may refer to the same input this may be shared among
  // other sensors.
  SensorState* sensor_state_;
  // Used to indicate if the state is owned by this instance, or if it was
  // passed in from outside
  bool should_delete_state_;

  // A unique number corresponding to this sensor. Set during Init().
  uint8_t sensor_id_;
};

/*===========================================================================*/

// Defines the sensor collections and sets the pins for them appropriately.

//Declare array of sensorstates and raw sensors
SensorState *kSensorStates[kNumSensorStates];
Sensor *kSensors[kNumSensors];
void InitSensors()
{
    //create annonymous sensor states
  for (int i = 0; i < kNumSensorStates; i++)
  {
    kSensorStates[i] = new SensorState();
  }
  
  //map sensors to sensor states
  for (int i = 0; i < kNumSensors; i++)
  {
    kSensors[i] = new Sensor(sensorMapping1[i], kSensorStates[sensorMapping2[i]]);
  }
}


 

/*=====================================================================
 * EEPROM code
 *====================================================================*/
bool needWriteEEPROM = false;
long lastWriteEEPROM = millis();

inline bool ShouldWriteEEPROM()
{
  return needWriteEEPROM && (millis() - lastWriteEEPROM > 5000);
}

void WriteIntsToEEPROM(int offset)
{
    for (uint8_t i = 0; i < kNumSensors; i++) {
        EEPROM.updateInt(offset + i*sizeof(int), kSensors[i]->GetThreshold());
    }
    needWriteEEPROM = false;
    lastWriteEEPROM = millis();
}

void ReadIntsFromEEPROM(int offset)
{
    for (uint8_t i = 0; i < kNumSensors; i++) {
        uint16_t value = EEPROM.readInt(offset + i*sizeof(int));
        if (value != 0xFFFF) // default value
        {
            kSensors[i]->UpdateThreshold(value);
        }
    }    
}





/*===========================================================================*/

class SerialProcessor {
 public:
   void Init(long baud_rate) {
    Serial.begin(baud_rate);
  }

  void CheckAndMaybeProcessData() {
    while (Serial.available() > 0) {
      size_t bytes_read = Serial.readBytesUntil(
          '\n', buffer_, kBufferSize - 1);
      buffer_[bytes_read] = '\0';

      if (bytes_read == 0) { return; }
 
      switch(buffer_[0]) {
        case 'o':
        case 'O':
          UpdateOffsets();
          break;
        case 'v':
        case 'V':
          PrintValues();
          break;
        case 't':
        case 'T':
          PrintThresholds();
          break;
        case 'l':
        case 'L':
          SetLEDs(bytes_read);
          break;
        default:
          UpdateAndPrintThreshold(bytes_read);
          break;
      }
    }  
  }

  // L -- toggle leds
  // L1 -- turn leds on
  // L0 -- turn leds off
  void SetLEDs(size_t bytes_read)
  {
    if (bytes_read == 1)
    {
      muteLEDs = !muteLEDs;
    } 
    else
    {
      muteLEDs = buffer_[1] - '0' == 0;
    }
    #if USE_LEDS
    needLEDUpdate = true;
    
    for (int i = 1; i < 5; i++)
    {
      UpdateLEDColor(i, false);
    }
    #endif
  }
  
  void ToggleLEDs()
  {
    SetLEDs(!muteLEDs);    
  }
  
  void UpdateAndPrintThreshold(size_t bytes_read) {
    // Need to specify:
    // Sensor number + Threshold value.
    // {0, 1, 2, 3} + "0"-"1023"
    // e.g. 3180 (fourth FSR, change threshold to 180)
    
    if (bytes_read < 2 || bytes_read > 5) { return; }

    size_t sensor_index = buffer_[0] - '0';
    if (sensor_index >= kNumSensors) { return; }

    kSensors[sensor_index]->UpdateThreshold(
        strtoul(buffer_ + 1, nullptr, 10)
    );
    needWriteEEPROM = true;
    PrintThresholds();
  }

  void UpdateOffsets() {
    for (size_t i = 0; i < kNumSensors; ++i) {
      kSensors[i]->UpdateOffset();
    }
  }

  void PrintValues() {
    Serial.print("v");
    for (size_t i = 0; i < kNumSensors; ++i) {
      Serial.print(" ");
      Serial.print(kSensors[i]->GetCurValue());
    }
    Serial.print("\n");
  }

  void PrintThresholds() {
    Serial.print("t");
    for (size_t i = 0; i < kNumSensors; ++i) {
      Serial.print(" ");
      Serial.print(kSensors[i]->GetThreshold());
    }
    Serial.print("\n");
  }

 private:
   static const size_t kBufferSize = 64;
   char buffer_[kBufferSize];
};



/*===========================================================================*/

SerialProcessor serialProcessor;
// Timestamps are always "unsigned long" regardless of board type So don't need
// to explicitly worry about the widths.
unsigned long lastSend = 0;
// loopTime is used to estimate how long it takes to run one iteration of
// loop().
long loopTime = -1;

void setup() {

  InitializeLEDs();
  InitSensors();
    
  for (int i = 1; i < 5; i++)
  {
    UpdateLEDColor(i, false);
  }
  
  #if USE_LEDS
    FastLED.show();
  #endif
  
  serialProcessor.Init(kBaudRate);
  ButtonStart();
  for (size_t i = 0; i < kNumSensors; ++i) {
    // Button numbers should start with 1.
    kSensors[i]->Init(i + 1);
  }
  ReadIntsFromEEPROM(0);

  SetFastADC();

}



void loop() {
  unsigned long startMicros = micros();
  // We only want to send over USB every millisecond, but we still want to
  // read the analog values as fast as we can to have the most up to date
  // values for the average.
  static bool willSend = (loopTime == -1 ||
                          startMicros - lastSend + loopTime >= 1000);

  serialProcessor.CheckAndMaybeProcessData();

  for (size_t i = 0; i < kNumSensors; ++i) {
    kSensors[i]->EvaluateSensor(willSend);
  }

  if (willSend) {
    lastSend = startMicros;
    
    Joystick.sendState();
    // send the leds directly after sending joystick.
    // note that WS2812B takes 1.5ms to send :(
    #if USE_LEDS
      if (needLEDUpdate) {
        FastLED.show();
        needLEDUpdate = false;
      }
    #endif
    
  }

  if (loopTime == -1) {
    loopTime = micros() - startMicros;
  }

  //save eeprom every 5 seconds if necessary.
  if (ShouldWriteEEPROM())
  {
    WriteIntsToEEPROM(0);
  }
}
