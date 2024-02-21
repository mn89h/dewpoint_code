#include "CapacitorReadout.h"

HardwareTimer* CapacitorReadout::timer;
uint32_t CapacitorReadout::channel;
PinName CapacitorReadout::measurement_pin;
uint32_t CapacitorReadout::input_freq;
volatile uint32_t CapacitorReadout::rolloverCompareCount;
volatile uint32_t CapacitorReadout::LastCapture;
volatile uint32_t CapacitorReadout::CurrentCapture;
volatile bool CapacitorReadout::ignoreMeasurement;
volatile uint32_t CapacitorReadout::samples[CAP_MAXNUMSAMPLES];
volatile uint8_t CapacitorReadout::ctr;

void CapacitorReadout::it_risingEdge() {
  CurrentCapture = TIM4->CCR1; // direct access to Capture Compare Register for channel 2, Timer 2.
  /* frequency computation */
  if (CurrentCapture > LastCapture) {
    samples[ctr] = CurrentCapture - LastCapture;
  }
  else if (CurrentCapture <= LastCapture) {
    /* 0x10000 is max overflow value */
    samples[ctr] = 0x10000 - LastCapture + CurrentCapture;
  }
  rolloverCompareCount = 0;
  LastCapture = CurrentCapture;
  ctr++;
  if (ctr > numSamples - 1) {
    timer->pause();
    ctr = 0;
  }
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void CapacitorReadout::it_rollover() {
  rolloverCompareCount++;
  if (rolloverCompareCount > 1){
    ignoreMeasurement = true;
    Serial.println("NO");
  }
}


CapacitorReadout::CapacitorReadout(PinName pin, uint32_t channel)
    // measurement_pin(pin), channel(channel)
{
    CapacitorReadout::measurement_pin = pin;
    CapacitorReadout::channel = channel;
}

bool CapacitorReadout::init() {

    // Automatically retrieve TIM instance and channel associated to measure_frequency_pin
    // in this case it should retrieve Timer 2 channel 2 which is associated to PA1
    // This automatic algorithm is used to be compatible with all STM32 series automatically.
    // Here we are just checking that it works.
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(measurement_pin, PinMap_PWM);
    channel = STM_PIN_CHANNEL(pinmap_function(measurement_pin, PinMap_PWM));

    // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    timer = new HardwareTimer(Instance);

    // Configure rising edge detection to measure frequency
    timer->setMode(channel, TIMER_INPUT_CAPTURE_RISING, measurement_pin);

    // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
    //  = (SystemCoreClock) / 65535
    // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
    // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
    // The maximum frequency depends on processing of the interruption and thus depend on board used
    // Example on Nucleo_L476RG with systemClock at 80MHz the interruption processing is around 4,5 microseconds and thus Max frequency is around 220kHz
    uint32_t PrescalerFactor = 1;
    timer->setPrescaleFactor(PrescalerFactor);
    timer->setOverflow(0xffff); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
    // TIM4->ARR = 0xefffffff; // basically we extend the count to 32 bits, allowing the measurement of much lower frequencies.
    timer->attachInterrupt(channel, it_risingEdge);
    timer->attachInterrupt(it_rollover);

    // Compute this scale factor only once: G474RE, Prescaler = 2: input_freq = 84 MHz
    // maximum period: 65536/84MHz=780us -> fmin = 1.28
    input_freq = timer->getTimerClkFreq() / timer->getPrescaleFactor();

    return true;
}

void CapacitorReadout::wait() {
  delay(numSamples - 1);      // delay for x ms (assuming minimum frequency is 1 kHz)
}

bool CapacitorReadout::measure(bool asyncMode) {
    if(!asyncMode) {
      delay(1);               // delay before 
    }
    ignoreMeasurement = false;
    timer->resume();
    if(!asyncMode) {
      this->wait();
    }
    timer->pause();           // force timer stop
    if (!ignoreMeasurement) {
      counted_ticks = 0;
      uint32_t used_samples = numSamples - 1;
      uint32_t sorted_ticks[numSamples-1];
      for (int i = 1; i < numSamples; i++) {
        sorted_ticks[i-1] = samples[i];
        counted_ticks += samples[i];
      }

      // using median
      std::sort(std::begin(sorted_ticks), std::end(sorted_ticks));
      int32_t median = sorted_ticks[(numSamples-1)/2];
      
      for (int i = 0; i < numSamples - 1; i++) {
        uint32_t deviation;
        if (median > sorted_ticks[i]) deviation = median - sorted_ticks[i];
        else deviation = sorted_ticks[i] - median;
        if (deviation > outlier_th) {
          used_samples--;
          counted_ticks -= sorted_ticks[i];
        }
      }

      // // using average
      // uint32_t average = counted_ticks / used_samples;
      // for (int i = 1; i < numSamples; i++) {
      //   uint32_t deviation;
      //   if (average > samples[i]) deviation = average - samples[i];
      //   else deviation = samples[i] - average;
      //   if (deviation > outlier_th) {
      //     used_samples--;
      //     counted_ticks -= samples[i];
      //   }
      // }
      if (used_samples != 0)
        counted_ticks = counted_ticks / used_samples;
      else {
        counted_ticks = median;
        // Serial.print(median);
        // for (int i = 0; i < numSamples - 1; i++) {
        //   Serial.print((String)" " + sorted_ticks[i]);
        // }
        // Serial.println();

        // counted_ticks = average;
        // Serial.print(average);
        // for (int i = 1; i < numSamples; i++) {
        //   Serial.print((String)" " + samples[i]);
        // }
        // Serial.println();
      }
    }
    return true;
}

float CapacitorReadout::getFrequency() {
  if (counted_ticks == 0) {
    return __FLT_MAX__;
  }
  float returnVal = (float) input_freq / (counted_ticks);
  return returnVal;
}

uint32_t CapacitorReadout::getTicks() {
  if (counted_ticks == 0) return UINT32_MAX;
  uint32_t returnVal = (counted_ticks);
  return returnVal;
}