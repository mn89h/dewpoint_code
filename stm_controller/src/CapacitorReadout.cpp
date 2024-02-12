#include "CapacitorReadout.h"

HardwareTimer* CapacitorReadout::timer;
uint32_t CapacitorReadout::channel;
PinName CapacitorReadout::measurement_pin;
uint32_t CapacitorReadout::numSamples;
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
    samples[ctr] = 0x10000 + CurrentCapture - LastCapture;
  }
  ctr++;
  if (ctr > numSamples - 1) {
    timer->pause();
    ctr = 0;
  }
  rolloverCompareCount = 0;
  LastCapture = CurrentCapture;
}

/* In case of timer rollover, frequency is to low to be measured set value to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void CapacitorReadout::it_rollover() {
  rolloverCompareCount++;
  if (rolloverCompareCount > 1){
    ignoreMeasurement = true;
  }
}


CapacitorReadout::CapacitorReadout(PinName pin, uint32_t channel)
    // measurement_pin(pin), channel(channel)
{
    CapacitorReadout::measurement_pin = pin;
    CapacitorReadout::channel = channel;
}

bool CapacitorReadout::init() {
    numSamples = 4;

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
    timer->setOverflow(50000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
    TIM4->ARR = 0xffffffff; // basically we extend the count to 32 bits, allowing the measurement of much lower frequencies.
    timer->attachInterrupt(channel, it_risingEdge);
    timer->attachInterrupt(it_rollover);

    // Compute this scale factor only once: G474RE, Prescaler = 2: input_freq = 84 MHz
    // maximum period: 65536/84MHz=780us -> fmin = 1.28
    input_freq = timer->getTimerClkFreq() / timer->getPrescaleFactor();

    return true;
}

float CapacitorReadout::getFrequency() {
    ignoreMeasurement = false;
    timer->resume();
    delay(numSamples - 1);  // delay for x ms (assuming minimum frequency is 1 kHz)
    timer->pause();         // force timer stop
    float returnVal = 0.0;
    if (!ignoreMeasurement) {
        for (int i = 1; i < numSamples; i++) {
            returnVal += samples[i];
        }
        returnVal = input_freq / (returnVal / (numSamples - 1));
    }
    return returnVal;
}

uint32_t CapacitorReadout::getTicks() {
    ignoreMeasurement = false;
    timer->resume();
    delay(numSamples - 1);  // delay for x ms (assuming minimum frequency is 1 kHz)
    timer->pause();         // force timer stop
    uint32_t returnVal = 0;
    if (!ignoreMeasurement) {
        for (int i = 1; i < numSamples; i++) {
            returnVal += samples[i];
        }
        returnVal = (returnVal / (numSamples - 1));
    }
    return returnVal;
}