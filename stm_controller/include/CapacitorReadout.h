#include <Arduino.h>
#include <HardwareTimer.h>

#ifndef CAPACITORREADOUT_H
#define CAPACITORREADOUT_H  

#define CAP_MAXNUMSAMPLES 20

class CapacitorReadout {
    public:
    CapacitorReadout(PinName pin = PB_6, uint32_t channel = 4);
    bool init();
    bool measure(bool asyncMode = false);
    void wait();
    float getFrequency();
    uint32_t getTicks();

    static void it_risingEdge();
    static void it_rollover();
    
    private:
    static HardwareTimer* timer;
    static uint32_t channel;
    static PinName measurement_pin;
    static const uint32_t numSamples = 8;
    const int32_t outlier_th = 30;
    uint32_t counted_ticks;
    
    static uint32_t input_freq;
    static volatile uint32_t rolloverCompareCount;
    static volatile uint32_t LastCapture;
    static volatile uint32_t CurrentCapture;
    static volatile bool ignoreMeasurement;
    static volatile uint32_t samples[CAP_MAXNUMSAMPLES];
    static volatile uint8_t ctr;

};

#endif