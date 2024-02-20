#ifndef __SHT45_H__
#define __SHT45_H__

#include <Arduino.h>
#include <Wire.h>

class SHT45 {
    private:
        uint8_t _addr;
        TwoWire *_wire = NULL;
        float _t, _h;

    public:
        SHT45(TwoWire *wire = &Wire, uint8_t addr = 0x44);
        bool init();

        //these methods performs measurement, waiting and data acquisition
        //be sure to call before reading values (asyncMode == false performs wait and receive within measure)
        bool measure(bool asyncMode = false);
        void wait();
        bool receiveData();
        float temperature();
        float humidity();

};

#endif