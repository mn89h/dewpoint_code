#include <Arduino.h>
#include <stm32g474xx.h>

template <typename T>
class TempSensor {
    private:
    TwoWire* wire;

    public:
    T* sensor;
    TempSensor(T* sensor, TwoWire* wire) : sensor(sensor), wire(wire) {}
    void setup();
    int getTemperature();
    void setAlert();
};