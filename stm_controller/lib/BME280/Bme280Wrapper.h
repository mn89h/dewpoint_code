#ifndef __BMP280_BW_H__
#define __BMP280_BW_H__

#include <bme280.h>
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#define BME280_I2C_ADDR 0x76

class BME280Wrapper
{
  public:
    //true: uses forced mode, sensore measures values on demand
    //false: uses continuous measuring mode
    BME280Wrapper(TwoWire* wire);
    BME280Wrapper(int8_t cspin);

    bool init(bool forced = true);

    //these methods performs measurement, waiting and data acquisition
    //be sure to call before reading values (asyncMode == false performs wait and receive within measure)
    bool measure(bool asyncMode = false);
    void wait();
    bool receiveData();

    //Temperature in degrees of Celsius * 100
    int32_t getTemperature();

    //Relative humidity in % * 1024
    uint32_t getHumidity();

    //Air pressure in Pa
    uint32_t getPressure();

  private:
    bool beginI2C();
    bool beginSPI();
    void I2CInit();
    void SPIInit();
    int8_t setSensorSettings();
  
    static int8_t I2CRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
    static int8_t I2CWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
    static int8_t SPIRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
    static int8_t SPIWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr);
    static void delayusec(uint32_t period, void *intf_ptr);

    static int _cs;
    static TwoWire* wire;

    struct bme280_dev bme280;
    struct bme280_data comp_data;

    bool useI2C;
    bool forced = false;
    bool error = false;
};

#endif

