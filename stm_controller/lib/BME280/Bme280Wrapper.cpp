#include <Arduino.h>

#include <limits.h>

#include "BME280Wrapper.h"

#define DOUBLE_NOT_CALCULATED -1000.0

int BME280Wrapper::_cs = -1;
TwoWire* BME280Wrapper::wire = nullptr;

BME280Wrapper::BME280Wrapper(TwoWire* wire)
{
  useI2C = true;
  BME280Wrapper::wire = wire;
}

BME280Wrapper::BME280Wrapper(int8_t cspin)
{
  useI2C = false;
  BME280Wrapper::_cs = cspin;
}

bool BME280Wrapper::init(bool forced) {
  this->forced = forced;
  if (useI2C) {
    return beginI2C();
  }
  else {
    return beginSPI();
  }
}

bool BME280Wrapper::beginI2C()
{
  static uint8_t dev_addr_place;

  I2CInit();

  dev_addr_place = BME280_I2C_ADDR;
  bme280.intf_ptr = &dev_addr_place;
  bme280.intf = BME280_I2C_INTF;

  int8_t ret = bme280_init(&bme280);

  setSensorSettings();

  return (ret == BME280_OK);
}

bool BME280Wrapper::beginSPI()
{
  static uint8_t dev_addr_place;

  dev_addr_place = 0;
  bme280.intf_ptr = &dev_addr_place;
  bme280.intf = BME280_SPI_INTF;

  SPIInit();
  pinMode(_cs, OUTPUT);

  int8_t ret = bme280_init(&bme280);

  setSensorSettings();

  return (ret == BME280_OK);
}

void BME280Wrapper::wait() {
  if(forced) {
    uint32_t tmpDelay = bme280_cal_meas_delay(&(bme280.settings));
    delay(tmpDelay);
  }
}

bool BME280Wrapper::receiveData() {
  int8_t ret = BME280_OK;

  ret += bme280_get_sensor_data(BME280_PRESS | BME280_HUM | BME280_TEMP, &comp_data, &bme280);

  return (ret == BME280_OK);
}
  
bool BME280Wrapper::measure(bool asyncMode) {
  if(forced)
  {
    setSensorSettings();
  }
  // else periodic measurement

  if (!asyncMode) {
    wait();
    receiveData();
  }
  return true;
}

int32_t BME280Wrapper::getTemperature()
{
  return comp_data.temperature;
}

uint32_t BME280Wrapper::getHumidity()
{
  return comp_data.humidity;
}

uint32_t BME280Wrapper::getPressure()
{
  return comp_data.pressure;
}

/**
 * Wrapper functions for Bosch BME280 driver.
 */
#define SPI_READ  0x80
#define SPI_WRITE 0x7F

SPISettings bme280SpiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

void BME280Wrapper::I2CInit() 
{
  bme280.intf = BME280_I2C_INTF;
  bme280.write = BME280Wrapper::I2CWrite;
  bme280.read = BME280Wrapper::I2CRead;
  bme280.delay_us = BME280Wrapper::delayusec;
}

void BME280Wrapper::SPIInit() 
{
  bme280.intf = BME280_SPI_INTF;
  bme280.write = BME280Wrapper::SPIWrite;
  bme280.read = BME280Wrapper::SPIRead;
  bme280.delay_us = BME280Wrapper::delayusec;

  SPI.begin();
}

int8_t BME280Wrapper::I2CRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{
  uint8_t dev_addr = *((uint8_t *)intf_ptr);
//  Serial.println("I2C_bus_read");
  int8_t ret = BME280_OK;

//  Serial.println(dev_addr, HEX);
  wire->beginTransmission(dev_addr);
  
//  Serial.println(reg_addr, HEX);
  wire->write(reg_addr);
  wire->endTransmission();
  
  wire->requestFrom((int)dev_addr, (int)cnt);
  
  uint8_t available = wire->available();
  if(available != cnt)
  {
    ret = BME280_E_COMM_FAIL;
  }
  
  for(uint8_t i = 0; i < available; i++)
  {
    if(i < cnt) 
    {
      *(reg_data + i) = wire->read();
//      Serial.print(*(reg_data + i), HEX);
//      Serial.print(" ");
    }
    else
      wire->read();
  }

//  Serial.println();
  
  return ret;
}

int8_t BME280Wrapper::I2CWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{  
  uint8_t dev_addr = *((uint8_t *)intf_ptr);
//  Serial.println("I2C_bus_write");
  int8_t ret = BME280_OK;

//  Serial.println(dev_addr, HEX);
  wire->beginTransmission(dev_addr);

//  Serial.println(reg_addr, HEX);
  wire->write(reg_addr);
  wire->write(reg_data, cnt);
  wire->endTransmission();
  
  return ret;
}

int8_t BME280Wrapper::SPIRead(uint8_t reg_addr, uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{
//  Serial.println("SPI_bus_read");
  int32_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);

//  Serial.println(reg_addr | SPI_READ, HEX);

  SPI.transfer(reg_addr | SPI_READ);
  for (uint8_t i = 0; i < cnt; i++) {
    *(reg_data + i) = SPI.transfer(0);
    
//    Serial.print(*(reg_data + i), HEX);
//    Serial.print(" ");
  }

//  Serial.println();
  
  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

int8_t BME280Wrapper::SPIWrite(uint8_t reg_addr, const uint8_t *reg_data, uint32_t cnt, void *intf_ptr)
{
//  Serial.println("SPI_bus_write");
  int8_t ret = BME280_OK;

  SPI.beginTransaction(bme280SpiSettings);
  digitalWrite(_cs, LOW);
  for (uint8_t i = 0; i < cnt; i++) 
  {
    uint8_t addr = (reg_addr++) & SPI_WRITE;
    uint8_t data = *(reg_data + i);

//    Serial.print(addr, HEX);
//    Serial.print(" ");
//    Serial.print(data, HEX);

    SPI.transfer(addr);
    SPI.transfer(data);
  }
//  Serial.println();

  digitalWrite(_cs, HIGH);
  SPI.endTransaction();

  return ret;
}

void BME280Wrapper::delayusec(uint32_t period, void *intf_ptr)
{
  delayMicroseconds(period);
}

int8_t BME280Wrapper::setSensorSettings()
{
  int8_t ret = BME280_OK;

  uint8_t settings_sel;

  bme280.settings.osr_h = BME280_OVERSAMPLING_16X;
  bme280.settings.osr_p = BME280_OVERSAMPLING_16X;
  bme280.settings.osr_t = BME280_OVERSAMPLING_16X;

  settings_sel = BME280_OSR_PRESS_SEL|BME280_OSR_TEMP_SEL|BME280_OSR_HUM_SEL;

  ret += bme280_set_sensor_settings(settings_sel, &bme280);

  if(forced)
    ret += bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280);
  else
    ret += bme280_set_sensor_mode(BME280_NORMAL_MODE, &bme280);

  return ret;
}
