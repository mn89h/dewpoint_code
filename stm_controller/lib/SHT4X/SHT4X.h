/*!
 *  @file SHT4X.h
 *
 *  This is a library for the SHT4X Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT4X Humidity & Temp Sensor
 *  -----> https://www.adafruit.com/product/4885
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef SHT4X_H
#define SHT4X_H

#include "Arduino.h"
#include "Wire.h"

#define SHT4X_DEFAULT_ADDR 0x44 /**< SHT4X I2C Address */

#define SHT4X_READSERIAL 0x89 /**< Read Out of Serial Register */
#define SHT4X_SOFTRESET 0x94  /**< Soft Reset */

/** Command setting */
typedef enum {
  SHT4X_NO_HEATER_HIGHPRECISION = 0xFD,
  SHT4X_NO_HEATER_MEDPRECISION = 0xF6,
  SHT4X_NO_HEATER_LOWPRECISION = 0xE0,
  SHT4X_HIGH_HEATER_1S = 0x39,
  SHT4X_HIGH_HEATER_100MS = 0x32,
  SHT4X_MED_HEATER_1S = 0x2F,
  SHT4X_MED_HEATER_100MS = 0x24,
  SHT4X_LOW_HEATER_1S = 0x1E,
  SHT4X_LOW_HEATER_100MS = 0x15,
} sht4x_cmd;

/**
 * Driver for the Adafruit SHT4X Temperature and Humidity breakout board.
 */
class SHT4X {
public:
  SHT4X(TwoWire* wire);

  bool init();
  //these methods performs measurement, waiting and data acquisition
  //be sure to call before reading values (asyncMode == false performs wait and receive within measure)
  bool measure(bool asyncMode = false);
  void wait();
  bool receiveData();
  float getTemperature();
  float getHumidity();
  uint32_t readSerial(void);
  bool reset(void);

  void setCommand(sht4x_cmd cmd);
  sht4x_cmd getCommand(void);

private:
  float _temperature, ///< Last reading's temperature (C)
      _humidity;      ///< Last reading's humidity (percent)

  TwoWire* wire = NULL;      ///< Pointer to I2C bus interface

  sht4x_cmd _cmd = SHT4X_NO_HEATER_HIGHPRECISION;

  bool readBytes(uint8_t *val, uint8_t n);
  bool writeCmd(uint8_t cmd);
};

#endif
