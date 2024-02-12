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
#define SHT4X_NOHEAT_HIGHPRECISION                                             \
  0xFD /**< High precision measurement, no heater */
#define SHT4X_NOHEAT_MEDPRECISION                                              \
  0xF6 /**< Medium precision measurement, no heater */
#define SHT4X_NOHEAT_LOWPRECISION                                              \
  0xE0 /**< Low precision measurement, no heater */

#define SHT4X_HIGHHEAT_1S                                                      \
  0x39 /**< High precision measurement, high heat for 1 sec */
#define SHT4X_HIGHHEAT_100MS                                                   \
  0x32 /**< High precision measurement, high heat for 0.1 sec */
#define SHT4X_MEDHEAT_1S                                                       \
  0x2F /**< High precision measurement, med heat for 1 sec */
#define SHT4X_MEDHEAT_100MS                                                    \
  0x24 /**< High precision measurement, med heat for 0.1 sec */
#define SHT4X_LOWHEAT_1S                                                       \
  0x1E /**< High precision measurement, low heat for 1 sec */
#define SHT4X_LOWHEAT_100MS                                                    \
  0x15 /**< High precision measurement, low heat for 0.1 sec */

#define SHT4X_READSERIAL 0x89 /**< Read Out of Serial Register */
#define SHT4X_SOFTRESET 0x94  /**< Soft Reset */

/** How precise (repeatable) the measurement will be */
typedef enum {
  SHT4X_HIGH_PRECISION,
  SHT4X_MED_PRECISION,
  SHT4X_LOW_PRECISION,
} sht4x_precision_t;

/** Optional pre-heater configuration setting */
typedef enum {
  SHT4X_NO_HEATER,
  SHT4X_HIGH_HEATER_1S,
  SHT4X_HIGH_HEATER_100MS,
  SHT4X_MED_HEATER_1S,
  SHT4X_MED_HEATER_100MS,
  SHT4X_LOW_HEATER_1S,
  SHT4X_LOW_HEATER_100MS,
} sht4x_heater_t;

/**
 * Driver for the Adafruit SHT4X Temperature and Humidity breakout board.
 */
class SHT4X {
public:
  SHT4X(TwoWire* wire);

  bool init();
  uint32_t readSerial(void);
  bool reset(void);

  void setPrecision(sht4x_precision_t prec);
  sht4x_precision_t getPrecision(void);
  void setHeater(sht4x_heater_t heat);
  sht4x_heater_t getHeater(void);

private:
  float _temperature, ///< Last reading's temperature (C)
      _humidity;      ///< Last reading's humidity (percent)

  TwoWire* i2c_dev = NULL;      ///< Pointer to I2C bus interface

  sht4x_precision_t _precision = SHT4X_HIGH_PRECISION;
  sht4x_heater_t _heater = SHT4X_NO_HEATER;

  bool writeCommand(uint16_t cmd);
  bool readCommand(uint16_t command, uint8_t *buffer, uint8_t num_bytes);
};

#endif
