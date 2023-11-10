/*!
 *  @file ADT7422.h
 *
 * 	I2C Driver for Microchip's ADT7422 I2C Temp sensor
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADT7422_H
#define _ADT7422_H

#include "Arduino.h"
#include <Wire.h>

#define ADT7422_I2CADDR_DEFAULT 0x48 ///< I2C address

#define ADT7422_REG__ADT7422_TEMPMSB 0x0 ///< Temp. value MSB
#define ADT7422_REG__ADT7422_TEMPLSB 0x1 ///< Temp. value LSB
#define ADT7422_REG__ADT7422_STATUS 0x2  ///< Status register
#define ADT7422_REG__ADT7422_CONFIG 0x3  ///< Configuration register
#define ADT7422_REG__ADT7422_ID 0xB      ///< Manufacturer identification
#define ADT7422_REG__ADT7422_SWRST 0x2F  ///< Temperature hysteresis


typedef union {
	uint8_t rawData;
	struct {
		uint8_t faultqueue : 2;
		uint8_t ct_polarity : 1;
		uint8_t int_polarity : 1;
		uint8_t intct_mode : 1;
		uint8_t operation_mode : 2;
		uint8_t resolution : 1;
	};
} ADT7422_ConfigurationRegister;

enum class ADT7422_FAULTQ    {NUMFAULTS1 = 0, NUMFAULTS2 = 1, NUMFAULTS3 = 2, NUMFAULTS4 = 3};  //!<  Fault Queue (num faults to trigger int/ct)
enum class ADT7422_CTPOL     {ACTIVELOW = 0, ACTIVEHIGH = 1};                                   //!<  CT Polarity
enum class ADT7422_INTPOL    {ACTIVELOW = 0, ACTIVEHIGH = 1};                                   //!<  INT Polarity
enum class ADT7422_INTCTMODE {INTMODE = 0, COMPMODE = 1};                                       //!<  INT/CT Mode (reset if readback or if within limits)
enum class ADT7422_OPMODE    {CONTINUOUS = 0, ONESHOT, ONESPS, SHUTDOWN};                       //!<  Operating Mode
enum class ADT7422_RES       {NUMBITS13 = 0, NUMBITS16};                                        //!<  Number of Bits (16 or 13 w/ CT&INT info)

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            ADT7422 Temp Sensor
 */
class ADT7422 {
public:
  ADT7422(TwoWire *wire, uint8_t a = ADT7422_I2CADDR_DEFAULT);
  void init();
  void reset();
  void setResolution(ADT7422_RES resolution);
  void setOperationMode(ADT7422_OPMODE opmode);
  float readTemperature();

private:
  TwoWire* wire = NULL;
  uint8_t address;
  ADT7422_ConfigurationRegister config;

  void writeConfig();
};

#endif
