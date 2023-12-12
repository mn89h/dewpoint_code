/*!
 *  @file ADT7422.cpp
 *
 *  @mainpage Adafruit ADT7422 I2C Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for Microchip's ADT7422 I2C Temp sensor
 *
 * 	This is a library for the Adafruit ADT7422 breakout:
 * 	http://www.adafruit.com/products/1782
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section author Author
 *
 *  K.Townsend (Adafruit Industries)
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "ADT7422.h"

/*!
 *    @brief  Instantiates a new ADT7422 class
 *    @param  addr The I2C address, defaults to 0x48
 *    @param  wire The I2C interface, pointer to a TwoWire, defaults to WIre
 */
ADT7422::ADT7422(TwoWire *wire, uint8_t addr) :
  wire(wire), address(addr) 
{
  config.rawData = 0x00;
}

/*!
 *    @brief  Setups the HW
 *    @return True if initialization was successful, otherwise false.
 */
bool ADT7422::init() {
  if(readDeviceID() != 0xCB) {
    return false;
  }
  reset();
  setResolution(ADT7422_RES::NUMBITS16);
  return true;
}

/// @brief Writes the configuration
void ADT7422::writeConfig() {
  wire->beginTransmission(address);
	wire->write(ADT7422_REG__CONFIG);
  wire->write(config.rawData);
	wire->endTransmission();
}

/// @brief Sets the resolution
/// @param resolution 
void ADT7422::setResolution(ADT7422_RES resolution) {
  config.resolution = (uint8_t) resolution;
  writeConfig();
}

/// @brief Sets the operation mode (continuous, one-shot, 1sps, shutdown)
/// @param opmode 
void ADT7422::setOperationMode(ADT7422_OPMODE opmode) {
  config.operation_mode = (uint8_t) opmode;
  writeConfig();
}

/*!
 *   @brief  Perform a soft reset
 *   @return True on success
 */
void ADT7422::reset() {
  wire->beginTransmission(address);
	wire->write(ADT7422_REG__SWRST);
	wire->endTransmission();
  delay(10);
}

/*!
 *   @brief  Reads the 13-bit or 16-bit temperature register and returns the Centigrade
 *           temperature as a float.
 *   @return Temperature in Centigrade.
 */
float ADT7422::readTemperature() {
  wire->beginTransmission(address);
	wire->write(ADT7422_REG__TEMPMSB);
	wire->endTransmission(false); // IMPORTANT: Repeated Start

	wire->requestFrom(address, (uint8_t)2);

	uint8_t msb = wire->read();
	uint8_t lsb = wire->read();

	uint16_t val = msb << 8 | lsb;

  float temp = (int16_t) val;
  if(config.resolution == (uint8_t) ADT7422_RES::NUMBITS16) {   // 16-bit
    temp = temp / 128.0;
  }
  else {                                                        // 13-bit
    temp = ((int16_t) val) >> 3;
    temp = temp / 16.0;
  }

  return temp;
}

uint8_t ADT7422::readDeviceID() {
  wire->beginTransmission(address);
  wire->write(ADT7422_REG__ID);
  wire->endTransmission(false); // IMPORTANT: Repeated Start

  wire->requestFrom(address, 1);
  uint8_t id = wire->read();

  return id;
}
