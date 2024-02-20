

/*!
 *  @file VCNL4040.cpp
 *
 *  @mainpage VCNL4040 proximity and ambient light sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the VCNL4040 proximity and ambient light sensor library
 *
 *  @section author Author
 *
 *  Malte Nilges
 *  Bryan Siepert for Adafruit Industries
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 */

#include "Arduino.h"
#include <Wire.h>

#include "VCNL4040.hpp"


/*!
 *    @brief  Instantiates a new VCNL4040 class
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  i2c_address
 *            The I2C address to be used.
 */
VCNL4040::VCNL4040(TwoWire *wire, uint8_t i2c_addr) :
  wire(wire), address(i2c_addr) 
{
  als_conf.rawData = 0x0001;
  als_thdHigh.rawData = 0x0000;
  als_thdLow.rawData = 0x0000;
  ps_conf12.rawData = 0x0001;
  ps_conf3Ms.rawData = 0x0000;
  ps_canc.rawData = 0x0000;
  ps_thdLow.rawData = 0x0000;
  ps_thdLow.rawData = 0x0000;
}


/*!
 *    @brief  Initializes the settings
 *    @return True if initialization was successful, otherwise false.
 */
bool VCNL4040::init() {
  return _init();
}

bool VCNL4040::_init() {
  delay(3);

  setProximityLEDCurrent(VCNL4040_LEDCurrent::LED_CURRENT_200MA);
  setProximityLEDDutyCycle(VCNL4040_LEDDutyCycle::LED_DUTY_1_80);
  setProximityIntegrationTime(VCNL4040_ProximityIntegration::PROXIMITY_INTEGRATION_TIME_8T);
  setProximityHighResolution(true);
  enableForceMode(true);
  enableProximity(true);
  enableWhiteLight(false);
  enableAmbientLight(false);

  return true;
}
/**************** Sensor Data Getters *************************************/

/*!
    @brief Gets the current proximity sensor value.
    @return The current proximity measurement in units
*/
uint16_t VCNL4040::getProximity() {
  return readRegister(VCNL4040_REG__PS_DATA);
}

/*!
    @brief Gets the current ambient light sensor value.
    @return The current ambient light measurement in units
*/
uint16_t VCNL4040::getAmbientLight() {
  return readRegister(VCNL4040_REG__ALS_DATA);
}

/*!
    @brief Gets the current white light value.
    @return The current white light measurement in units
*/
uint16_t VCNL4040::getWhiteLight() {
  return readRegister(VCNL4040_REG__WHITE_DATA);
}
/*!
    @brief Enable Force Mode (PS_AF)
    @param  enable
            Set to true to enable proximity measurements,
            set to false to disable.
*/
void VCNL4040::enableForceMode(bool enable) {
  ps_conf3Ms.af = (uint8_t) enable;
  writeRegister(VCNL4040_REG__PS_CONF3_MS, ps_conf3Ms.rawData);
}

/*!
    @brief Wait for 6500 (used after single triggering)
*/
void VCNL4040::wait() {
  delayMicroseconds(6000);
}

/*!
    @brief Trigger Single Measurement (Force Mode needs to be enabled first).
    @param asyncMode False: delay after trigger, True: do nothing.
*/
bool VCNL4040::triggerSingle(bool asyncMode) {
  ps_conf3Ms.trig = 1;
  writeRegister(VCNL4040_REG__PS_CONF3_MS, ps_conf3Ms.rawData);
  if(!asyncMode) wait();
  return true;
}

/*!
    @brief Gets the current ambient light sensor in Lux.
    @return The current ambient light measurement in Lux
*/
float VCNL4040::getLux() {
  // scale the lux depending on the value of the integration time
  // see page 8 of the VCNL4040 application note:
  // https://www.vishay.com/docs/84307/designingvcnl4040.pdf
  return (readRegister(VCNL4040_REG__ALS_DATA) * (0.1 / (1 << (uint8_t)getAmbientIntegrationTime())));
}
/**************** Sensor Enable Functions   *******************************/


/*!
    @brief Enables or disables proximity measurements.
    @param  enable
            Set to true to enable proximity measurements,
            set to false to disable.
*/
void VCNL4040::enableProximity(bool enable) {
  ps_conf12.sd = (uint8_t) ~enable;
  writeRegister(VCNL4040_REG__PS_CONF1_2, ps_conf12.rawData);
}

/*!
    @brief Enables ambient light measurements
    @param  enable
            Set to true to enable ambient light measurements,
            set to false to disable.
*/
void VCNL4040::enableAmbientLight(bool enable) {
  als_conf.sd = (uint8_t) ~enable;
  writeRegister(VCNL4040_REG__ALS_CONF1, als_conf.rawData);
}

/*!
    @brief Enables white light measurements
    @param  enable
            Set to true to enable white light measurements,
            set to false to disable.
*/
void VCNL4040::enableWhiteLight(bool enable) {
  ps_conf3Ms.white_en = (uint8_t) ~enable;
  writeRegister(VCNL4040_REG__PS_CONF3_MS, ps_conf3Ms.rawData);
}

/******************** Tuning Functions ********************************** */


/*!
    @brief Gets the integration time for proximity sensing measurements.
    @returns The integration time being used for proximity measurements.
*/
VCNL4040_ProximityIntegration VCNL4040::getProximityIntegrationTime() {
  return (VCNL4040_ProximityIntegration) ps_conf12.it;
}

/*!
    @brief Sets the integration time for proximity sensing measurements.
    @param  integration_time
            The integration time to use for proximity measurements. Must be a
            `VCNL4040_ProximityIntegration`.
*/
void VCNL4040::setProximityIntegrationTime(VCNL4040_ProximityIntegration integration_time) {
  ps_conf12.it = (uint8_t) integration_time;
  writeRegister(VCNL4040_REG__PS_CONF1_2, ps_conf12.rawData);
}


/*!
    @brief Gets the integration time for ambient light sensing measurements.
    @returns The integration time being used for ambient light measurements.
*/
VCNL4040_AmbientIntegration VCNL4040::getAmbientIntegrationTime() {
  return (VCNL4040_AmbientIntegration) als_conf.it;
}


/*!
    @brief Sets the integration time for ambient light sensing measurements.
    @param  integration_time
            The integration time to use for ambient light measurements. Must be
   a `VCNL4040_AmbientIntegration`.
*/
void VCNL4040::setAmbientIntegrationTime(VCNL4040_AmbientIntegration integration_time) {
  // delay according to the integration time to let the reading at the old IT
  // clear out
  uint16_t old_it_ms = ((8 << als_conf.it) * 10);
  uint16_t new_it_ms = ((8 << (uint8_t)integration_time) * 10);

  als_conf.it = (uint8_t)integration_time;
  writeRegister(VCNL4040_REG__ALS_CONF1, als_conf.rawData);
  delay((old_it_ms + new_it_ms + 1));
}


/*!
    @brief Gets the current for the LED used for proximity measurements.
    @returns The LED current value being used for proximity measurements.
*/
VCNL4040_LEDCurrent VCNL4040::getProximityLEDCurrent() {
  return (VCNL4040_LEDCurrent)ps_conf3Ms.led_i;
}

/*!
    @brief Sets the current for the LED used for proximity measurements.
    @param  led_current
            The current value to be used for proximity measurements. Must be a
            `VCNL4040_LEDCurrent`.
*/
void VCNL4040::setProximityLEDCurrent(VCNL4040_LEDCurrent led_current) {
  ps_conf3Ms.led_i = (uint8_t) led_current;
  writeRegister(VCNL4040_REG__PS_CONF3_MS, ps_conf3Ms.rawData);
}


/*!
    @brief Sets the duty cycle for the LED used for proximity measurements.
    @returns The duty cycle value being used for proximity measurements.
*/
VCNL4040_LEDDutyCycle VCNL4040::getProximityLEDDutyCycle() {
  return (VCNL4040_LEDDutyCycle)ps_conf12.duty;
}

/*!
    @brief Sets the duty cycle for the LED used for proximity measurements.
    @param  duty_cycle
            The duty cycle value to be used for proximity measurements. Must be
   a `VCNL4040_LEDDutyCycle`.
*/
void VCNL4040::setProximityLEDDutyCycle(VCNL4040_LEDDutyCycle duty_cycle) {
  ps_conf12.duty = (uint8_t) duty_cycle;
  writeRegister(VCNL4040_REG__PS_CONF1_2, ps_conf12.rawData);
}


/*!
    @brief Gets the resolution of proximity measurements
    @return The current proximity measurement resolution
            If true, proximity measurements are 16-bit,
            If false, proximity measurements are 12-bit,
*/
bool VCNL4040::getProximityHighResolution() {
  return (bool)ps_conf12.hd;
}

/*!
    @brief Sets the resolution of proximity measurements
    @param  high_resolution
            Set to true to take 16-bit measurements for proximity,
            set to faluse to use 12-bit measurements.
*/
void VCNL4040::setProximityHighResolution(bool high_resolution) {
  ps_conf12.hd = (uint8_t) high_resolution;
  writeRegister(VCNL4040_REG__PS_CONF1_2, ps_conf12.rawData);
}

/*!
    @brief Writes a register with specified data
    @param reg Register to write to
    @param data Data to write
*/
void VCNL4040::writeRegister(uint8_t reg, uint16_t data) {
  wire->beginTransmission(address);
	wire->write(reg);
  wire->write(lowByte(data));
  wire->write(highByte(data));
	wire->endTransmission();
}

/*!
    @brief  Reads the data stored within the specified register
    @param  reg Register to be read.
    @return Data stored in register
 */
uint16_t VCNL4040::readRegister(uint8_t reg) {
  wire->beginTransmission(address);
	wire->write(reg);
	wire->endTransmission(false);

	wire->requestFrom(address, (uint8_t)2);

	uint8_t lsb = wire->read();
	uint8_t msb = wire->read();

	uint16_t val = msb << 8 | lsb;
  return val;
}