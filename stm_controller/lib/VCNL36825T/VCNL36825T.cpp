

/*!
 *  @file VCNL36825T.cpp
 *
 *  @mainpage VCNL36825T proximity and ambient light sensor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the VCNL36825T proximity and ambient light sensor library
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

#include "VCNL36825T.hpp"


/*!
 *    @brief  Instantiates a new VCNL36825T class
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  i2c_address
 *            The I2C address to be used.
 */
VCNL36825T::VCNL36825T(TwoWire *wire, uint8_t i2c_addr) :
  wire(wire), address(i2c_addr)
{
  ps_conf1.rawData = 0x0001;
  writeRegister(VCNL36825T_REG__PS_CONF1, ps_conf1.rawData);
  ps_conf2.rawData = 0x0001;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
  ps_conf3.rawData = 0x0000;
  writeRegister(VCNL36825T_REG__PS_CONF3, ps_conf3.rawData);
  ps_conf4.rawData = 0x0000;
  ps_thdLow.rawData = 0x0000;
  ps_thdHigh.rawData = 0x0000;
  ps_canc.rawData = 0x0000;
}


/*!
 *    @brief  Initializes the settings
 *    @return True if initialization was successful, otherwise false.
 */
bool VCNL36825T::init() {
  return _init();
}

bool VCNL36825T::_init() {
  delay(5);
  enableSensor(true);
  setProximityHighResolution(true);
  setProximityIntegrationTime(VCNL36825T_ProximityIntegration::PROXIMITY_INTEGRATION_TIME_8T);
  setProximityIntegrationTimeBank(VCNL36825T_ProximityIntegrationBank::PROXIMITY_INTEGRATION_TIME_BANK_50US);
  setMultiPulse(VCNL36825T_MultiPulse::MPS4);
  setProximityLEDCurrent(VCNL36825T_LEDCurrent::LED_CURRENT_20MA);
  setProximityLEDPeriod(VCNL36825T_LEDPeriod::PERIOD_80MS);

  enableForceMode(true);
  enableProximity(true);

  return true;
}
/**************** Sensor Data Getters *************************************/

/*!
    @brief Gets the current proximity sensor value.
    @return The current proximity measurement in units
*/
uint16_t VCNL36825T::getProximity() {
  return readRegister(VCNL36825T_REG__PS_DATA);
}

/**************** Sensor Enable Functions   *******************************/

/*!
    @brief Initiates the startup sequence according to datasheet
*/
void VCNL36825T::enableSensor(bool start) {
  if (start) {
    ps_conf1.on = 1;
    writeRegister(VCNL36825T_REG__PS_CONF1, ps_conf1.rawData);
    delay(3);
    ps_conf1.cal = 1;
    // ps_conf1.reserved3 = 1 << 1;
    writeRegister(VCNL36825T_REG__PS_CONF1, ps_conf1.rawData);
  }
  else {
    ps_conf1.on = 0;
    ps_conf1.cal = 0;
    writeRegister(VCNL36825T_REG__PS_CONF1, ps_conf1.rawData);
  }
  delay(5);
}

/*!
    @brief Enables or disables proximity measurements.
    @param  enable
            Set to true to enable proximity measurements,
            set to false to disable.
*/
void VCNL36825T::enableProximity(bool enable) {
  ps_conf2.st = (uint8_t) ~enable;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
}

/*!
    @brief Enable Force Mode (PS_AF)
    @param  enable
            Set to true to enable proximity measurements,
            set to false to disable.
*/
void VCNL36825T::enableForceMode(bool enable) {
  ps_conf3.af = (uint8_t) enable;
  writeRegister(VCNL36825T_REG__PS_CONF3, ps_conf3.rawData);
}

/*!
    @brief Wait for 6500 (used after single triggering)
*/
void VCNL36825T::wait() {
  delayMicroseconds(6500); // valid for (IT = 400us, MPS = 4)
}

/*!
    @brief Trigger Single Measurement (Force Mode needs to be enabled first).
    @param asyncMode False: delay after trigger, True: do nothing.
*/
bool VCNL36825T::triggerSingle(bool asyncMode) {
  ps_conf3.trig = 1;
  writeRegister(VCNL36825T_REG__PS_CONF3, ps_conf3.rawData);
  ps_conf3.trig = 0;
  if(!asyncMode) wait();
  getProximity(); // ugly workaround for values not appearing after delay and single read
  return true;
}

/******************** Tuning Functions ********************************** */


/*!
    @brief Gets the integration time for proximity sensing measurements.
    @returns The integration time being used for proximity measurements.
*/
VCNL36825T_ProximityIntegration VCNL36825T::getProximityIntegrationTime() {
  return (VCNL36825T_ProximityIntegration) ps_conf2.it;
}

/*!
    @brief Sets the integration time for proximity sensing measurements.
    @param  integration_time
            The integration time to use for proximity measurements. Must be a
            `VCNL36825T_ProximityIntegration`.
*/
void VCNL36825T::setProximityIntegrationTime(VCNL36825T_ProximityIntegration integration_time) {
  ps_conf2.it = (uint8_t) integration_time;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
}

/*!
    @brief Gets the integration time bank, or timebase of the integration time setting
    @return integration time bank
*/
VCNL36825T_ProximityIntegrationBank VCNL36825T::getProximityIntegrationTimeBank() {
  return (VCNL36825T_ProximityIntegrationBank)ps_conf2.itb;
}

/*!
    @brief Sets the integration time bank, or timebase of the integration time setting
    @param  integration_time_bank timebase of integration time setting
*/
void VCNL36825T::setProximityIntegrationTimeBank(VCNL36825T_ProximityIntegrationBank integration_time_bank) {
  ps_conf2.itb = (uint8_t) integration_time_bank;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
}


VCNL36825T_MultiPulse VCNL36825T::getMultiPulse() {
  return (VCNL36825T_MultiPulse)ps_conf2.mps;
}

void VCNL36825T::setMultiPulse(VCNL36825T_MultiPulse multi_pulse) {
  ps_conf2.mps = (uint8_t) multi_pulse;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
}

/*!
    @brief Gets the current for the LED used for proximity measurements.
    @returns The LED current value being used for proximity measurements.
*/
VCNL36825T_LEDCurrent VCNL36825T::getProximityLEDCurrent() {
  return (VCNL36825T_LEDCurrent)ps_conf3.i_vcsel;
}

/*!
    @brief Sets the current for the LED used for proximity measurements.
    @param  led_current
            The current value to be used for proximity measurements. Must be a
            `VCNL36825T_LEDCurrent`.
*/
void VCNL36825T::setProximityLEDCurrent(VCNL36825T_LEDCurrent led_current) {
  ps_conf3.i_vcsel = (uint8_t) led_current;
  writeRegister(VCNL36825T_REG__PS_CONF3, ps_conf3.rawData);
}

/*!
    @brief Sets the duty cycle for the LED used for proximity measurements.
    @returns The duty cycle value being used for proximity measurements.
*/
VCNL36825T_LEDPeriod VCNL36825T::getProximityLEDPeriod() {
  return (VCNL36825T_LEDPeriod)ps_conf2.period;
}

/*!
    @brief Sets the period used for proximity measurements.
    @param  period
            The period used for proximity measurements.
*/
void VCNL36825T::setProximityLEDPeriod(VCNL36825T_LEDPeriod period) {
  ps_conf2.period = (uint8_t) period;
  writeRegister(VCNL36825T_REG__PS_CONF2, ps_conf2.rawData);
}

/*!
    @brief Gets the resolution of proximity measurements
    @return The current proximity measurement resolution
            If true, proximity measurements are 16-bit,
            If false, proximity measurements are 12-bit,
*/
bool VCNL36825T::getProximityHighResolution() {
  return (bool)ps_conf3.hd;
}

/*!
    @brief Sets the resolution of proximity measurements
    @param  high_resolution
            Set to true to take 16-bit measurements for proximity,
            set to faluse to use 12-bit measurements.
*/
void VCNL36825T::setProximityHighResolution(bool high_resolution) {
  ps_conf3.hd = (uint8_t) high_resolution;
  writeRegister(VCNL36825T_REG__PS_CONF3, ps_conf3.rawData);
}

/*!
    @brief Writes a register with specified data
    @param reg Register to write to
    @param data Data to write
*/
void VCNL36825T::writeRegister(uint8_t reg, uint16_t data) {
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
uint16_t VCNL36825T::readRegister(uint8_t reg) {
  wire->beginTransmission(address);
	wire->write(reg);
	wire->endTransmission(false);

	wire->requestFrom(address, (uint8_t)2);

	uint8_t lsb = wire->read();
	uint8_t msb = wire->read();

	uint16_t val = msb << 8 | lsb;
  return val;
}