/*

Arduino Library for Silicon Labs Si7051 ±0.1°C (max) Digital Temperature Sensor
Written by AA for ClosedCube

---

The MIT License (MIT)

Copyright (c) 2016 ClosedCube Limited

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <Wire.h>
#include "Si7051.h"

/// @brief Constructor
/// @param wire I2C line
/// @param address I2C device address
Si7051::Si7051(TwoWire* wire, uint8_t address) {
	_address = address;
	_wire = wire;
}

/// @brief initializes the device by setting the resolution to 14 bits
bool Si7051::init() {
	setResolution(14);
	return true;
}

/// @brief resets all registers
void Si7051::reset() {
	_wire->beginTransmission(_address);
	_wire->write(0xFE);
	_wire->endTransmission();
}

/// @brief Read firmware version
/// @return firmware version
uint8_t Si7051::readFirmwareVersion() {
	_wire->beginTransmission(_address);
	_wire->write(0x84);
	_wire->write(0xB8);
	_wire->endTransmission();

	_wire->requestFrom(_address, (uint8_t)1);

	return _wire->read();
}

/// @brief Sets the resolution of the measurement
/// @param resolution resolution in bits, ranging from 11-14 bits
void Si7051::setResolution(uint8_t resolution) {
	SI7051_Register reg;

	switch (resolution) {
		case 14:
			reg.resolution0 = 0;
			reg.resolution7 = 0;
			break;
		case 13:
			reg.resolution0 = 0;
			reg.resolution7 = 1;
			break;
		case 12:
			reg.resolution0 = 1;
			reg.resolution7 = 0;
			break;	
		case 11:
			reg.resolution0 = 1;
			reg.resolution7 = 1;
			break;
	}
	
	_wire->beginTransmission(_address);
	_wire->write(0xE6);
	_wire->write(reg.rawData);
	_wire->endTransmission();
}

/// @brief Initiate a temperature measurement and read back the result.
/// @return temperatute in °C
float Si7051::readTemperature() {
	requestTemperatureDataBlocking();
	//delay(10);
	return receiveTemperatureData();
}

/// @brief Initiates a measurement w/o blocking I2C communication, if readout is performed before finish NACK is returned (SCK not hold)
void Si7051::requestTemperatureDataNonBlocking() {
	_wire->beginTransmission(_address);
	_wire->write(0xF3); //no hold master (NACK during measurement); use 0xE3 for holding SCK line
	_wire->endTransmission();
}

/// @brief Initiates a measurement w/ blocking I2C communication, SCK is stretched until measurement is complete
void Si7051::requestTemperatureDataBlocking() {
	_wire->beginTransmission(_address);
	_wire->write(0xE3); //no hold master (NACK during measurement); use 0xE3 for holding SCK line
	_wire->endTransmission();
}

/// @brief Read back the measured temperature.
/// @return temperature in °C
float Si7051::receiveTemperatureData() {
	_wire->requestFrom(_address, (uint8_t)2);

	uint8_t msb = _wire->read();
	uint8_t lsb = _wire->read();

	uint16_t val = msb << 8 | lsb;

	return (175.72*val) / 65536 - 46.85;
}

