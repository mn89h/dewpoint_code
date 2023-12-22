#include "Wire.h"
#include "HDC1080JS.h"

HDC1080JS::HDC1080JS(TwoWire* wire, uint8_t address) : 
	_wire(wire), _address(address)
{
}

bool HDC1080JS::init() {
	config();
	delay(15);

	// Start one measurement
	_wire->beginTransmission(_address);
	_wire->write(0x00);
	_wire->endTransmission();
	delay(15);

	return true;
}

void HDC1080JS::config(){

	temperatureRaw=0;
	humidityRaw=0;

	//config the temp sensor to read temp then humidity in one transaction
	//config the resolution to 14 bits for temp & humidity

	writeRegister(0x02,0x10);

}

void HDC1080JS::writeRegister(uint8_t address, uint16_t value){
	_wire->beginTransmission(_address);
	_wire->write(address);
	_wire->write(value);
	_wire->endTransmission();
}



void HDC1080JS::requestMeas(){
	//set pointer register
	_wire->beginTransmission(_address);
	_wire->write(0x00);
	_wire->endTransmission();

}

void HDC1080JS::readTempHumid(){
	// delay(15);
	// maybe misses data for first readout if init not called
	_wire->requestFrom(_address, 4);
	temperatureRaw = temperatureRaw << 8 | _wire->read();
	temperatureRaw = temperatureRaw << 8 | _wire->read();
	humidityRaw = humidityRaw << 8 | _wire->read();
	humidityRaw = humidityRaw << 8 | _wire->read();

}

//returns temp in celcius
float HDC1080JS::getTemp(){

	// (rawTemp/2^16)*165 - 40
	return ( (float)temperatureRaw )*165/65536 - 40;

}

float HDC1080JS::getRelativeHumidity(){

	//(rawHumidity/2^16)*100
	return ( (float)humidityRaw )*100/65536;
}

float* HDC1080JS::getTempHumid(float* tempHumid){

	*tempHumid = getTemp();
	*(tempHumid+1) = getRelativeHumidity();
	return tempHumid;

}
