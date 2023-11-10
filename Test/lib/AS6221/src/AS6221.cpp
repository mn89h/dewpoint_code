/************************************************************
  Malte Nilges
  AS6221 Library Source File
  Creation Date: 11/10/2023

  loosely based on
  Brandon Williams
  https://github.com/will2055/AS6221-Arduino-Library/src
************************************************************/

/*
  NOTE: Read for use for the most accurate readings from the sensor
  - Avoid heavy bypass traffic on the I2C bus for most accurate temperature readings
  - Use the highest available communication speeds
  - Use the minimal supply voltage acceptable for the system
*/

#include <Arduino.h>
#include <Wire.h>
#include "AS6221.h"

AS6221::AS6221(uint8_t address, TwoWire *wire){
  _i2cPort = wire;
  _deviceAddress = address;
  config.rawData = AS6221_CONFIG_DEFAULT;
}

void AS6221::init(){
  setConversionRate(AS6221_CONVRATE::C125ms);
}

void AS6221::setConversionRate(AS6221_CONVRATE rate) {
  config.conv_rate = (uint8_t) rate;
  setConfig(config.rawData);
}

uint8_t AS6221::getAddress() {
  return _deviceAddress;
}

uint16_t AS6221::readRegister(uint8_t reg, uint8_t size){

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg);
  _i2cPort->endTransmission();
  
  _i2cPort->requestFrom(_deviceAddress, size);
  
  uint8_t dataBuffer[size];
  
  int16_t datac = 0;

  if(_i2cPort->available() <= 2){
    for(size_t i = 0; i < size; i++) dataBuffer[i] = _i2cPort->read();
  }
  
  datac = ((dataBuffer[0] << 8) | dataBuffer[1]);

  return datac;
}

void AS6221::writeRegister(uint8_t reg, int16_t data){

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg);
  _i2cPort->write(highByte(data));
  _i2cPort->write(lowByte(data));
  _i2cPort->endTransmission();

}

float AS6221::readTemperature(){
  int16_t digitalTempC = (int16_t) readRegister(AS6221_REG__TVAL,2); // int16_t cast required

  float finalTempC;
  finalTempC = digitalTempC * 0.0078125;

  return finalTempC;
}


/*
 * Sets TLow Threshold, if temp drops below the threshold then 
 * interrupt is triggered. Pointer to error flag to for custom
 * error reporting.
 */
/* NOT IMPLEMENTED
bool AS6221::setTLow(int16_t lowLimit){        

  if(lowLimit < getTHigh() && lowLimit != getTLow()){
    int16_t lowTemp = lowLimit / 0.0078125;
    writeRegister(TLOW, lowTemp);
    return true;
  }
  return false;
}

bool AS6221::setTHigh(int16_t highLimit){
	
    if(highLimit > getTLow()){
		int16_t highTemp = highLimit / 0.0078125;
		writeRegister(THIGH, highTemp);
		return true;
  }
  
  else{
	  //Serial.println("Value is below the Low Temperature Threshold.\n Please choose a different value.");
	  return false;
  }
}
*/

uint16_t AS6221::readConfig(){
		return readRegister(AS6221_REG__CONFIG,2);
}

void AS6221::setConfig(uint16_t targetState){
		writeRegister(AS6221_REG__CONFIG, targetState);
}

//Sleep Single-Shot mode (0xC1A0) returns odd register value (FFFFC1A0)
