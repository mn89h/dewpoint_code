#include <Arduino.h>
#include "TemperatureSensor.h"

TemperatureSensor::TemperatureSensor(void* sensor, const std::type_info& sensorType, const String &friendlyName, int sensorId, bool inUse) :
    sensorPtr(sensor), sensorType(sensorType), friendlyName(friendlyName), sensorId(sensorId), inUse(inUse)
{
}

bool TemperatureSensor::init() {
    if (sensorType == typeid(ADT7422)) return (*(ADT7422*)sensorPtr).init();
    if (sensorType == typeid(AS6221)) return (*(AS6221*)sensorPtr).init();
    if (sensorType == typeid(Si7051)) return (*(Si7051*)sensorPtr).init();
    if (sensorType == typeid(TMP117)) return (*(TMP117*)sensorPtr).init();
    if (sensorType == typeid(HDC1080JS)) return (*(HDC1080JS*)sensorPtr).init();
    if (sensorType == typeid(SHT31)) return (*(SHT31*)sensorPtr).init();
    return false;
}

float TemperatureSensor::readValue(bool writeToSerial) {
    if (!inUse) return __FLT_MAX__;
    if (sensorType == typeid(ADT7422)) return readValue(*(ADT7422*)sensorPtr, writeToSerial);
    if (sensorType == typeid(AS6221)) return readValue(*(AS6221*)sensorPtr, writeToSerial);
    if (sensorType == typeid(Si7051)) return readValue(*(Si7051*)sensorPtr, writeToSerial);
    if (sensorType == typeid(TMP117)) return readValue(*(TMP117*)sensorPtr, writeToSerial);
    if (sensorType == typeid(HDC1080JS)) return readValue(*(HDC1080JS*)sensorPtr, writeToSerial);
    if (sensorType == typeid(SHT31)) return readValue(*(SHT31*)sensorPtr, writeToSerial);
    return __FLT_MAX__;
}

void TemperatureSensor::printInfo(){
    Serial.print("ID: ");
    Serial.print(sensorId);
    Serial.print(", NAME: ");
    Serial.print(friendlyName);
    Serial.print(", STATUS: ");
    if(inUse)   Serial.println("1");
    else        Serial.println("0");
}

void TemperatureSensor::enable(){
    inUse = true;
}

void TemperatureSensor::disable(){
    inUse = false;
}

int TemperatureSensor::getSensorId() {
    return sensorId;
}

bool TemperatureSensor::getStatus(){
    return inUse;
}

void TemperatureSensor::writeInfoToSerial(){
    binaryUInt id;
    id.value = sensorId;
    
    binaryUInt time;
    time.value = millis();

    Serial.write(id.raw, 4);
    Serial.write(time.raw, 4);
}

float TemperatureSensor::readValue(ADT7422 &sensor, bool writeToSerial)
{
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float TemperatureSensor::readValue(AS6221 &sensor, bool writeToSerial)
{
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float TemperatureSensor::readValue(Si7051 &sensor, bool writeToSerial)
{
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float TemperatureSensor::readValue(TMP117 &sensor, bool writeToSerial)
{
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float TemperatureSensor::readValue(HDC1080JS &sensor, bool writeToSerial)
{
    binaryFloat reading;
    // data is ready only 15ms after that command
    // -> readout from previous sampling
    sensor.readTempHumid();
    reading.value = sensor.getTemp();
    sensor.requestMeas();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float TemperatureSensor::readValue(SHT31 &sensor, bool writeToSerial)
{
    binaryFloat reading;
    // data is ready only 15ms after that command
    // -> readout from previous sampling
    sensor.readData();
    reading.value = sensor.getTemperature(); 
    sensor.requestData();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}