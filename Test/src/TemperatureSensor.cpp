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
    return false;
}

float TemperatureSensor::readValue(bool writeToSerial) {
    
    if (sensorType == typeid(ADT7422)) return readValue(*(ADT7422*)sensorPtr, writeToSerial);
    if (sensorType == typeid(AS6221)) return readValue(*(AS6221*)sensorPtr, writeToSerial);
    if (sensorType == typeid(Si7051)) return readValue(*(Si7051*)sensorPtr, writeToSerial);
    if (sensorType == typeid(TMP117)) return readValue(*(TMP117*)sensorPtr, writeToSerial);
    return __FLT_MAX__;
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