#include <Arduino.h>
#include "Sensor.h"

Sensor::Sensor(void* sensor, const std::type_info& sensorType, const String &friendlyName, uint8_t sensorCat, uint8_t sensorId, TwoWire* wire, uint8_t i2c_switchAddress, int i2c_channel, bool inUse) :
    sensorPtr(sensor), sensorType(sensorType), friendlyName(friendlyName), sensorCat(sensorCat), sensorId(sensorId), wire(wire), i2c_switchAddress(i2c_switchAddress), i2c_channel(i2c_channel), inUse(inUse) {
}

bool Sensor::init() {
    if (i2c_channel != -1) I2CTools::switchSetChannel(wire, i2c_switchAddress, i2c_channel);
    if (sensorType == typeid(ADT7422)) return (*(ADT7422*)sensorPtr).init();
    if (sensorType == typeid(AS6221)) return (*(AS6221*)sensorPtr).init();
    if (sensorType == typeid(Si7051)) return (*(Si7051*)sensorPtr).init();
    if (sensorType == typeid(TMP117)) return (*(TMP117*)sensorPtr).init();
    if (sensorType == typeid(HDC1080JS)) return (*(HDC1080JS*)sensorPtr).init();
    if (sensorType == typeid(SHT31)) return (*(SHT31*)sensorPtr).init();
    if (sensorType == typeid(SHT31)) return (*(SHT31*)sensorPtr).init();
    if (sensorType == typeid(VCNL36825T)) return (*(VCNL36825T*)sensorPtr).init();
    if (sensorType == typeid(VCNL4040)) return (*(VCNL4040*)sensorPtr).init();
    if (sensorType == typeid(CapacitorReadout)) return (*(CapacitorReadout*)sensorPtr).init();
    if (sensorType == typeid(SHT4X)) return (*(SHT4X*)sensorPtr).init();
    if (sensorType == typeid(SHT45)) return (*(SHT45*)sensorPtr).init();
    if (sensorType == typeid(BME280Wrapper)) return (*(BME280Wrapper*)sensorPtr).init();
    return false;
}

float Sensor::readValue(bool writeToSerial, DataType type, bool receiveData) {
    if (!inUse) return __FLT_MAX__;
    if (i2c_channel != -1) I2CTools::switchSetChannel(wire, i2c_switchAddress, i2c_channel);
    if (sensorType == typeid(ADT7422)) return readValue(*(ADT7422*)sensorPtr, writeToSerial);
    if (sensorType == typeid(AS6221)) return readValue(*(AS6221*)sensorPtr, writeToSerial);
    if (sensorType == typeid(Si7051)) return readValue(*(Si7051*)sensorPtr, writeToSerial);
    if (sensorType == typeid(TMP117)) return readValue(*(TMP117*)sensorPtr, writeToSerial);
    if (sensorType == typeid(HDC1080JS)) return readValue(*(HDC1080JS*)sensorPtr, writeToSerial);
    if (sensorType == typeid(SHT31)) return readValue(*(SHT31*)sensorPtr, writeToSerial);
    if (sensorType == typeid(VCNL36825T)) return readValue(*(VCNL36825T*)sensorPtr, writeToSerial, receiveData);
    if (sensorType == typeid(VCNL4040)) return readValue(*(VCNL4040*)sensorPtr, writeToSerial, receiveData);
    if (sensorType == typeid(CapacitorReadout)) return readValue(*(CapacitorReadout*)sensorPtr, writeToSerial, receiveData);
    if (sensorType == typeid(SHT4X)) return readValue(*(SHT4X*)sensorPtr, writeToSerial, type, receiveData);
    if (sensorType == typeid(SHT45)) return readValue(*(SHT45*)sensorPtr, writeToSerial, type, receiveData);
    if (sensorType == typeid(BME280Wrapper)) return readValue(*(BME280Wrapper*)sensorPtr, writeToSerial, type, receiveData);
    return __FLT_MAX__;
}

bool Sensor::measure(bool asyncMode) {
    if (!inUse) return false;
    if (i2c_channel != -1) I2CTools::switchSetChannel(wire, i2c_switchAddress, i2c_channel);
    // // NOT YET IMPLEMENTED
    // if (sensorType == typeid(ADT7422)) return measure(*(ADT7422*)sensorPtr, asyncMode);
    // if (sensorType == typeid(AS6221)) return measure(*(AS6221*)sensorPtr, asyncMode);
    // if (sensorType == typeid(Si7051)) return measure(*(Si7051*)sensorPtr, asyncMode);
    // if (sensorType == typeid(TMP117)) return measure(*(TMP117*)sensorPtr, asyncMode);
    // if (sensorType == typeid(HDC1080JS)) return measure(*(HDC1080JS*)sensorPtr, asyncMode);
    // if (sensorType == typeid(SHT31)) return measure(*(SHT31*)sensorPtr, asyncMode);
    if (sensorType == typeid(VCNL36825T)) return measure(*(VCNL36825T*)sensorPtr, asyncMode);
    if (sensorType == typeid(VCNL4040)) return measure(*(VCNL4040*)sensorPtr, asyncMode);
    if (sensorType == typeid(CapacitorReadout)) return measure(*(CapacitorReadout*)sensorPtr, asyncMode);
    if (sensorType == typeid(SHT4X)) return measure(*(SHT4X*)sensorPtr, asyncMode);
    if (sensorType == typeid(SHT45)) return measure(*(SHT45*)sensorPtr, asyncMode);
    if (sensorType == typeid(BME280Wrapper)) return measure(*(BME280Wrapper*)sensorPtr, asyncMode);
    return false;
}

void Sensor::printInfo(){
    Serial.print("ID: ");
    Serial.print(sensorId);
    Serial.print(", NAME: ");
    Serial.print(friendlyName);
    Serial.print(", STATUS: ");
    if(inUse)   Serial.println("1");
    else        Serial.println("0");
}

void Sensor::enable(){
    inUse = true;
}

void Sensor::disable(){
    inUse = false;
}

uint8_t Sensor::getSensorCat() {
    return sensorCat;
}

uint8_t Sensor::getSensorId() {
    return sensorId;
}

bool Sensor::getStatus(){
    return inUse;
}

void Sensor::writeInfoToSerial(){    
    binaryUInt time;
    time.value = millis();

    Serial.write(&sensorCat, 1);
    Serial.write(&sensorId, 1);
    Serial.write(time.raw, 4);
}

float Sensor::readValue(ADT7422 &sensor, bool writeToSerial) {
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(AS6221 &sensor, bool writeToSerial) {
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(Si7051 &sensor, bool writeToSerial) {
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(TMP117 &sensor, bool writeToSerial) {
    binaryFloat reading;
    reading.value = sensor.readTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(HDC1080JS &sensor, bool writeToSerial) {
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
float Sensor::readValue(SHT31 &sensor, bool writeToSerial) {
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
float Sensor::readValue(VCNL36825T &sensor, bool writeToSerial, bool receiveData) {
    binaryFloat reading;
    reading.value = (float) sensor.getProximity();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(VCNL4040 &sensor, bool writeToSerial, bool receiveData) {
    binaryFloat reading;
    reading.value = (float) sensor.getProximity();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(CapacitorReadout &sensor, bool writeToSerial, bool receiveData) {
    binaryFloat reading;
    reading.value = (float) sensor.getFrequency();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(SHT4X &sensor, bool writeToSerial, DataType type, bool receiveData) {
    if (receiveData) sensor.receiveData();
    binaryFloat reading;
    reading.value = (float) sensor.getHumidity();
    if (type == DataType::TEMP) reading.value = (float) sensor.getTemperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(SHT45 &sensor, bool writeToSerial, DataType type, bool receiveData) {
    if (receiveData) sensor.receiveData();
    binaryFloat reading;
    reading.value = (float) sensor.humidity();
    //delay(10); //internal delay
    if (type == DataType::TEMP) reading.value = (float) sensor.temperature();

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}
float Sensor::readValue(BME280Wrapper &sensor, bool writeToSerial, DataType type, bool receiveData) {
    if (receiveData) sensor.receiveData();
    binaryFloat reading;
    reading.value = (float) sensor.getPressure();
    if (type == DataType::HUM) reading.value = (float) sensor.getHumidity() / 1024;
    if (type == DataType::TEMP) reading.value = (float) sensor.getTemperature() / 100; 

    if (writeToSerial) {
        writeInfoToSerial();
        Serial.write(reading.raw, 4);
    }
    return reading.value;
}


// bool Sensor::measure(ADT7422 &sensor, bool asyncMode) {
//     binaryFloat reading;
//     reading.value = sensor.readTemperature();

// }
// bool Sensor::measure(AS6221 &sensor, bool asyncMode) {
//     binaryFloat reading;
//     reading.value = sensor.readTemperature();

// }
// bool Sensor::measure(Si7051 &sensor, bool asyncMode) {
//     binaryFloat reading;
//     reading.value = sensor.readTemperature();

// }
// bool Sensor::measure(TMP117 &sensor, bool asyncMode) {
//     binaryFloat reading;
//     reading.value = sensor.readTemperature();

// }
// bool Sensor::measure(HDC1080JS &sensor, bool asyncMode) {
//     binaryFloat reading;
//     // data is ready only 15ms after that command
//     // -> readout from previous sampling
//     sensor.readTempHumid();
//     reading.value = sensor.getTemp();
//     sensor.requestMeas();

// }
// bool Sensor::measure(SHT31 &sensor, bool asyncMode) {
//     sensor.requestData();
// }
bool Sensor::measure(VCNL36825T &sensor, bool asyncMode) {
    return sensor.triggerSingle(asyncMode);
}
bool Sensor::measure(VCNL4040 &sensor, bool asyncMode) {
    return sensor.triggerSingle(asyncMode);
}
bool Sensor::measure(CapacitorReadout &sensor, bool asyncMode) {
    return sensor.measure(asyncMode);
}
bool Sensor::measure(SHT4X &sensor, bool asyncMode) {
    return sensor.measure(asyncMode);
}
bool Sensor::measure(SHT45 &sensor, bool asyncMode) {
    return sensor.measure(asyncMode);
}
bool Sensor::measure(BME280Wrapper &sensor, bool asyncMode) {
    return sensor.measure(asyncMode);
}
