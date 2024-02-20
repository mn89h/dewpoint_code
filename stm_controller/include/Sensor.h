#include <Arduino.h>
#include "ADT7422.h"
#include "AS6221.h"
#include "Si7051.h"
#include "TMP117.h"
#include "HDC1080JS.h"
#include "SHT31.h"
#include "VCNL36825T.hpp"
#include "VCNL4040.hpp"
#include "CapacitorReadout.h"
#include "SHT4X.h"
#include "SHT45.h"
#include "Bme280Wrapper.h"


#include "tools.h"

typedef union {
  float value;
  byte raw[4];
} binaryFloat;
typedef union {
  uint32_t value;
  byte raw[4];
} binaryUInt;
typedef union {
  uint16_t value;
  byte raw[2];
} binaryUShort;

class Sensor {
  public:
    // enum SensorType {
    //   SENSOR_MAX44009,
    //   SENSOR_SHT3X
    // };
    enum DataType {
      TEMP,
      HUM,
      PRESS,
      PROX,
      DFLT
    };
    Sensor(void* sensor, const std::type_info& type, const String& friendlyName, uint8_t sensorCat, uint8_t sensorId, TwoWire* wire = &Wire, uint8_t i2c_switchAddress = 0x70, int i2c_channel = -1, bool inUse = true);

    bool init();
    bool measure(bool asyncMode = true);
    float readValue(bool writeToSerial = true, DataType type = DataType::DFLT, bool receiveData = true);
    void* getSensor();
    void printInfo();
    void enable();
    void disable();
    uint8_t getSensorCat();
    uint8_t getSensorId();
    bool getStatus();
  private:
    void* sensorPtr;
    const std::type_info& sensorType;
    String friendlyName;
    uint8_t sensorCat;
    uint8_t sensorId;
    TwoWire* wire;
    uint8_t i2c_switchAddress;
    int i2c_channel;
    bool inUse;
    
    void writeInfoToSerial();
		float readValue(ADT7422& sensor, bool writeToSerial);
		float readValue(AS6221& sensor, bool writeToSerial);
		float readValue(Si7051& sensor, bool writeToSerial);
		float readValue(TMP117& sensor, bool writeToSerial);
		float readValue(HDC1080JS& sensor, bool writeToSerial);
		float readValue(SHT31& sensor, bool writeToSerial);
		float readValue(VCNL36825T& sensor, bool writeToSerial, bool receiveData);
		float readValue(VCNL4040& sensor, bool writeToSerial, bool receiveData);
		float readValue(CapacitorReadout& sensor, bool writeToSerial, bool receiveData);
		float readValue(SHT4X& sensor, bool writeToSerial, DataType type, bool receiveData);
		float readValue(SHT45& sensor, bool writeToSerial, DataType type, bool receiveData);
		float readValue(BME280Wrapper& sensor, bool writeToSerial, DataType type, bool receiveData);

    // // NOT YET IMPLEMENTED
		// bool measure(ADT7422& sensor, bool asyncMode);
		// bool measure(AS6221& sensor, bool asyncMode);
		// bool measure(Si7051& sensor, bool asyncMode);
		// bool measure(TMP117& sensor, bool asyncMode);
		// bool measure(HDC1080JS& sensor, bool asyncMode);
		// bool measure(SHT31& sensor, bool asyncMode);
		bool measure(VCNL36825T& sensor, bool asyncMode);
		bool measure(VCNL4040& sensor, bool asyncMode);
		bool measure(CapacitorReadout& sensor, bool asyncMode);
		bool measure(SHT4X& sensor, bool asyncMode);
		bool measure(SHT45& sensor, bool asyncMode);
		bool measure(BME280Wrapper& sensor, bool asyncMode);
};

