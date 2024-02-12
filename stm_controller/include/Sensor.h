#include <Arduino.h>
#include "ADT7422.h"
#include "AS6221.h"
#include "Si7051.h"
#include "TMP117.h"
#include "HDC1080JS.h"
#include "SHT31.h"
#include "VCNL36825T.hpp"
#include "VCNL4040.hpp"


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
    Sensor(void* sensor, const std::type_info& type, const String& friendlyName, uint8_t sensorCat, uint8_t sensorId, TwoWire* wire = &Wire, uint8_t i2c_switchAddress, int i2c_channel = -1, bool inUse = true);

    bool init();
    float readValue(bool writeToSerial = true, DataType type = DataType::DFLT);
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
		float readValue(VCNL36825T& sensor, bool writeToSerial);
		float readValue(VCNL4040& sensor, bool writeToSerial);
};

