#include <Arduino.h>
#include "ADT7422.h"
#include "AS6221.h"
#include "Si7051.h"
#include "TMP117.h"
#include "HDC1080JS.h"
#include "SHT31.h"

typedef union {
  float value;
  byte raw[4];
} binaryFloat;
typedef union {
  uint32_t value;
  byte raw[4];
} binaryUInt;

class TemperatureSensor {
  public:
    enum SensorType {
      SENSOR_MAX44009,
      SENSOR_SHT3X
    };
    TemperatureSensor(void* sensor, const std::type_info& type, const String& friendlyName, int sensorId, bool inUse = true);

    bool init();
    float readValue(bool writeToSerial = true);
    void* getSensor();
    void printInfo();
    void enable();
    void disable();
    int getSensorId();
    bool getStatus();
  private:
    void* sensorPtr;
    const std::type_info& sensorType;
    String friendlyName;
    int sensorId;
    bool inUse;
    
    void writeInfoToSerial();
		float readValue(ADT7422& sensor, bool writeToSerial);
		float readValue(AS6221& sensor, bool writeToSerial);
		float readValue(Si7051& sensor, bool writeToSerial);
		float readValue(TMP117& sensor, bool writeToSerial);
		float readValue(HDC1080JS& sensor, bool writeToSerial);
		float readValue(SHT31& sensor, bool writeToSerial);
};

