#include <Arduino.h>
#include <Wire.h>
#include <stm32g4xx.h>
#include <USBSerial.h>
#include <SerialCommands.h>

#include <list>
#include <memory>

#include "TemperatureSensor.h"
#include "VCNL4040.hpp"
#include "VCNL36825T.hpp"
#include "tools.h"

std::list<std::unique_ptr<TemperatureSensor>> temp_sensors;
std::unique_ptr<VCNL4040> light_sensor1;
std::unique_ptr<VCNL36825T> light_sensor2;
bool readTemperature;
bool readLight;
int readTemperatureSamples;
int currentTemperatureSample = 0;

void cmd_unrecognized(SerialCommands* sender, const char* cmd) {
  sender->GetSerial()->print("Unrecognized command [");
  sender->GetSerial()->print(cmd);
  sender->GetSerial()->println("]");
}

void reset(SerialCommands* sender) {
  readTemperature = false;
}

void readTemperature_on(SerialCommands* sender) {
  readTemperature = true;
}

void readTemperature_off(SerialCommands* sender) {
  readTemperature = false;
}

void readLight_on(SerialCommands* sender) {
  readLight = true;
}

void readTemperature_setSamples(SerialCommands* sender) {
  char* numSamples_str = sender->Next();
	if (numSamples_str == NULL) {
		sender->GetSerial()->println("ERROR NO_ID");
		return;
	}
  readTemperatureSamples = atoi(numSamples_str);
}

void readLight_off(SerialCommands* sender) {
  readLight = false;
}

void enumerate(SerialCommands* sender) {
  for (auto &&sensor : temp_sensors) {
    sensor->printInfo();
  }
  sender->GetSerial()->println("END");
}

void toggleTempSensor(SerialCommands* sender) {
	char* sensorId_str = sender->Next();
	if (sensorId_str == NULL) {
		sender->GetSerial()->println("ERROR NO_ID");
		return;
	}
  int sensorId = atoi(sensorId_str);

	char* offOn_str = sender->Next();
	if (offOn_str == NULL) {
		sender->GetSerial()->println("ERROR NO_ONOFF");
		return;
	}
  int offOn = atoi(offOn_str);

  bool foundSensor = false;
  for (auto &&sensor : temp_sensors) {
    if(sensor->getSensorId() == sensorId) {
      foundSensor = true;
      if(offOn == 0)      sensor->disable();
      else if(offOn == 1) sensor->enable();
      else                sender->GetSerial()->println("ERROR WRONG_ONOFF");
    }
  }
  if(!foundSensor){
    sender->GetSerial()->println("ERROR WRONG_ID");
  }
}

void toggleTempSensorAll(SerialCommands* sender) {
	char* offOn_str = sender->Next();
	if (offOn_str == NULL) {
		sender->GetSerial()->println("ERROR NO_ONOFF");
		return;
	}
  int offOn = atoi(offOn_str);

  for (auto &&sensor : temp_sensors) {
    if(offOn == 0)      sensor->disable();
    else if(offOn == 1) sensor->enable();
    else                sender->GetSerial()->println("ERROR WRONG_ONOFF");
  }
}

char serial_command_buffer_[48];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

SerialCommand cmd_listTempSensors("LIST_TEMP_SENSORS", enumerate);
SerialCommand cmd_toggleTempSensor("TOGGLE_TEMP_SENSOR", toggleTempSensor);
SerialCommand cmd_toggleTempSensorAll("TOGGLE_TEMP_SENSOR_ALL", toggleTempSensorAll);
SerialCommand cmd_readTemperature_on("READ_TEMP_ON", readTemperature_on);
SerialCommand cmd_readTemperature_off("READ_TEMP_OFF", readTemperature_off);
SerialCommand cmd_readTemperature_samples("READ_TEMP_SAMPLES", readTemperature_setSamples);
SerialCommand cmd_readLight_on("READ_LIGHT_ON", readLight_on);
SerialCommand cmd_readLight_off("READ_LIGHT_OFF", readLight_off);
SerialCommand cmd_reset("RESET", reset);

// Select the correct address setting
#define TMP117_ADDR_SDA 0x4A
#define SI7051_ADDR     0x40
#define ADT7422_ADDR    0x48
#define AS6221_ADDR     0x49

#define I2CSWITCH_ADDR      0x70
#define CHANNEL_VCNL36825T  0x01  // CH0
#define CHANNEL_VCNL4040    0x02  // CH1


TwoWire i2c_flex1 = TwoWire(PC9, PC8);
TwoWire i2c_flex2 = TwoWire(PB9, PB8);
TwoWire i2c_rigid = TwoWire(PA8, PA9);
// TwoWire i2c_rigid = TwoWire(PC9, PC8); //I2C3
// TwoWire i2c_flex1 = TwoWire(PA8, PA9); //I2C2
// TwoWire i2c_flex2 = TwoWire(PB9, PB8); //I2C1
// TwoWire i2c_temp = TwoWire(PA14, PA13); //I2C1


void setup() {

  // Initiate wire library and serial communication
  //Wire.begin();
  Serial.begin(1000000); // configured as USBSerial, baud rate irrelevant
  // while(!Serial.available()) {}
  Serial.println("OK");

  
  serial_commands_.SetDefaultHandler(cmd_unrecognized);
  serial_commands_.AddCommand(&cmd_readTemperature_on);
  serial_commands_.AddCommand(&cmd_readTemperature_off);
  serial_commands_.AddCommand(&cmd_readTemperature_samples);
  serial_commands_.AddCommand(&cmd_readLight_on);
  serial_commands_.AddCommand(&cmd_readLight_off);
  serial_commands_.AddCommand(&cmd_listTempSensors);
  serial_commands_.AddCommand(&cmd_toggleTempSensor);
  serial_commands_.AddCommand(&cmd_toggleTempSensorAll);
  serial_commands_.AddCommand(&cmd_reset);


  // enable flex PCB LDOs
  pinMode(PA5, OUTPUT); //upper
  pinMode(PA6, OUTPUT); //lower
  digitalWrite(PA5, HIGH);
  digitalWrite(PA6, HIGH);

  i2c_flex1.begin();
  i2c_flex2.begin();
  i2c_rigid.begin();

  delay(100);

  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new TMP117(&i2c_flex2, TMP117_ADDR_SDA), typeid(TMP117), "J2:TMP117", 0, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new Si7051(&i2c_flex2, SI7051_ADDR), typeid(Si7051), "J2:Si7051", 1, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new AS6221(&i2c_flex2, AS6221_ADDR), typeid(AS6221), "J2:AS6221", 2, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new ADT7422(&i2c_flex2, ADT7422_ADDR), typeid(ADT7422), "J2:ADT7422", 3, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new Si7051(&i2c_flex1, SI7051_ADDR), typeid(Si7051), "J1:Si7051", 4, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR), typeid(AS6221), "J1:AS6221", 5, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new ADT7422(&i2c_flex1, ADT7422_ADDR), typeid(ADT7422), "J1:ADT7422", 6, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new HDC1080JS(&i2c_rigid, 0x40), typeid(HDC1080JS), "EX:HDC1080JS", 7, true));
  temp_sensors.push_back(std::make_unique<TemperatureSensor>((void*) new SHT31(&i2c_rigid, 0x44), typeid(SHT31), "EX:SHT31", 8, true));

  for (auto &&sensor : temp_sensors){
    sensor->init();
  }

  // uint8_t i2cswitch_select = CHANNEL_VCNL36825T;
  // I2CTools::writeBytes(&i2c_rigid, I2CSWITCH_ADDR, &i2cswitch_select, 1);
  
  // delay(100);

  while(!Serial.available()) {}
  I2CTools::portScan(&i2c_rigid, 0x30, 0x70);

  light_sensor2 = std::make_unique<VCNL36825T>(&i2c_rigid, (uint8_t)0x60); 
  light_sensor2->init();

  // // light_sensor1 testing
  // light_sensor1 = std::make_unique<VCNL4040>(&i2c_rigid, (uint8_t)0x60); 
  // light_sensor1->init();
  // light_sensor1->setProximityLEDCurrent(VCNL4040_LEDCurrent::LED_CURRENT_200MA);
  // light_sensor1->setProximityIntegrationTime(VCNL4040_ProximityIntegration::PROXIMITY_INTEGRATION_TIME_8T);

  // // EXTERNAL LED CONTROL
  // uint32_t dac_val = 0;
  // while (true)
  // {
  //   dac_write_value(PA_4, dac_val, 1);
  //   Serial.println(dac_val);
  //   dac_val += 10;
  //   if(dac_val == 120){
  //     dac_val = 0;
  //   }
  //   delay(5000);
  // }
  
}

/************************* Infinite Loop Function **********************************/
void loop() {
  serial_commands_.ReadSerial();
  if (readTemperature || currentTemperatureSample < readTemperatureSamples) {
    for (auto &&sensor : temp_sensors){
      sensor->readValue();
    }
    if (currentTemperatureSample < readTemperatureSamples) 
      currentTemperatureSample += 1;
    if (currentTemperatureSample == readTemperatureSamples) {
      currentTemperatureSample = 0;
      readTemperatureSamples = 0;
    }
  }
  if (readLight) {
    // Serial.println(light_sensor1->getProximity());
    // Serial.println(light_sensor1->getAmbientLight());
    // Serial.println(light_sensor1->getLux());
    Serial.println(light_sensor2->getProximity());
    for (auto &&sensor : temp_sensors){
      if(sensor->getStatus()) {
        Serial.println(sensor->readValue(false));
      }
    }
  }
  // delay(250);
  delay(50);
}
