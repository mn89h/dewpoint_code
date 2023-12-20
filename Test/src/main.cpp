#include <Arduino.h>
#include <Wire.h>
#include <stm32g4xx.h>
#include <USBSerial.h>
#include <SerialCommands.h>

#include <list>
#include <memory>

#include "TemperatureSensor.h"
#include "VCNL4040.hpp"

std::list<std::unique_ptr<TemperatureSensor>> temp_sensors;
std::unique_ptr<VCNL4040> light_sensor;
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

char serial_command_buffer_[48];
SerialCommands serial_commands_(&Serial, serial_command_buffer_, sizeof(serial_command_buffer_), "\r\n", " ");

SerialCommand cmd_listTempSensors("LIST_TEMP_SENSORS", enumerate);
SerialCommand cmd_toggleTempSensor("TOGGLE_TEMP_SENSOR", toggleTempSensor);
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

// TwoWire i2c_flex1 = TwoWire(PC9, PC8);
// TwoWire i2c_flex2 = TwoWire(PB9, PB8);
// TwoWire i2c_rigid = TwoWire(PA8, PA9);
TwoWire i2c_rigid = TwoWire(PC9, PC8);
TwoWire i2c_flex1 = TwoWire(PA8, PA9);
TwoWire i2c_flex2 = TwoWire(PB9, PB8);


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

  for (auto &&sensor : temp_sensors){
    sensor->init();
  }

  while(!Serial.available()) {}
  light_sensor = std::make_unique<VCNL4040>(&i2c_rigid, (uint8_t)0x60); 
  i2c_rigid.beginTransmission(0x70);
  i2c_rigid.write(0x02);
  i2c_rigid.endTransmission();
  i2c_rigid.requestFrom(0x70, 2);

  light_sensor->init();
  light_sensor->setProximityLEDCurrent(VCNL4040_LEDCurrent::LED_CURRENT_200MA);
  light_sensor->setProximityIntegrationTime(VCNL4040_ProximityIntegration::PROXIMITY_INTEGRATION_TIME_8T);

  i2c_rigid.beginTransmission(0x60);
  i2c_rigid.write(0x03);
  i2c_rigid.endTransmission(false);
  i2c_rigid.requestFrom(0x60, 2);
  uint8_t lower = i2c_rigid.read();
  uint8_t higher = i2c_rigid.read();
  Serial.println(lower);
  Serial.println(higher);
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
    Serial.println(light_sensor->getProximity());
    Serial.println(light_sensor->getAmbientLight());
    // Serial.println(light_sensor->getLux());
  }
  delay(50);
}
