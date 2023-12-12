#include <Arduino.h>
#include <Wire.h>
#include <stm32g4xx.h>
#include <USBSerial.h>

#include <list>
#include <memory>

#include "TemperatureSensor.h"

// Select the correct address setting
#define TMP117_ADDR_SDA 0x4A
#define SI7051_ADDR     0x40
#define ADT7422_ADDR    0x48
#define AS6221_ADDR     0x49

TwoWire i2c_flex1 = TwoWire(PC9, PC8);
TwoWire i2c_flex2 = TwoWire(PB9, PB8);
// TwoWire rigid_i2c= TwoWire();

std::list<std::unique_ptr<TemperatureSensor>> temp_sensors;

void setup() {

  // Initiate wire library and serial communication
  //Wire.begin();
  Serial.begin(115600); // configured as USBSerial, baud rate irrelevant
  while(!Serial.available()) {}
  Serial.println("OK");
  // enable flex PCB LDOs
  pinMode(PA5, OUTPUT); //upper
  pinMode(PA6, OUTPUT); //lower
  digitalWrite(PA5, HIGH);
  digitalWrite(PA6, HIGH);

  i2c_flex1.begin();
  i2c_flex2.begin();
  // rigid_i2c.begin();

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
}

/************************* Infinite Loop Function **********************************/
void loop() {
  Serial.println("A");

  for (auto &&sensor : temp_sensors){
    sensor->readValue();
  }
  delay(2000);
}
