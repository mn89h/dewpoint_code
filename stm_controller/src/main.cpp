#include <Arduino.h>
#include <Wire.h>
#include <stm32g4xx.h>
#include <USBSerial.h>
#include <SerialCommands.h>

#include <list>
#include <memory>

#include "Sensor.h"
#include "VCNL4040.hpp"
#include "VCNL36825T.hpp"
#include "tools.h"
#include "PeltierController.h"


HardwareSerial Ser(PC0, PC1);
TwoWire i2c_flex1 = TwoWire(PB9, PB8);
TwoWire i2c_flex2 = TwoWire(PA8, PA9);
TwoWire i2c_rigid = TwoWire(PC9, PC8);

std::list<std::unique_ptr<Sensor>> sensors;
std::unique_ptr<PeltierController> peltier;
bool readTemperature;
bool readLight;
bool controlPeltier;
int readTemperatureSamples;
int currentTemperatureSample = 0;
uint16_t initial_light_value;


// Select the correct address setting
#define AS6221_ADDR_A     0x47
#define AS6221_ADDR_B     0x48
#define VCNL4040_ADDR     0x60
#define VCNL36825T_ADDR   0x60

#define I2CSWITCH_ADDR      0x70
#define CHANNEL_VCNL36825T  0x01  // CH0
#define CHANNEL_VCNL4040    0x02  // CH1



#define SENSOR_TEMP 0
#define SENSOR_PROX 1
#define SENSOR_HUM 2
#define SENSOR_AMB 3
#define SENSOR_CAP 4

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
  for (auto &&sensor : sensors) {
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
  for (auto &&sensor : sensors) {
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

  for (auto &&sensor : sensors) {
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

void setup() {

  // Initiate wire library and serial communication
  //Wire.begin();
  Serial.begin(1000000); // configured as USBSerial, baud rate irrelevant
  // while(!Serial.available()) {}
  Serial.println("OK");


  analogWriteFrequency(200000); // default PWM frequency is 1kHz, change it to 2kHz
  analogWrite(PB4, 0); // 127 means 50% duty cycle so a square wave
  
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

  i2c_flex1.begin();
  // i2c_flex2.begin();
  i2c_rigid.begin();

  delay(100);

  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F0", SENSOR_TEMP, 0, &i2c_flex1, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F1", SENSOR_TEMP, 1, &i2c_flex1, I2CSWITCH_ADDR, 1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F2", SENSOR_TEMP, 2, &i2c_flex1, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F3", SENSOR_TEMP, 3, &i2c_flex1, I2CSWITCH_ADDR, 3, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new CapacitorReadout(), typeid(CapacitorReadout), "Capacitor", SENSOR_CAP, 0, nullptr, 0, -1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_rigid, AS6221_ADDR_A), typeid(AS6221), "AS6221:B0", SENSOR_TEMP, 4, &i2c_rigid, I2CSWITCH_ADDR, -1, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:CENTER", SENSOR_PROX, 0, &i2c_rigid, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:BORDER", SENSOR_PROX, 1, &i2c_rigid, I2CSWITCH_ADDR, 1, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:BORDER", SENSOR_PROX, 2, &i2c_rigid, I2CSWITCH_ADDR, 2, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:CENTER", SENSOR_PROX, 3, &i2c_rigid, I2CSWITCH_ADDR, 3, false));

  for (auto &&sensor : sensors){
    if (sensor->getStatus()) {
      sensor->init();
    }
  }

  while(!Serial.available()) {}
  Serial.println("Base PCB I2C Devices");
  I2CTools::switchScan(&i2c_rigid, I2CSWITCH_ADDR, 4);
  Serial.println("Flex PCB I2C Devices");
  I2CTools::switchScan(&i2c_flex1, I2CSWITCH_ADDR, 4);

  // while(true){

  //   for (auto &&sensor : sensors){
  //     if (sensor->getStatus()) {
  //       if (sensor->getSensorCat() == SENSOR_TEMP) {
  //         Serial.print("TEMP");
  //       }
  //       if (sensor->getSensorCat() == SENSOR_PROX) {
  //         Serial.print("PROX");
  //       }
  //       if (sensor->getSensorCat() == SENSOR_CAP) {
  //         Serial.print("CAPF");
  //       }
  //       Serial.print(sensor->getSensorId());
  //       Serial.print(": ");
  //       Serial.println(sensor->readValue(false));
  //     }
  //   }
  //   delay(500);
  // }

  //TODO: PROX Sensor turn off, HUM/AMB sensors, peltier/heater pwm

  uint32_t period_start = millis();
  while(true) {
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        if (sensor->getSensorCat() == SENSOR_TEMP) {
          Serial.print("TEMP");
        }
        if (sensor->getSensorCat() == SENSOR_PROX) {
          Serial.print("PROX");
        }
        if (sensor->getSensorCat() == SENSOR_CAP) {
          Serial.print("CAPF");
        }
        Serial.print(sensor->getSensorId());
        Serial.print(": ");
        Serial.println(sensor->readValue(false));
      }
    }
    uint32_t period_end = millis();
    uint32_t elapsed = period_end - period_start;
    if(elapsed > 1000){
      analogWrite(PB4, 32); // 32 means 12.5%
    }
    if(elapsed > 40000){
      analogWrite(PB4, 0);
    }
    if(elapsed > 80000) {
      break;
    }
    delay(200);
  }


  // delay(10);
  // peltier = std::make_unique<PeltierController>(PinName(D3), PeltierController::ARDUINOPWM);
  // initial_light_value = (uint16_t) (0.9 * light_sensor2->getProximity());
  // Serial.print("Initial Light Value: ");
  // Serial.println(initial_light_value);
  // controlPeltier = true;
  // delay(10);

  // Ser.begin(19200);
  // delay(1);
  // Ser.println("*IDN?");
  // while (!Ser.available()){
  //   delay(500);
  //   Serial.println("wait");
  // }
  // String str = Serial.readString();
  // str.trim();
  // Serial.println(str);
}

#define PERIOD 250000    //us

/************************* Infinite Loop Function **********************************/
void loop() {
  serial_commands_.ReadSerial();
  uint32_t period_start = micros();
  
  uint32_t period_end = micros();
  uint32_t elapsed = period_end - period_start;
  if (elapsed < PERIOD) {
    delayMicroseconds(PERIOD - elapsed);
  }
  else {
    Serial.println("Missed timing");
  }
}
