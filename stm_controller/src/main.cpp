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
#include "SHT4X.h"
#include "Bme280Wrapper.h"
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
bool startPrimaryRoutine;
bool startSecondaryRoutine;
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

void startRoutine(SerialCommands* sender) {
	startPrimaryRoutine = true;
}
void startRoutine2(SerialCommands* sender) {
	startSecondaryRoutine = true;
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
SerialCommand cmd_start1("START", startRoutine);
SerialCommand cmd_start2("START2", startRoutine2);

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
  serial_commands_.AddCommand(&cmd_start1);
  serial_commands_.AddCommand(&cmd_start2);

  i2c_flex1.begin();
  // i2c_flex2.begin();
  i2c_rigid.begin();

  delay(100);

  //IMPROV: sensor type as capability bit pattern
  sensors.push_back(std::make_unique<Sensor>((void*) new CapacitorReadout(), typeid(CapacitorReadout), "Capacitor", SENSOR_CAP, 0, nullptr, 0, -1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F0", SENSOR_TEMP, 0, &i2c_flex1, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F1", SENSOR_TEMP, 1, &i2c_flex1, I2CSWITCH_ADDR, 1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F2", SENSOR_TEMP, 2, &i2c_flex1, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F3", SENSOR_TEMP, 3, &i2c_flex1, I2CSWITCH_ADDR, 3, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_rigid, AS6221_ADDR_A), typeid(AS6221), "AS6221:B0", SENSOR_TEMP, 4, &i2c_rigid, I2CSWITCH_ADDR, -1, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:CENTER", SENSOR_PROX, 0, &i2c_rigid, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:BORDER", SENSOR_PROX, 1, &i2c_rigid, I2CSWITCH_ADDR, 1, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:BORDER", SENSOR_PROX, 2, &i2c_rigid, I2CSWITCH_ADDR, 2, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:CENTER", SENSOR_PROX, 3, &i2c_rigid, I2CSWITCH_ADDR, 3, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new SHT4X(&i2c_flex1), typeid(SHT4X), "SHT45:BORDER", SENSOR_HUM, 0, &i2c_flex1, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new SHT4X(&i2c_flex1), typeid(SHT4X), "SHT45:CENTER", SENSOR_HUM, 1, &i2c_flex1, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new BME280Wrapper(&i2c_flex1), typeid(BME280Wrapper), "BME280", SENSOR_AMB, 0, &i2c_rigid, I2CSWITCH_ADDR, 2, true));

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

  // Force PWM off
  analogWrite(PB4, 0);

  // uint32_t period_start = millis();
  // while(true) {
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
  //   uint32_t period_end = millis();
  //   uint32_t elapsed = period_end - period_start;
  //   if(elapsed > 1000){
  //     analogWrite(PB4, 32); // 32 means 12.5%
  //   }
  //   if(elapsed > 40000){
  //     analogWrite(PB4, 0);
  //   }
  //   if(elapsed > 80000) {
  //     break;
  //   }
  //   delay(200);
  // }


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

void routine_main() {
  // Selfcheck
  float reading;
  bool self_check_failed = false;
  for (auto &&sensor : sensors){
    if (sensor->getStatus()) {
      if (sensor->getSensorCat() == SENSOR_TEMP) {
        reading = sensor->readValue(false);
        if (reading > 50.0 || reading <= 0.0) {
          self_check_failed = true;
          Serial.println("Temp failed!");
        }
      }
      else if (sensor->getSensorCat() == SENSOR_PROX) {
        reading = sensor->readValue(false);
        if (reading <= 5000.0 || reading >= 15000.0) {
          self_check_failed = true;
          Serial.println("Prox failed!");
        }
      }
      else if (sensor->getSensorCat() == SENSOR_CAP) {
        reading = sensor->readValue(false);
        if (reading <= 5000.0 || reading >= 12000.0) {
          self_check_failed = true;
          Serial.println("Cap failed!");
        }
      }
    }
  }
  if (self_check_failed) {
    Serial.println("Selfcheck failed!");
    return;
  }

  const uint32_t periodMS = 200;
  const float periodS = (float) periodMS / 1000; 
  float limit = 200.0;
  float T_target = 0.0;
  float deltaTperS = 0.0;
  float deltaTperPeriod = 0.0;
  float T_next = 0.0;
  float T_actual = __FLT_MAX__;
  float humidity = 0.0;
  float proximity = 0.0;
  float capacitance = 0.0;
  uint8_t numTempReadings = 0;

  float error = 0.0;
  float prev_error = 0.0;
  float Kp = 20; // 27
  float Ki = 5;
  float Kd = 30;
  float pwm_factor = 0.0;
  float proportional = 0.0;
  float integrator = 0.0;
	float differentiator = 0.0;

  enum State {
    ENTRY,
    STAGE0_DESCEND,
    STAGE0_HOLD,
    STAGE1_ASCEND,
    STAGE1_HOLD
  };
  State state = ENTRY;

  uint32_t time_checkpoint;
  bool finished = false;

  while(true) {
    uint32_t period_start = millis();

    // disable PWM for signal integrity
    analogWrite(PB4, 0);
    delayMicroseconds(100);

    // read current temperature and sensor values
    numTempReadings = 0;
    T_actual = 0;
    humidity = 0;
    
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        if (sensor->getSensorCat() == SENSOR_TEMP) {
          T_actual += sensor->readValue(false);
          numTempReadings++;
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          proximity = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_CAP) {
          capacitance = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_HUM) {
          humidity += sensor->readValue(false);
        }
      }
    }
    humidity = humidity / 2;
    T_actual = T_actual / numTempReadings;

    // decide next state based on temperature or time
    switch (state) {
      case ENTRY : {
        state = STAGE0_DESCEND;
        T_target = 10.0;
        T_next = T_actual;
        deltaTperS = -0.6;
        deltaTperPeriod = deltaTperS * periodS;
        break;
      }
      case STAGE0_DESCEND : {
        if (T_next < T_target) T_next = T_target;
        if (T_actual <= T_target) {
          state = STAGE0_HOLD;
          deltaTperS = 0;
          deltaTperPeriod = deltaTperS * periodS;
          time_checkpoint = period_start;
        }
        break;
      }
      case STAGE0_HOLD : {
        if (period_start - time_checkpoint >= 3000) {
          state = STAGE1_ASCEND;
          T_target = 20.0;
          deltaTperS = 0.2;
          deltaTperPeriod = deltaTperS * periodS;
        }
      }
      case STAGE1_ASCEND : {
        if (T_next > T_target) T_next = T_target;
        if (T_actual >= T_target) {
          Serial.println("Finished");
          finished = true;
        }
      }
    }

    // Break loop if execution is finished
    if (finished) {
      break;
    }

    // Calculate next target temperature
    T_next = T_next + deltaTperPeriod;

    // Calculate error between next target temperature and current temperature
    prev_error = error;
    error = T_next - T_actual;

    // Calculate PID parts using the error
    proportional = - (error * Kp);
    integrator = integrator - (0.5 * Kp * periodS * (error + prev_error));
    if (integrator > 160) integrator = 160;   // anti wind-up for integrator
    else if (integrator < 0) integrator = 0;
    differentiator = - (Kd * (error - prev_error) / periodS);

    // Turn PID parts into pulse width and set limit
    pwm_factor = proportional + integrator + differentiator;
    if (pwm_factor > limit) pwm_factor = limit;
    else if (pwm_factor < 0) pwm_factor = 0;

    // Turn PWM back on using new value
    analogWrite(PB4, (uint8_t)pwm_factor);

    // Print status
    Serial.print(T_actual + (String)" " + T_next + " " + humidity + " " + proportional + " " + integrator + " " + differentiator + " " + error + " " + (uint8_t)pwm_factor + " " + proximity + " " + capacitance);
    Serial.println();

    // Timing (use micros?)
    uint32_t period_end = millis();
    uint32_t elapsed = period_end - period_start;
    if (elapsed < periodMS) {
      delay(periodMS - elapsed);
    }
    else {
      Serial.println((String)"TIMING: " + elapsed);
    }
  }

  // Force PWM off
  analogWrite(PB4, 0);
}



#define PERIOD 250000    //us

/************************* Infinite Loop Function **********************************/
void loop() {
  serial_commands_.ReadSerial();
  uint32_t period_start = micros();
  
  if (startPrimaryRoutine) {
    routine_main();
    startPrimaryRoutine = false;
  }
  if (startSecondaryRoutine) {
    startSecondaryRoutine = false;
  }

  uint32_t period_end = micros();
  uint32_t elapsed = period_end - period_start;
  if (elapsed < PERIOD) {
    delayMicroseconds(PERIOD - elapsed);
  }
}
