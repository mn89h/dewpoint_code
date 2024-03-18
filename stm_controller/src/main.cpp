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
bool startHeaterRoutine;
uint32_t timelimitMS = 600000;
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
void startRoutineHeater(SerialCommands* sender) {
	startHeaterRoutine = true;
}
void setTimelimit(SerialCommands* sender) {
	char* tl_str = sender->Next();
	if (tl_str == NULL) {
		sender->GetSerial()->println("ERROR NO_ID");
		return;
	}
  timelimitMS = atoi(tl_str);
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
SerialCommand cmd_start2("S2", startRoutine2);
SerialCommand cmd_startHeater("HEAT", startRoutineHeater);
SerialCommand cmd_setTimelimit("TL", setTimelimit);


enum Direction {
  HOLD,
  ASCENDING,
  DESCENDING
};

template <typename T, size_t N>
constexpr String arrayToString(T (&arr)[N]) {
  String returnVal = "";
  for (int i = 0; i < N - 1; i++) {
    returnVal += (String) (arr[i]) + ";";
  }
  returnVal += (String) (arr[N - 1]);
  return returnVal;
}
String directionToString(Direction dir) {
  if (dir == HOLD) return "H";
  if (dir == ASCENDING) return "A";
  if (dir == DESCENDING) return "D";
  return "";
}

void setup() {

  // Initiate wire library and serial communication
  //Wire.begin();
  Serial.begin(1000000); // configured as USBSerial, baud rate irrelevant
  // while(!Serial.available()) {}
  Serial.println("OK");


  analogWriteFrequency(200000); // default PWM frequency is 1kHz, change it to 2kHz
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  digitalWrite(D5, LOW);
  digitalWrite(D6, LOW);
  // analogWrite(D5, 0); // 127 means 50% duty cycle so a square wave
  // analogWrite(D6, 0); // 127 means 50% duty cycle so a square wave
  
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
  serial_commands_.AddCommand(&cmd_startHeater);
  serial_commands_.AddCommand(&cmd_setTimelimit);

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
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_rigid, AS6221_ADDR_A), typeid(AS6221), "AS6221:B0", SENSOR_TEMP, 4, &i2c_rigid, I2CSWITCH_ADDR, -1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:CENTER", SENSOR_PROX, 0, &i2c_rigid, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(VCNL36825T), "VCNL36825T:BORDER", SENSOR_PROX, 1, &i2c_rigid, I2CSWITCH_ADDR, 1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:BORDER", SENSOR_PROX, 2, &i2c_rigid, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:CENTER", SENSOR_PROX, 3, &i2c_rigid, I2CSWITCH_ADDR, 3, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new SHT45(&i2c_flex1), typeid(SHT45), "SHT45:BORDER", SENSOR_HUM, 0, &i2c_flex1, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new SHT45(&i2c_flex1), typeid(SHT45), "SHT45:CENTER", SENSOR_HUM, 1, &i2c_flex1, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new BME280Wrapper(&i2c_flex1), typeid(BME280Wrapper), "BME280", SENSOR_AMB, 0, &i2c_rigid, I2CSWITCH_ADDR, 2, true));

  for (auto &&sensor : sensors){
    if (sensor->getStatus()) {
      sensor->init();
    }
  }
  delay(10);

  Serial.println("Base PCB I2C Devices");
  I2CTools::switchScan(&i2c_rigid, I2CSWITCH_ADDR, 4);
  Serial.println("Flex PCB I2C Devices");
  I2CTools::switchScan(&i2c_flex1, I2CSWITCH_ADDR, 4);

  while (!Serial.available()) {}
  Serial.println("initialized");
}



void routine_fall1rise1() {
  // Selfcheck
  // initiate measurements
  for (auto &&sensor : sensors){
    if (sensor->getStatus()) {
      if (sensor->getSensorCat() != SENSOR_CAP) {
        sensor->measure(true);
      }
    }
    delayMicroseconds(50);
  }
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
        if (reading <= 4000.0 || reading >= 60000.0) {
          self_check_failed = true;
          Serial.println("Prox failed! " + (String) reading + " " + sensor->getSensorId());
        }
      }
      else if (sensor->getSensorCat() == SENSOR_CAP) {
        sensor->measure(false);
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
  float deltaTperS = 0.0;
  float deltaTperPeriod = 0.0;
  float T_next = 0.0;
  float T_actual = __FLT_MAX__;
  float T_sensor[5];
  float humidity[3];
  float pressure;
  float proximity[4];
  float capacitance = 0.0;
  uint8_t numTempReadings = 0;

  float error = 0.0;
  float prev_error = 0.0;
  const float Kp = 20; // 27
  const float Ki = 5;
  const float Kd = 30;
  const float limit = 200.0;
  float pwm_factor = 0.0;
  float proportional = 0.0;
  float integrator = 0.0;
	float differentiator = 0.0;

  const uint8_t numStates = 7;
  // deltaTperS == 0 defines hold, please make sure proper holdTime is set at respective array index
  const float deltaTperSs[numStates] = {0, -0.8, 0, 1, 0, -0.1, 0};
  const float T_targets[numStates] = {0, 10, 10, 18.0, 18.0, 10, 10};
  const uint32_t holdTimes[numStates] = {1000, 0, 4000, 0, 500, 0, 4000};

  uint8_t state = 0;
  bool next_state = true;
  bool initialized = false;

  Direction direction = DESCENDING;

  const String info_head = "routine_name duration period PID T hum cap prox dTs T_targets holdTimes kp ki kd info";
  const String info = "main_fall1rise1 - 0.2s x x x x " + 
                      arrayToString(deltaTperSs) + " " +
                      arrayToString(T_targets) + " " +
                      arrayToString(holdTimes) + " " +
                      Kp + " " + Ki + " " + Kd + " " + 
                      "stability test (slow +dT)";
  Serial.println(info_head);
  Serial.println(info);
  const String cols = "Ttarget Tavg T0 T1 T2 T3 T4 H0 H1 H2 P2 P I D Error PID PROX0 PROX3 CAP";

  uint32_t time_checkpoint;
  bool finished = false;

  while(true) {
    uint32_t period_start = millis();

    // disable PWM for signal integrity
    analogWrite(D5, 0);
    delayMicroseconds(100);

    // initiate measurements
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_CAP) {
          sensor->measure(false);
          capacitance = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          sensor->measure(false);
        }
        else {
          sensor->measure(true);
        }
      }
      delayMicroseconds(50);
    }

    // wait for measurements to finish
    delay(10);

    // read current temperature and sensor values
    numTempReadings = 0;
    T_actual = 0;
    
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_TEMP && id < 4) {
          T_sensor[id] = sensor->readValue(false);
          T_actual += T_sensor[id];
          numTempReadings++;
        }
        if (sensor->getSensorCat() == SENSOR_TEMP && id == 4) {
          T_sensor[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_HUM) {
          humidity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX && (id == 0 || id == 3)) {
          proximity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_AMB) {
          humidity[id + 2] = sensor->readValue(false, Sensor::DataType::HUM);
          pressure = sensor->readValue(false, Sensor::DataType::PRESS, false);
        }
      }
    }
    T_actual = T_actual / numTempReadings;

    // initialize T_next and set time_checkpoint for use in hold states
    // also set direction and deltaTperPeriod (not really necessary but provides readability)
    if (next_state) {
      if (!initialized) {
        T_next = T_actual;
        initialized = true;
      }

      deltaTperS = deltaTperSs[state];
      deltaTperPeriod = deltaTperS * periodS;
      if (deltaTperPeriod < 0.0) {
        direction = DESCENDING;
      }
      else if (deltaTperPeriod > 0.0) {
        direction = ASCENDING;
      }
      else {
        direction = HOLD;
      }
      next_state = false;
      time_checkpoint = period_start;
    }

    // Calculate next target temperature
    T_next = T_next + deltaTperPeriod;

    // Limit T_next
    if ((direction == DESCENDING && T_next < T_targets[state]) ||
        (direction == ASCENDING && T_next > T_targets[state])) {
      T_next = T_targets[state];
    }

    // Check if T_target or holdTime has been reached and continue to next_state (or finish)
    if ((direction == DESCENDING && T_actual <= T_targets[state]) ||
        (direction == ASCENDING && T_actual >= T_targets[state]) ||
        (direction == HOLD && (period_start - time_checkpoint) >= holdTimes[state])) {
      state += 1;
      if (state < numStates) next_state = true;
      else finished = true;
    }

    // Break loop if execution is finished
    if (finished) {
      break;
    }

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
    analogWrite(D5, (uint8_t)pwm_factor);

    // Print status
    Serial.print(T_next + (String)" " + T_actual + " " + T_sensor[0] + " " + T_sensor[1] + " " + T_sensor[2] + " " + T_sensor[3] + " " + T_sensor[4] + " " + humidity[0] + " "  + humidity[1] + " "  + humidity[2] + " "  + pressure + " " + proportional + " " + integrator + " " + differentiator + " " + error + " " + pwm_factor + " " + proximity[0] + " " + proximity[3] + " " + capacitance);
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
  analogWrite(D5, 0);
}

bool shiftInsertAndCompare(float arr[], int n, float value, float compareValue) {
  bool allElementsLarger = true;

  // Copy values from original array, shifted right
  for(int i = n-1; i > 0; i--) {
    arr[i] = arr[i-1];
    if (arr[i] <= compareValue) {
      allElementsLarger = false;
    }
  }
  arr[0] = value;
  if (arr[0] <= compareValue) {
    allElementsLarger = false;
  }
  return allElementsLarger;
}

float shiftInsertAndAverage(float arr[], int n, float value, float lastAverage) {
  float average = lastAverage - (arr[n-1] / n);
  // Copy values from original array, shifted right
  for(int i = n-1; i > 0; i--) {
    arr[i] = arr[i-1];
  }
  arr[0] = value;
  average += (value / n);
  return average;
}



void routine_controlToCap() {
  const uint32_t periodMS = 200;
  const float periodS = (float) periodMS / 1000; 
  float deltaTperS = 0.0;
  float deltaTperPeriod = 0.0;
  float T_next = 0.0;
  float T_actual = __FLT_MAX__;
  float T_sensor[5];
  float humidity[3];
  float pressure;
  float proximity[4];
  float capacitance = 0.0;
  uint8_t numTempReadings = 0;

  float error = 0.0;
  float prev_error = 0.0;
  const float Kp = 20; // 27
  const float Ki = 5;
  const float Kd = 20;
  const float limit = 253.0;
  float pwm_factor = 0.0;
  float proportional = 0.0;
  float integrator = 0.0;
	float differentiator = 0.0;

  const float T_lowerLimOs = -25.0;
  const float T_upperLimOs = +1.5;
  const float C_lowerCtrlF = 0.825;
  const float C_upperCtrlF = 0.975;
  const uint8_t numStates = 15;
  // deltaTperS == 0 defines hold, please make sure proper holdTime is set at respective array index
  // const Direction directions[numStates] = {HOLD, DESCENDING_C, HOLD_DESCENDING, ASCENDING_T, HOLD_ASCENDING}
  const float deltaTperSs[numStates] = {0, -1.3, 0, 1, 0, -0.4, 0, 0.3, 0, -0.2, 0, 0.2, 0, -0.1, 0};
  const uint32_t holdTimes[numStates] = {4000, 0, 0, 0, 5000, 0, 2000, 0, 1000, 0, 2000, 0, 1000, 0, 4000};
  const uint32_t timeoutLim = 2000;

  float T_lowerLim;
  float T_upperLim;
  float C_lowerCtrl;
  float C_upperCtrl;

  uint8_t state = 0;
  bool next_state = true;
  bool initialized = false;

  Direction direction = DESCENDING;

  const String info_head = "routine_name duration period PID T hum cap prox dTs holdTimes kp ki kd info";
  const String info = "main_controlToCap - 0.2s x x x x " + 
                      arrayToString(deltaTperSs) + " " +
                      arrayToString(holdTimes) + " " +
                      Kp + " " + Ki + " " + Kd + " " + 
                      "stability test (slow +dT)";
  Serial.println(info_head);
  Serial.println(info);
  const String cols = "Ttarget Tavg T0 T1 T2 T3 T4 H0 H1 H2 P2 DIR P I D Error PID PROX0 PROX3 CAP";

  uint32_t time_checkpoint;
  bool timeout_check = false;
  bool finished = false;
  float pwm_factor_cooler = 0.0;
  float last_C;
  float average_dC;
  float min_dC;
  float min_dC_Temp = 1E6;
  float last_dCs[16] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0};
  float minC = 10000.0;
  uint32_t sampleCount = 0;
  uint32_t timeoutCount = 0;

  while(true) {
    uint32_t period_start = millis();

    // disable PWM for signal integrity
    analogWrite(D5, 0);
    analogWrite(D6, 0);
    delayMicroseconds(500);
    last_C = capacitance;

    // initiate measurements
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_CAP) {
          sensor->measure(false);
          capacitance = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          sensor->measure(false);
        }
        else {
          sensor->measure(true);
        }
      }
      delayMicroseconds(50);
    }

    // wait for measurements to finish
    delay(10);
    // analogWrite(D5, (uint8_t)pwm_factor_cooler);

    // // reduce rising capacitance threshold
    // if (capacitance < minC) {
    //   minC = capacitance;
    // }
    // bool risingTrend = shiftInsertAndCompare(lastCs, 5, capacitance, minC);
    // if (direction == ASCENDING && risingTrend && C_upperCtrl > capacitance) {
    //   C_upperCtrl = capacitance;
    // }

    // read current temperature and sensor values
    numTempReadings = 0;
    T_actual = 0;
    
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_TEMP && id < 4) {
          T_sensor[id] = sensor->readValue(false);
          T_actual += T_sensor[id];
          numTempReadings++;
        }
        if (sensor->getSensorCat() == SENSOR_TEMP && id == 4) {
          T_sensor[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_HUM) {
          humidity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX && (id == 0 || id == 3)) {
          proximity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_AMB) {
          humidity[id + 2] = sensor->readValue(false, Sensor::DataType::HUM);
          pressure = sensor->readValue(false, Sensor::DataType::PRESS, false);
        }
      }
    }
    sampleCount += 1;
    T_actual = T_actual / numTempReadings;

    // initialize T_next and set time_checkpoint for use in hold states
    // also set direction and deltaTperPeriod (not really necessary but provides readability)
    if (next_state) {
      if (!initialized) {
        T_next = T_actual;
        // T_lowerLim = T_actual + T_lowerLimOs;
        T_lowerLim = -10.0;
        T_upperLim = T_actual + T_upperLimOs;
        C_lowerCtrl = capacitance * C_lowerCtrlF;
        C_upperCtrl = capacitance * C_upperCtrlF;
        Serial.println((String)"Limits: " + T_lowerLim + " " + T_upperLim + " " + C_lowerCtrl + " " + C_upperCtrl);
        initialized = true;
        last_C = capacitance;
      }

      if (state == 2) {
        Serial.print("min_dC_Temp ");
        Serial.println(min_dC_Temp);
        float new_T_upperLim = min_dC_Temp + 10;
        if (new_T_upperLim < T_upperLim)
          T_upperLim = new_T_upperLim;
        T_lowerLim = min_dC_Temp - 1.5;
      }

      deltaTperS = deltaTperSs[state];
      deltaTperPeriod = deltaTperS * periodS;
      if (deltaTperPeriod < 0.0) {
        direction = DESCENDING;
      }
      else if (deltaTperPeriod > 0.0) {
        direction = ASCENDING;
      }
      else {
        direction = HOLD;
      }
      next_state = false;
      time_checkpoint = period_start;
      timeout_check = false;
    }

    // Calculate next target temperature
    T_next = T_next + deltaTperPeriod;

    // Limit T_next
    if (direction == DESCENDING && T_next < T_lowerLim) {
      T_next = T_lowerLim;
      time_checkpoint = period_start;
      timeout_check = true;
    }
    if (direction == ASCENDING && T_next > T_upperLim) {
      T_next = T_upperLim;
      time_checkpoint = period_start;
      timeout_check = true;
    }

    // Check if T_target or holdTime has been reached and continue to next_state (or finish)
    if ((direction == DESCENDING && capacitance < C_lowerCtrl) ||
        (direction == DESCENDING && capacitance < 5850.0) ||
        (direction == ASCENDING && capacitance > C_upperCtrl && capacitance > 6150.0) ||
        (direction == ASCENDING && capacitance > 6600.0) ||
        (direction == HOLD && (period_start - time_checkpoint) >= holdTimes[state])) {
      state += 1;
      next_state = true;
    }
    else if (timeout_check && (period_start - time_checkpoint) >= timeoutLim) {
      Serial.println("ms");
      state += 1;
      next_state = true;
    }
    // Break loop if execution is finished
    if (state >= numStates) {
      finished = true;
      break;
    }
    // // Remove usage of rise limit due to possible too early leave of ASCENDING state
    // if (capacitance < 6500.0) {
    //   C_riseLimit = 6500.0;
    // }

    // Calculate error between next target temperature and current temperature
    prev_error = error;
    error = T_next - T_actual;

    // Calculate PID parts using the error
    proportional = - (error * Kp);
    integrator = integrator - (0.5 * Kp * periodS * (error + prev_error));
    if (integrator > 170) integrator = 170;   // anti wind-up for integrator
    else if (integrator < 0) integrator = 0;
    differentiator = - (Kd * (error - prev_error) / periodS);

    // Turn PID parts into pulse width and set limit
    pwm_factor = proportional + integrator + differentiator;
    pwm_factor_cooler = pwm_factor;
    if (pwm_factor_cooler > limit) pwm_factor_cooler = limit;
    else if (pwm_factor_cooler < 0) pwm_factor_cooler = 0;
    if (capacitance < 3500) pwm_factor_cooler = 0;

    // Turn PWM back on using new value
    analogWrite(D5, (uint8_t)pwm_factor_cooler);

    // heat for faster condensation
    float pwm_factor_heater = 0.0;
    if (direction == ASCENDING) {
      pwm_factor_heater = 10 * pwm_factor;
      if (pwm_factor_heater < -150) pwm_factor_heater = 150;
      else if (pwm_factor_heater < 0) pwm_factor_heater = -pwm_factor_heater;
      else pwm_factor_heater = 0;
    }
    if (capacitance < 3500.0){
      analogWrite(D6, (uint8_t)150);
    }

    float dC = (capacitance - last_C); // removed division of step size (constant)
    if (state < 2) {
      if (sampleCount < 16) {
        average_dC = shiftInsertAndAverage(last_dCs, sampleCount + 1, dC, average_dC);
      }
      else {
        average_dC = shiftInsertAndAverage(last_dCs, 16, dC, average_dC);
        if (average_dC < min_dC && (capacitance >= 6150.0 || capacitance > C_upperCtrl)) {
          min_dC = average_dC;
          min_dC_Temp = T_actual;
        }
      }
    }

    // // not working yet, would need tuning
    // if (direction == ASCENDING && capacitance > 6250 && average_dC < 3.0) {
    //   timeoutCount += 1;
    // }
    // if (timeoutCount > 20) {
    //   next_state = true;
    //   state += 1;
    //   timeoutCount = 0;
    // }

    // Print status
    const String printString = 
        T_next + (String)" " + T_actual + " " + 
        T_sensor[0] + " " + T_sensor[1] + " " + T_sensor[2] + " " + T_sensor[3] + " " + T_sensor[4] + " " + 
        humidity[0] + " "  + humidity[1] + " "  + humidity[2] + " "  + pressure + " " + 
        directionToString(direction) + " " + proportional + " " + integrator + " " + differentiator + " " + error + " " + pwm_factor + " " + 
        proximity[0] + " " + proximity[3] + " " + capacitance; 
    Serial.println(printString);

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
  analogWrite(D5, 0);
  analogWrite(D6, 0);
}


void routine_nopid30min() {
  const uint32_t periodMS = 100;
  const float periodS = (float) periodMS / 1000; 
  float T_sensor[4];
  float T_actual = 0.0;
  float humidity[3];
  float pressure;
  float capacitance = 0.0;
  float proximity[4];
  uint8_t numTempReadings = 0;
  uint32_t runduration_ms = 30 * 60 * 1000; // min * s * us
  if (timelimitMS != 0) {
    runduration_ms = timelimitMS;
  }

  const String info_head = "routine_name duration period PID T hum cap prox T_target(s) kp ki kd info";
  const String info = "nopid30min 30min 0.2s - x x x - - - stability test (slow +dT)";
  Serial.println(info_head);
  Serial.println(info);

  uint32_t routine_start = millis();
  uint32_t next_period = routine_start;
  while(true) {
    next_period = next_period + 200;
    
    // initiate measurements
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_CAP) {
          sensor->measure(false);
          capacitance = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          sensor->measure(false);
        }
        else {
          sensor->measure(true);
        }
      }
      delayMicroseconds(50);
    }

    // wait for measurements to finish
    delay(30);

    // read current temperature and sensor values
    numTempReadings = 0;
    T_actual = 0;
    
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_TEMP && id < 4) {
          T_sensor[id] = sensor->readValue(false);
          T_actual += T_sensor[id];
          numTempReadings++;
        }
        else if (sensor->getSensorCat() == SENSOR_HUM) {
          humidity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          proximity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_AMB) {
          humidity[id + 2] = sensor->readValue(false, Sensor::DataType::HUM);
          pressure = sensor->readValue(false, Sensor::DataType::PRESS, false);
        }
      }
    }
    T_actual = T_actual / numTempReadings;

    // Print status
    Serial.print(T_actual + (String)" " + T_sensor[0] + " " + T_sensor[1] + " " + T_sensor[2] + " " + T_sensor[3] + " " + humidity[0] + " " + humidity[1] + " " + humidity[2] + " " + pressure + " " + proximity[0] + " " + proximity[1] + " " + proximity[2] + " " + proximity[3] + " " + capacitance);
    Serial.println();

    if (next_period > routine_start + runduration_ms) break;

    // Timing (use micros?)
    uint32_t period_end = millis();
    if (period_end < next_period) {
      delay(next_period - period_end);
    }
    else {
      Serial.println((String)"TIMING: " + ((int32_t) next_period - period_end));
    }
  }
}

void routine_pid30min() {
  const uint32_t periodMS = 100;
  const float periodS = (float) periodMS / 1000; 
  float T_sensor[4];
  float T_actual = 0.0;
  float T_next = 25.2;
  float humidity[3];
  float pressure;
  float capacitance = 0.0;
  float proximity[4];
  uint8_t numTempReadings = 0;
  uint32_t runduration_ms = 30 * 60 * 1000; // min * s * us
  if (timelimitMS != 0) {
    runduration_ms = timelimitMS;
  }

  
  float error = 0.0;
  float prev_error = 0.0;
  const float Kp = 20; // 27
  const float Ki = 5;
  const float Kd = 20;
  const float limit = 253.0;
  float pwm_factor = 0.0;
  float proportional = 0.0;
  float integrator = 0.0;
	float differentiator = 0.0;
  
  float pwm_factor_cooler = 0.0;

  const String info_head = "routine_name duration period PID T hum cap prox T_target(s) kp ki kd info";
  const String info = "nopid30min 30min 0.2s - x x x - - - stability test (slow +dT)";
  Serial.println(info_head);
  Serial.println(info);

  uint32_t routine_start = millis();
  uint32_t next_period = routine_start;
  while(true) {
    next_period = next_period + periodMS;
    
    // initiate measurements
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_CAP) {
          sensor->measure(false);
          capacitance = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          sensor->measure(false);
        }
        else {
          sensor->measure(true);
        }
      }
      delayMicroseconds(50);
    }

    // wait for measurements to finish
    delay(30);

    // read current temperature and sensor values
    numTempReadings = 0;
    T_actual = 0;
    
    for (auto &&sensor : sensors){
      if (sensor->getStatus()) {
        uint8_t id = sensor->getSensorId();
        if (sensor->getSensorCat() == SENSOR_TEMP && id < 4) {
          T_sensor[id] = sensor->readValue(false);
          T_actual += T_sensor[id];
          numTempReadings++;
        }
        else if (sensor->getSensorCat() == SENSOR_HUM) {
          humidity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_PROX) {
          proximity[id] = sensor->readValue(false);
        }
        else if (sensor->getSensorCat() == SENSOR_AMB) {
          humidity[id + 2] = sensor->readValue(false, Sensor::DataType::HUM);
          pressure = sensor->readValue(false, Sensor::DataType::PRESS, false);
        }
      }
    }
    T_actual = T_actual / numTempReadings;

    if (T_next > 15.0){
      T_next = T_next - 1 * periodS;
    }
    else {
      T_next = 15.0;
    }
    
    // Calculate error between next target temperature and current temperature
    prev_error = error;
    error = T_next - T_actual;

    // Calculate PID parts using the error
    proportional = - (error * Kp);
    integrator = integrator - (0.5 * Kp * periodS * (error + prev_error));
    if (integrator > 170) integrator = 170;   // anti wind-up for integrator
    else if (integrator < 0) integrator = 0;
    differentiator = - (Kd * (error - prev_error) / periodS);

    // Turn PID parts into pulse width and set limit
    pwm_factor = proportional + integrator + differentiator;
    pwm_factor_cooler = pwm_factor;
    if (pwm_factor_cooler > limit) pwm_factor_cooler = limit;
    else if (pwm_factor_cooler < 0) pwm_factor_cooler = 0;
    if (capacitance < 3500) pwm_factor_cooler = 0;

    // Turn PWM back on using new value
    analogWrite(D5, (uint8_t)pwm_factor_cooler);

    // Print status
    Serial.print(T_next + (String)" " + T_actual + " " + T_sensor[0] + " " + T_sensor[1] + " " + T_sensor[2] + " " + T_sensor[3] + " " + humidity[0] + " " + humidity[1] + " " + humidity[2] + " " + pressure + " " + proximity[0] + " " + proximity[1] + " " + proximity[2] + " " + proximity[3] + " " + capacitance);
    Serial.println();

    if (next_period > routine_start + runduration_ms) break;

    // Timing (use micros?)
    uint32_t period_end = millis();
    if (period_end < next_period) {
      delay(next_period - period_end);
    }
    else {
      Serial.println((String)"TIMING: " + ((int32_t) next_period - period_end));
    }
  }
}

void routine_heater() {
  analogWrite(D6, 100);
  delay(2000);
  analogWrite(D6, 0);
}

#define PERIOD 250000    //us

/************************* Infinite Loop Function **********************************/
void loop() {
  serial_commands_.ReadSerial();
  uint32_t period_start = micros();
  
  if (startPrimaryRoutine) {
    routine_controlToCap();
    startPrimaryRoutine = false;
  }
  if (startSecondaryRoutine) {
    routine_pid30min();
    startSecondaryRoutine = false;
  }
  if (startHeaterRoutine) {
    routine_heater();
    startHeaterRoutine = false;
  }

  uint32_t period_end = micros();
  uint32_t elapsed = period_end - period_start;
  if (elapsed < PERIOD) {
    delayMicroseconds(PERIOD - elapsed);
  }
}
