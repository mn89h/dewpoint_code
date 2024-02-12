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

std::list<std::unique_ptr<Sensor>> sensors;
std::unique_ptr<VCNL4040> light_sensor1;
std::unique_ptr<VCNL36825T> light_sensor2;
std::unique_ptr<PeltierController> peltier;
bool readTemperature;
bool readLight;
bool controlPeltier;
int readTemperatureSamples;
int currentTemperatureSample = 0;
uint16_t initial_light_value;

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

// Select the correct address setting
#define AS6221_ADDR_A     0x47
#define AS6221_ADDR_B     0x48
#define VCNL4040_ADDR     0x60
#define VCNL36825T_ADDR   0x60

#define I2CSWITCH_ADDR      0x70
#define CHANNEL_VCNL36825T  0x01  // CH0
#define CHANNEL_VCNL4040    0x02  // CH1


TwoWire i2c_flex1 = TwoWire(PB9, PB8);
TwoWire i2c_flex2 = TwoWire(PA8, PA9);
TwoWire i2c_rigid = TwoWire(PC9, PC8);

#define SENSOR_TEMP 0
#define SENSOR_PROX 1
#define SENSOR_HUM 2
#define SENSOR_AMB 3

uint32_t channelRising;
float FrequencyMeasured;
volatile uint8_t counter_state;
volatile uint32_t LastPeriodCapture = 0, CurrentCapture, Overflowed;
uint32_t input_freq = 0;
volatile uint32_t rolloverCompareCount = 0;
HardwareTimer *MyTim;

// /**
//     @brief  Input capture interrupt callback : Compute frequency and dutycycle of input signal
// */
// void TIMINPUT_Capture_Rising_IT_callback(void) {
//   CurrentCapture = MyTim->getCaptureCompare(channelRising);
//   /* frequency computation */
//   if (CurrentCapture > LastPeriodCapture){
//     FrequencyMeasured = input_freq / (CurrentCapture - LastPeriodCapture);
//   }
//   else if (CurrentCapture <= LastPeriodCapture){
//     /* 0x1000 is max overflow value */
//     FrequencyMeasured = input_freq / (0x10000 + CurrentCapture - LastPeriodCapture);
//   }

//   LastPeriodCapture = CurrentCapture;
//   rolloverCompareCount = 0;
// }
void TIMINPUT_Capture_Rising_IT_callback(void) {
  Overflowed = rolloverCompareCount;
  uint32_t current_count = MyTim->getCaptureCompare(channelRising);
  if (counter_state == 0) {
    LastPeriodCapture = current_count;
    rolloverCompareCount = 0;
    counter_state += 1;
  }
  else {
    MyTim->pause();
    CurrentCapture = current_count;
    counter_state = 0;
  }
}

/* In case of timer rollover, frequency is to low to be measured set values to 0
   To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision. */
void Rollover_IT_callback(void) {
  rolloverCompareCount++;
}

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

  i2c_flex1.begin();
  // i2c_flex2.begin();
  i2c_rigid.begin();

  delay(100);

  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F0", SENSOR_TEMP, 0, &i2c_flex1, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F1", SENSOR_TEMP, 1, &i2c_flex1, I2CSWITCH_ADDR, 1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_B), typeid(AS6221), "AS6221:F2", SENSOR_TEMP, 2, &i2c_flex1, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_flex1, AS6221_ADDR_A), typeid(AS6221), "AS6221:F3", SENSOR_TEMP, 3, &i2c_flex1, I2CSWITCH_ADDR, 3, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new AS6221(&i2c_rigid, AS6221_ADDR_A), typeid(AS6221), "AS6221:B0", SENSOR_TEMP, 4, &i2c_rigid, I2CSWITCH_ADDR, -1, false));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(AS6221), "VCNL36825T:CENTER", SENSOR_PROX, 0, &i2c_rigid, I2CSWITCH_ADDR, 0, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL36825T(&i2c_rigid, VCNL36825T_ADDR), typeid(AS6221), "VCNL36825T:BORDER", SENSOR_PROX, 1, &i2c_rigid, I2CSWITCH_ADDR, 1, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:BORDER", SENSOR_PROX, 2, &i2c_rigid, I2CSWITCH_ADDR, 2, true));
  sensors.push_back(std::make_unique<Sensor>((void*) new VCNL4040(&i2c_rigid, VCNL4040_ADDR), typeid(VCNL4040), "VCNL4040:CENTER", SENSOR_PROX, 3, &i2c_rigid, I2CSWITCH_ADDR, 3, true));

  for (auto &&sensor : sensors){
    sensor->init();
  }

  while(!Serial.available()) {}
  Serial.println("Base PCB I2C Devices");
  I2CTools::switchScan(&i2c_rigid, I2CSWITCH_ADDR, 4);
  Serial.println("Flex PCB I2C Devices");
  I2CTools::switchScan(&i2c_flex1, I2CSWITCH_ADDR, 4);

  for (auto &&sensor : sensors){
    if (sensor->getStatus()) {
      if (sensor->getSensorCat() == SENSOR_TEMP) {
        Serial.print("TEMP");
      }
      if (sensor->getSensorCat() == SENSOR_PROX) {
        Serial.print("PROX");
      }
      Serial.print(sensor->getSensorId());
      Serial.print(": ");
      Serial.println(sensor->readValue(false));
    }
  }

  //TODO: PROX Sensor turn off, HUM/AMB sensors, peltier/heater pwm, cap readout

  // uint32_t pwmPin_Peltier = 5;
  // TIM_TypeDef *pwmInst_Peltier = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmPin_Peltier), PinMap_PWM);
  // uint32_t pwmChannel_Peltier = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmPin_Peltier), PinMap_PWM));
  // HardwareTimer *Timer_Peltier = new HardwareTimer(pwmInst_Peltier);

  // uint32_t pwmPin_Heater = 6;
  // TIM_TypeDef *pwmInst_Heater = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pwmPin_Heater), PinMap_PWM);
  // uint32_t pwmChannel_Heater = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pwmPin_Heater), PinMap_PWM));
  // HardwareTimer *Timer_Heater = new HardwareTimer(pwmInst_Heater);

  // uint32_t percent = 50;

  // while(true){
  //   Serial.println(percent);
  //   Timer_Peltier->pause();
  //   Timer_Peltier->setPWM(pwmChannel_Peltier, pwmPin_Peltier, 200000, percent); // 5 Hertz, 10% dutycycle
  //   Timer_Peltier->resume();
  //   Timer_Heater->pause();
  //   Timer_Heater->setPWM(pwmChannel_Heater, pwmPin_Heater, 200000, 100-percent); // 5 Hertz, 10% dutycycle
  //   Timer_Heater->resume();
  //   percent += 00;
  //   if (percent > 100) percent = 0;
  //   delay(500);
  // }

  uint32_t pin = 2;
  // Automatically retrieve TIM instance and channelRising associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(pin), PinMap_PWM);
  channelRising = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin), PinMap_PWM));

  // Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
  MyTim = new HardwareTimer(Instance);

  // Configure rising edge detection to measure frequency
  MyTim->setMode(channelRising, TIMER_INPUT_FREQ_DUTY_MEASUREMENT, pin);

  // With a PrescalerFactor = 1, the minimum frequency value to measure is : TIM counter clock / CCR MAX
  //  = (SystemCoreClock) / 65535
  // Example on Nucleo_L476RG with systemClock at 80MHz, the minimum frequency is around 1,2 khz
  // To reduce minimum frequency, it is possible to increase prescaler. But this is at a cost of precision.
  // The maximum frequency depends on processing of both interruptions and thus depend on board used
  // Example on Nucleo_L476RG with systemClock at 80MHz the interruptions processing is around 10 microseconds and thus Max frequency is around 100kHz
  uint32_t PrescalerFactor = 1;
  MyTim->setPrescaleFactor(PrescalerFactor);
  MyTim->setOverflow(0x10000); // Max Period value to have the largest possible time to detect rising edge and avoid timer rollover
  MyTim->attachInterrupt(channelRising, TIMINPUT_Capture_Rising_IT_callback);
  MyTim->attachInterrupt(Rollover_IT_callback);

  // Compute this scale factor only once: G474RE, Prescaler = 2: input_freq = 84 MHz
  // maximum period: 65536/84MHz=780us -> fmin = 1.28
  input_freq = MyTim->getTimerClkFreq() / MyTim->getPrescaleFactor();
  Serial.println(input_freq);
  delay(100);


  I2CTools::switchSetChannel(&i2c_flex1, I2CSWITCH_ADDR, 3);
  AS6221 as4(&i2c_flex1, 0x47);
  as4.init();
  delay(100);
  Serial.println(as4.readTemperature());
  
  
  analogWriteFrequency(20000); // default PWM frequency is 1kHz, change it to 2kHz
  
  uint32_t period_start = millis();
  while(true) {
    MyTim->resume();
    delay(2);
    MyTim->pause(); //force pause if no second interrupt after 2ms
    int32_t period_in_ticks = Overflowed * 0x10000 + CurrentCapture - LastPeriodCapture;
    FrequencyMeasured = (float) input_freq / period_in_ticks;

    Serial.print(as4.readTemperature());
    Serial.print(" ");
    Serial.print(rolloverCompareCount);
    Serial.print(" ");
    Serial.print(period_in_ticks);
    Serial.print(" ");
    Serial.println(FrequencyMeasured);
    uint32_t period_end = millis();
    uint32_t elapsed = period_end - period_start;
    if(elapsed > 1000){
      analogWrite(6, 64); // 32 means 12.5%
    }
    if(elapsed > 16000){
      analogWrite(6, 0);
    }
    if(elapsed > 40000) {
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
