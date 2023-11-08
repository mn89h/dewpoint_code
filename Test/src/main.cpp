#include <Arduino.h>
#include <USBSerial.h>
#include <stm32g4xx.h>
#include <Wire.h>

// Select the correct address setting
uint8_t TMP117_ADDR_GND =  0x48;   // 1001000 
uint8_t TMP117_ADDR_VCC =  0x49;   // 1001001
uint8_t TMP117_ADDR_SDA =  0x4A;   // 1001010
uint8_t TMP117_ADDR_SCL =  0x4B;   // 1001011
uint8_t SI7051_ADDR     =  0x40;
uint8_t ADT7422_ADDR    =  0; //TODO
uint8_t AS6221_ADDR     =  0;

#include "TMP117.h"
#include "Si7051.h"

// #define ALERT_PIN               7     // low active alert pin
// #define LOW_TEMPERATURE_ALERT   20    // low alert at 20°C
// #define HIGH_TEMPERATURE_ALERT  28    // highalert at 28°C

TwoWire upper_i2c = TwoWire();
TwoWire lower_i2c = TwoWire();
TwoWire rigid_i2c= TwoWire();
TMP117 tmp117_upper(&upper_i2c, TMP117_ADDR_SDA);
TMP117 tmp117_lower(&lower_i2c, TMP117_ADDR_SDA);
Si7051 si7051_upper(&upper_i2c, SI7051_ADDR);



void setup() {

  // Initiate wire library and serial communication
  Wire.begin();
  Serial.begin(115200); // configured as USBSerial, baud rate irrelevant
  
  
  upper_i2c.setSCL(PC8);
  upper_i2c.setSDA(PC9);
  upper_i2c.begin();
  
  lower_i2c.setSCL(PC8);
  lower_i2c.setSDA(PC9);
  lower_i2c.begin();
  
  rigid_i2c.setSCL(PC8);
  rigid_i2c.setSDA(PC9);
  rigid_i2c.begin();


  tmp117_lower.init();
  tmp117_upper.init();     
}

/************************* Infinite Loop Function **********************************/
void loop() {
  tmp117_lower.getTemperature();
  
}
