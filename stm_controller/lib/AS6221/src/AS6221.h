/************************************************************
  Malte Nilges
  AS6221 Library Source File
  Creation Date: 11/10/2023

  loosely based on
  Brandon Williams
  https://github.com/will2055/AS6221-Arduino-Library/src
  
  This file defines AS6221 core function definitions and
  initializers.
************************************************************/

#ifndef __AS6221_H__
#define __AS6221_H__

#include <Wire.h>
#include <Arduino.h>

#define AS6221_REG__TVAL 0x0
#define AS6221_REG__CONFIG 0x1
#define AS6221_REG__TLOW 0x2
#define AS6221_REG__THIGH 0x3

#define AS6221_CONFIG_DEFAULT 0x40A0

typedef union {
	uint16_t rawData;
	struct {
		uint8_t reserved0 : 5;
		uint8_t alert : 1;
		uint8_t conv_rate : 2;
		uint8_t sleep : 1;
		uint8_t interrupt : 2;
		uint8_t polarity : 1;
		uint8_t cfaults : 2;
		uint8_t reserved13 : 2;
    uint8_t singleshot : 1;
	};
} AS6221_ConfigurationRegister;

enum class AS6221_CONVRATE  {C4000ms = 0, C1000ms, C250ms, C125ms};                             //!<  Conversion Rate
enum class AS6221_SLEEP     {CONTINUOUS = 0, SLEEP};                                            //!<  INT/CT Mode (reset if readback or if within limits)
enum class AS6221_INTERRUPT {COMP_MODE = 0, INTERRUPT_MODE = 1};                                //!<  CT Polarity
enum class AS6221_POL       {ACTIVELOW = 0, ACTIVEHIGH = 1};                                    //!<  INT Polarity
enum class AS6221_CFAULTS   {NUMFAULTS1 = 0, NUMFAULTS2 = 1, NUMFAULTS3 = 2, NUMFAULTS4 = 3};   //!<  Fault Queue (num faults to trigger int/ct)
enum class AS6221_OPMODE    {NORMAL = 0, SINGLESHOT = 1};                                       //!<  Operating Mode

struct{
	uint8_t tlow_err_flag;
	uint8_t thigh_err_flag;
} ERROR_FLAGS;

class AS6221{
  public:
    AS6221(TwoWire* wire = &Wire, uint8_t address = 0x48);
    bool init();
    void setConversionRate(AS6221_CONVRATE rate);
    uint8_t getAddress();
    float readTemperature();
    uint16_t readConfig();
    void setConfig(uint16_t targetState);
    // bool setTLow(int16_t lowLimit, uint8_t *tlow_err_flag);
    // bool setTHigh(int16_t highLimit);
  private:
    TwoWire *_i2cPort;
    uint8_t _deviceAddress;
    AS6221_ConfigurationRegister config;
    uint16_t readRegister(uint8_t reg, uint8_t size);
    void writeRegister(uint8_t reg, int16_t data);
};

#endif
