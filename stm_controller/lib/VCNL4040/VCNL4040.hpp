/*!
 *  @file VCNL4040.hpp
 *
 * 	I2C Driver for VCNL4040 proximity and ambient light sensor
 *
 *	BSD license (see license.txt)
 */

#ifndef _VCNL4040_HPP
#define _VCNL4040_HPP

#include "Arduino.h"
#include <Wire.h>
#define VCNL4040_I2CADDR_DEFAULT 0x60 ///< VCNL4040 default i2c address

// All addresses are for 16bit registers;
// duplicates are for high or low bytes that aren't used together
#define VCNL4040_REG__ALS_CONF1 0x00     ///< Ambient light sensor configuration register
#define VCNL4040_REG__ALS_THDH 0x01      ///< Ambient light high threshold register
#define VCNL4040_REG__ALS_THDL 0x02      ///< Ambient light low threshold register
#define VCNL4040_REG__PS_CONF1_2 0x03    ///< Proximity sensor configuration 1/2 register
#define VCNL4040_REG__PS_CONF3_MS 0x04   ///< Proximity sensor configuration 3/MS register
#define VCNL4040_REG__PS_THDL 0x06       ///< Proximity sensor low threshold register
#define VCNL4040_REG__PS_THDH 0x07       ///< Proximity sensor high threshold register
#define VCNL4040_REG__PS_DATA 0x08       ///< Proximity sensor data register
#define VCNL4040_REG__ALS_DATA 0x09      ///< Ambient light sensor data register
#define VCNL4040_REG__WHITE_DATA 0x0A    ///< White light sensor data register
#define VCNL4040_REG__INT_FLAG 0x0B      ///< Interrupt status register
#define VCNL4040_REG__DEVICE_ID 0x0C     ///< Device ID


typedef union {
	uint16_t rawData;
	struct {
		uint8_t sd : 1;
    uint8_t int_en : 1;
    uint8_t pers : 2;
    uint8_t reserved1 : 2;
    uint8_t it : 2;
    uint8_t reserved2 : 8;
	};
} VCNL4040_ALSConfReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdh_lo : 8;
    uint8_t thdh_hi : 8;
	};
} VCNL4040_ALSThdHighReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdl_lo : 8;
    uint8_t thdl_hi : 8;
	};
} VCNL4040_ALSThdLowReg;

typedef union {
	uint16_t rawData;
	struct {
    uint8_t sd : 1;
    uint8_t it : 3;
    uint8_t pers : 2;
    uint8_t duty : 2;
    uint8_t intr : 2;
    uint8_t reserved1 : 1;
    uint8_t hd :  1;
    uint8_t reserved2 : 4;
	};
} VCNL4040_PSConf12Reg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t sc_en : 1;
    uint8_t reserved1 : 1;
    uint8_t trig : 1;
    uint8_t af : 1;
    uint8_t smart_pers : 1;
    uint8_t mps : 2;
    uint8_t reserved2 : 1;
    uint8_t led_i : 3;
    uint8_t reserved3 : 3;
    uint8_t ms : 1;
    uint8_t white_en : 1;
	};
} VCNL4040_PSConf3MSReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t canc_lo : 8;
    uint8_t canc_hi : 8;
	};
} VCNL4040_PSCancReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdh_lo : 8;
    uint8_t thdh_hi : 8;
	};
} VCNL4040_PSThdHighReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdl_lo : 8;
    uint8_t thdl_hi : 8;
	};
} VCNL4040_PSThdLowReg;


enum class VCNL4040_LEDDutyCycle { LED_DUTY_1_40, LED_DUTY_1_80, LED_DUTY_1_160, LED_DUTY_1_320 };
enum class VCNL4040_LEDCurrent {
  LED_CURRENT_50MA,
  LED_CURRENT_75MA,
  LED_CURRENT_100MA,
  LED_CURRENT_120MA,
  LED_CURRENT_140MA,
  LED_CURRENT_160MA,
  LED_CURRENT_180MA,
  LED_CURRENT_200MA,
};

enum class VCNL4040_AmbientIntegration {
  AMBIENT_INTEGRATION_TIME_80MS = 0,
  AMBIENT_INTEGRATION_TIME_160MS,
  AMBIENT_INTEGRATION_TIME_320MS,
  AMBIENT_INTEGRATION_TIME_640MS,
};
enum class VCNL4040_ProximityIntegration {
  PROXIMITY_INTEGRATION_TIME_1T = 0,
  PROXIMITY_INTEGRATION_TIME_1T5,
  PROXIMITY_INTEGRATION_TIME_2T,
  PROXIMITY_INTEGRATION_TIME_2T5,
  PROXIMITY_INTEGRATION_TIME_3T,
  PROXIMITY_INTEGRATION_TIME_3T5,
  PROXIMITY_INTEGRATION_TIME_4T,
  PROXIMITY_INTEGRATION_TIME_8T,
};
enum class VCNL4040_ProximityType { PROXIMITY_INT_DISABLE, PROXIMITY_INT_CLOSE, PROXIMITY_INT_AWAY, PROXIMITY_INT_CLOSE_AWAY };
enum class VCNL4040_InterruptType { PROXIMITY_AWAY, PROXIMITY_CLOSE, AMBIENT_HIGH = 4, AMBIENT_LOW };

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the VCNL4040 I2C Digital Potentiometer
 */
class VCNL4040 {
public:
  VCNL4040(TwoWire *wire, uint8_t i2c_addr = VCNL4040_I2CADDR_DEFAULT);
  bool init();
  uint16_t getProximity();
  uint16_t getAmbientLight();
  uint16_t getWhiteLight();
  float getLux();

  void enableProximity(bool enable);
  void enableAmbientLight(bool enable);
  void enableWhiteLight(bool enable);
  void enableForceMode(bool enable);
  void triggerSingle();

  // Interrupts
  /*
  uint8_t getInterruptStatus();
  void enableAmbientLightInterrupts(bool enable);
  uint16_t getAmbientLightHighThreshold();
  void setAmbientLightHighThreshold(uint16_t high_threshold);
  uint16_t getAmbientLightLowThreshold();
  void setAmbientLightLowThreshold(uint16_t low_threshold);

  void enableProximityInterrupts(VCNL4040_ProximityType interrupt_condition);
  uint16_t getProximityLowThreshold();
  void setProximityLowThreshold(uint16_t low_threshold);
  uint16_t getProximityHighThreshold();
  void setProximityHighThreshold(uint16_t high_threshold);
  */

  VCNL4040_ProximityIntegration getProximityIntegrationTime();
  void setProximityIntegrationTime(VCNL4040_ProximityIntegration integration_time);

  VCNL4040_AmbientIntegration getAmbientIntegrationTime();
  void setAmbientIntegrationTime(VCNL4040_AmbientIntegration integration_time);

  VCNL4040_LEDCurrent getProximityLEDCurrent();
  void setProximityLEDCurrent(VCNL4040_LEDCurrent led_current);

  VCNL4040_LEDDutyCycle getProximityLEDDutyCycle();
  void setProximityLEDDutyCycle(VCNL4040_LEDDutyCycle duty_cycle);

  bool getProximityHighResolution();
  void setProximityHighResolution(bool high_resolution);

private:
  bool _init();
  TwoWire* wire;
  uint8_t address;

  VCNL4040_ALSConfReg als_conf;
  VCNL4040_ALSThdLowReg als_thdLow;
  VCNL4040_ALSThdHighReg als_thdHigh;
  VCNL4040_PSConf12Reg ps_conf12;
  VCNL4040_PSConf3MSReg ps_conf3Ms;
  VCNL4040_PSCancReg ps_canc;
  VCNL4040_PSThdLowReg ps_thdLow;
  VCNL4040_PSThdHighReg ps_thdHigh;

  void writeRegister(uint8_t reg, uint16_t data);
  uint16_t readRegister(uint8_t reg);
};

#endif