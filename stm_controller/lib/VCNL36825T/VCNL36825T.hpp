/*!
 *  @file VCNL36825T.hpp
 *
 * 	I2C Driver for VCNL36825T proximity and ambient light sensor
 *
 *	BSD license (see license.txt)
 */

#ifndef _VCNL36825T_HPP
#define _VCNL36825T_HPP

#include "Arduino.h"
#include <Wire.h>
#define VCNL36825T_I2CADDR_DEFAULT 0x60 ///< VCNL36825T default i2c address

// All addresses are for 16bit registers;
// duplicates are for high or low bytes that aren't used together
#define VCNL36825T_REG__PS_CONF1 0x00     ///< Proximity sensor configuration 1 register
#define VCNL36825T_REG__PS_CONF2 0x03     ///< Proximity sensor configuration 2 register
#define VCNL36825T_REG__PS_CONF3 0x04     ///< Proximity sensor configuration 3 register
#define VCNL36825T_REG__PS_THDL 0x05      ///< Proximity sensor low threshold register
#define VCNL36825T_REG__PS_THDH 0x06      ///< Proximity sensor high threshold register
#define VCNL36825T_REG__PS_CANC 0x07      ///< PS cancellation setting
#define VCNL36825T_REG__PS_CONF4 0x08     ///< PS auto-calibration period, number, interrupt setting, low power
#define VCNL36825T_REG__PS_DATA 0xF8      ///< Proximity sensor data register
#define VCNL36825T_REG__INT_FLAG 0xF9     ///< Interrupt status register
#define VCNL36825T_REG__DEVICE_ID 0xFA    ///< Device ID
#define VCNL36825T_REG__AC_DATA 0xFB      ///< PS auto calibration data, busy, sunlight protect


typedef union {
	uint16_t rawData;
	struct {
		uint8_t reserved1 : 1;
    uint8_t on : 1;
    uint8_t reserved2 : 5;
    uint8_t cal : 1;
    uint8_t reserved3 : 8;
	};
} VCNL36825T_PSConf1Reg;

typedef union {
	uint16_t rawData;
	struct {
    uint8_t st : 1;
    uint8_t smart_pers : 1;
    uint8_t int_en : 2;
    uint8_t pers : 2;
    uint8_t period : 2;
    uint8_t reserved1 : 2;
    uint8_t hg :  1;
    uint8_t itb : 1;
    uint8_t mps : 2;
    uint8_t it : 2;
	};
} VCNL36825T_PSConf2Reg;

typedef union {
	uint16_t rawData;
	struct {
    uint8_t reserved1 : 2;
		uint8_t sp_int : 1;
    uint8_t reserved2 : 1;
    uint8_t forcenum : 1;
    uint8_t trig : 1;
    uint8_t af : 1;
    uint8_t reserved3 : 1;
    uint8_t i_vcsel : 4;
    uint8_t hd : 1;
    uint8_t sc : 3;
	};
} VCNL36825T_PSConf3Reg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdh_lo : 8;
    uint8_t thdh_hi : 8;
	};
} VCNL36825T_PSThdHighReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t thdl_lo : 8;
    uint8_t thdl_hi : 8;
	};
} VCNL36825T_PSThdLowReg;

typedef union {
	uint16_t rawData;
	struct {
		uint8_t canc_lo : 8;
    uint8_t canc_hi : 8;
	};
} VCNL36825T_PSCancReg;

typedef union {
	uint16_t rawData;
	struct {
    uint8_t ac_int : 2;
    uint8_t reserved1 : 1;
		uint8_t ac_trig : 1;
    uint8_t ac : 1;
    uint8_t ac_num : 1;
    uint8_t ac_period : 2;
    uint8_t lpen : 1;
    uint8_t lpper : 2;
    uint8_t reserved2 : 5;
	};
} VCNL36825T_PSConf4Reg;

enum class VCNL36825T_LEDPeriod { PERIOD_10MS, PERIOD_20MS, PERIOD_40MS, PERIOD_80MS };
enum class VCNL36825T_LEDCurrent {
  RESERVED0 = 0,
  RESERVED1,
  LED_CURRENT_10MA,
  LED_CURRENT_12MA,
  LED_CURRENT_14MA,
  LED_CURRENT_16MA,
  LED_CURRENT_18MA,
  LED_CURRENT_20MA,
};

enum class VCNL36825T_ProximityIntegration {
  PROXIMITY_INTEGRATION_TIME_1T = 0,
  PROXIMITY_INTEGRATION_TIME_2T,
  PROXIMITY_INTEGRATION_TIME_4T,
  PROXIMITY_INTEGRATION_TIME_8T
};
enum class VCNL36825T_ProximityIntegrationBank { PROXIMITY_INTEGRATION_TIME_BANK_25US = 0, PROXIMITY_INTEGRATION_TIME_BANK_50US };
enum class VCNL36825T_ProximityType { PROXIMITY_INT_DISABLE, PROXIMITY_INT_CLOSE, PROXIMITY_INT_AWAY, PROXIMITY_INT_CLOSE_AWAY };
enum class VCNL36825T_InterruptType { PROXIMITY_AWAY, PROXIMITY_CLOSE, AMBIENT_HIGH = 4, AMBIENT_LOW };
enum class VCNL36825T_MultiPulse { MPS1, MPS2, MPS4, MPS8 };

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the VCNL36825T I2C Digital Potentiometer
 */
class VCNL36825T {
public:
  VCNL36825T(TwoWire *wire, uint8_t i2c_addr = VCNL36825T_I2CADDR_DEFAULT);
  boolean init();
  uint16_t getProximity();

  void enableSensor(bool start);
  void enableProximity(bool enable);

  // Interrupts
  /*
  uint8_t getInterruptStatus();

  void enableProximityInterrupts(VCNL36825T_ProximityType interrupt_condition);
  uint16_t getProximityLowThreshold();
  void setProximityLowThreshold(uint16_t low_threshold);
  uint16_t getProximityHighThreshold();
  void setProximityHighThreshold(uint16_t high_threshold);
  */

  VCNL36825T_ProximityIntegration getProximityIntegrationTime();
  void setProximityIntegrationTime(VCNL36825T_ProximityIntegration integration_time);

  VCNL36825T_ProximityIntegrationBank getProximityIntegrationTimeBank();
  void setProximityIntegrationTimeBank(VCNL36825T_ProximityIntegrationBank integration_time_bank);

  VCNL36825T_MultiPulse getMultiPulse();
  void setMultiPulse(VCNL36825T_MultiPulse multi_pulse);
  
  VCNL36825T_LEDPeriod getProximityLEDPeriod();
  void setProximityLEDPeriod(VCNL36825T_LEDPeriod period);

  VCNL36825T_LEDCurrent getProximityLEDCurrent();
  void setProximityLEDCurrent(VCNL36825T_LEDCurrent led_current);

  bool getProximityHighResolution();
  void setProximityHighResolution(bool high_resolution);

private:
  bool _init();
  TwoWire* wire;
  uint8_t address;

  VCNL36825T_PSConf1Reg ps_conf1;
  VCNL36825T_PSConf2Reg ps_conf2;
  VCNL36825T_PSConf3Reg ps_conf3;
  VCNL36825T_PSConf4Reg ps_conf4;
  VCNL36825T_PSCancReg ps_canc;
  VCNL36825T_PSThdLowReg ps_thdLow;
  VCNL36825T_PSThdHighReg ps_thdHigh;

  void writeRegister(uint8_t reg, uint16_t data);
  uint16_t readRegister(uint8_t reg);
};

#endif