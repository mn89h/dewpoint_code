/*!
 *  @file SHT4X.cpp
 *
 *  @mainpage Adafruit SHT4X Digital Humidity & Temp Sensor
 *
 *  @section intro_sec Introduction
 *
 *  This is a library for the SHT4X Digital Humidity & Temp Sensor
 *
 *  Designed specifically to work with the SHT4X Digital sensor from Adafruit
 *
 *  Pick one up today in the adafruit shop!
 *  ------> https://www.adafruit.com/product/4885
 *
 *  These sensors use I2C to communicate, 2 pins are required to interface
 *
 *  Adafruit invests time and resources providing this open source code,
 *  please support Adafruit andopen-source hardware by purchasing products
 *  from Adafruit!
 *
 *  @section author Author
 *
 *  Limor Fried/Ladyada (Adafruit Industries).
 *
 *  @section license License
 *
 *  BSD license, all text above must be included in any redistribution
 */

#include "SHT4X.h"

static uint8_t crc8(const uint8_t *data, int len);

/*!
 * @brief  SHT4X constructor
 */
SHT4X::SHT4X(TwoWire *wire) :
  wire(wire)
{
}

/**
 * Initialises the I2C bus, and assigns the I2C address to us.
 *
 * @param theWire   The I2C bus to use, defaults to &Wire
 *
 * @return True if initialisation was successful, otherwise False.
 */
bool SHT4X::init() {
  if (!reset()) {
    return false;
  }
}

/**
 * Gets the ID register contents.
 *
 * @return The 32-bit ID register.
 */
uint32_t SHT4X::readSerial(void) {
  uint8_t cmd = SHT4X_READSERIAL;
  uint8_t reply[6];

  if (!writeCmd(cmd)) {
    return false;
  }
  delay(10);
  if (readBytes(reply, 6)) {
    return false;
  }

  if ((crc8(reply, 2) != reply[2]) || (crc8(reply + 3, 2) != reply[5])) {
    return false;
  }

  uint32_t serial = 0;
  serial = reply[0];
  serial <<= 8;
  serial |= reply[1];
  serial <<= 8;
  serial |= reply[3];
  serial <<= 8;
  serial |= reply[4];

  return serial;
}

/**
 * Performs a soft reset of the sensor to put it into a known state.
 * @returns True on success, false if could not communicate with chip
 */
bool SHT4X::reset(void) {
  uint8_t cmd = SHT4X_SOFTRESET;
  if (!writeCmd(cmd)) {
    return false;
  }
  delay(1);
  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the precision rating - more precise takes longer!
    @param  prec The desired precision setting, will be used during reads
*/
/**************************************************************************/
void SHT4X::setCommand(sht4x_cmd cmd) { _cmd = cmd; }

/**************************************************************************/
/*!
    @brief  Gets the precision rating - more precise takes longer!
    @returns  The current precision setting, will be used during reads
*/
/**************************************************************************/
sht4x_cmd SHT4X::getCommand(void) { return _cmd; }

/**************************************************************************/
/*!
    @brief  Waits for the measurement to complete
*/
/**************************************************************************/
void SHT4X::wait() {
  uint32_t duration = 0;
  if (_cmd == SHT4X_HIGH_HEATER_100MS || _cmd == SHT4X_MED_HEATER_100MS || _cmd == SHT4X_LOW_HEATER_100MS) {
    duration = 100;
  }
  else if (_cmd == SHT4X_HIGH_HEATER_1S || _cmd == SHT4X_MED_HEATER_1S || _cmd == SHT4X_LOW_HEATER_1S) {
    duration = 1000;
  }
  else if (_cmd == SHT4X_NO_HEATER_HIGHPRECISION) {
    duration = 10;
  }
  else if (_cmd == SHT4X_NO_HEATER_MEDPRECISION) {
    duration = 5;
  }
  else if (_cmd == SHT4X_NO_HEATER_LOWPRECISION) {
    duration = 2;
  }
  delay(duration);
}

/**************************************************************************/
/*!
    @brief  Waits for the measurement to complete
*/
/**************************************************************************/
bool SHT4X::receiveData() {
  uint8_t readbuffer[6];
  if (!readBytes(readbuffer, 6)) {
    return false;
  }
  if (readbuffer[2] != crc8(readbuffer, 2) ||
      readbuffer[5] != crc8(readbuffer + 3, 2))
    return false;

  float t_ticks = (uint16_t)readbuffer[0] * 256 + (uint16_t)readbuffer[1];
  float rh_ticks = (uint16_t)readbuffer[3] * 256 + (uint16_t)readbuffer[4];
  _temperature = -45 + 175 * t_ticks / 65535;
  _humidity = -6 + 125 * rh_ticks / 65535;

  _humidity = min(max(_humidity, (float)0.0), (float)100.0);

  return true;
}


/**************************************************************************/
/*!
    @brief  Gets the humidity sensor and temperature values as sensor events
    @param  humidity Sensor event object that will be populated with humidity
   data
    @param  temp Sensor event object that will be populated with temp data
    @returns true if the event data was read successfully
*/
/**************************************************************************/
bool SHT4X::measure(bool asyncMode) {
  if (!writeCmd(_cmd)) {
    return false;
  }

  if (!asyncMode) {
    wait();
    return receiveData();
  }
  return true;
}

float SHT4X::getTemperature() {
  return _temperature;
}

float SHT4X::getHumidity() {
  return _humidity;
}

/**
 * Performs a CRC8 calculation on the supplied values.
 *
 * @param data  Pointer to the data to use when calculating the CRC8.
 * @param len   The number of bytes in 'data'.
 *
 * @return The computed CRC8 value.
 */
static uint8_t crc8(const uint8_t *data, int len) {
  /*
   *
   * CRC-8 formula from page 14 of SHT spec pdf
   *
   * Test data 0xBE, 0xEF should yield 0x92
   *
   * Initialization data 0xFF
   * Polynomial 0x31 (x8 + x5 +x4 +1)
   * Final XOR 0x00
   */

  const uint8_t POLYNOMIAL(0x31);
  uint8_t crc(0xFF);

  for (int j = len; j; --j) {
    crc ^= *data++;

    for (int i = 8; i; --i) {
      crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
    }
  }
  return crc;
}


bool SHT4X::readBytes(uint8_t *val, uint8_t n)
{
  int rv = wire->requestFrom(SHT4X_DEFAULT_ADDR, (uint8_t) n);
  if (rv == n)
  {
    for (uint8_t i = 0; i < n; i++)
    {
      val[i] = wire->read();
    }
    return true;
  }
  return false;
}

bool SHT4X::writeCmd(uint8_t cmd)
{
  wire->beginTransmission(SHT4X_DEFAULT_ADDR);
  wire->write(cmd);
  if (wire->endTransmission() != 0)
  {
    return false;
  }
  return true;
}
