#include <Arduino.h>
#include "tools.h"

SerialSpeedTest::SerialSpeedTest(HardwareSerial& serial) : serial(serial) {}

void SerialSpeedTest::init() {
  count = 10000000; // starting with 8 digits gives consistent chars/line
  prior_count = count;
  count_per_second = 0;
  prior_msec = millis();
}

void SerialSpeedTest::start() {
    for (int i = 0; i < 100000000; i++){
    serial.print("count=");
    serial.print(count);
    serial.print(", lines/sec=");
    serial.println(count_per_second);
    count = count + 1;
    uint32_t msec = millis();
    if (msec - prior_msec > 1000) {
      // when 1 second as elapsed, update the lines/sec count
      prior_msec = prior_msec + 1000;
      count_per_second = count - prior_count;
      prior_count = count;
    }
  }
}

void I2CTools::portScan(TwoWire* wire, int startAddress, int stopAddress) {
  for(int i = startAddress; i <= stopAddress; i++){
      wire->beginTransmission(i);
      uint8_t code = wire->endTransmission();
      if(code == 0){
          Serial.print("Device at 0x");
          Serial.print(i, HEX);
          Serial.println(", Code: 0");
      }
      if(code == 4){
          Serial.print("Device at 0x");
          Serial.print(i, HEX);
          Serial.println(", Code: 4");
      }
  }
  Serial.println("Finished port scan");
}

void I2CTools::switchScan(TwoWire* wire, int switchAddress, int noChannels, int startAddress, int stopAddress) {
  for(int ch = 0; ch < noChannels; ch++){
    Serial.print("Channel ");
    Serial.println(ch);
    
    I2CTools::switchSetChannel(wire, switchAddress, ch);
    I2CTools::portScan(wire, startAddress, stopAddress);
  }
  Serial.println();
}

void I2CTools::switchSetChannel(TwoWire* wire, int switchAddress, int channel) {
  uint8_t channelBitstream = 1 << channel;
  I2CTools::writeBytes(wire, switchAddress, &channelBitstream, 1);
}

/// @brief Writes bytes to an I2C device. Pass an array with the data
/// @brief and if necessary the prepending register address to write to.
/// @brief Example: uint8_t data[] = {0x01, 0xFF};
/// @param wire The I2C interface. 
/// @param device The device address (7bits)
/// @param writeBuf The writeBuffer (with preceding register addresses)
/// @param numBytes The total number of bytes (including register address bytes)
/// @param repeatedStart Determines whether STOP bit is left out after transmission. Defaults to false. 
void I2CTools::writeBytes(TwoWire *wire, uint8_t device, const uint8_t* writeBuf, int numBytes, bool repeatedStart) {
  wire->beginTransmission(device);
  wire->write(writeBuf, numBytes);
  wire->endTransmission(~repeatedStart);
}

/// @brief Reads bytes from device and prints it to Serial. If reading from a 
/// @brief specific register is required, call writeBytes(...) with the address register first
/// @param wire The I2C interface
/// @param device The device address
/// @param numBytes The number of bytes to read
void I2CTools::readBytes(TwoWire *wire, uint8_t device, int numBytes) {
  uint8_t* readBuf = new uint8_t[numBytes];
  wire->readBytes(readBuf, numBytes);
  Serial.write(readBuf, numBytes);
}
