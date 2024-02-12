#include <Arduino.h>
#include <Wire.h>

#ifndef TOOLS_H
#define TOOLS_H

class SerialSpeedTest {
    public:
		SerialSpeedTest(HardwareSerial& serial);
		
		void init();
		        
		void start();
		
	private:
		HardwareSerial& serial;
        int count;
        int prior_count;
        int count_per_second;
        int prior_msec;
};

class I2CTools {
    public:
        static void portScan(TwoWire* wire, int startAddress, int stopAddress);
        static void writeBytes(TwoWire* wire, uint8_t device, const uint8_t* writeBuf, int numBytes, bool repeatedStart = false);
        static void readBytes(TwoWire* wire, uint8_t device, int numBytes);
        static void switchScan(TwoWire* wire, int switchAddress, int noChannels = 4, int startAddress = 0x30, int stopAddress = 0x79);
        static void switchSetChannel(TwoWire* wire, int switchAddress, int channel);
};

#endif