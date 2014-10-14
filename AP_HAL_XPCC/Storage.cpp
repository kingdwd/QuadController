#include <stdio.h>
#include <string.h>
#include "Storage.h"
#include <eeprom/eeprom.hpp>
#include <xpcc/architecture.hpp>
using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

Storage::Storage()
{
}

void Storage::init(void*)
{
}

//we are not using semaphore here because eeprom driver uses a
//different i2c delegate than i2cdriver
//and the i2c driver sorts everything out
void Storage::read_block(void* dst, uint16_t src, size_t n) {
	XPCC_LOG_DEBUG .printf("r\n");
	eeprom.read(src+1024, (uint8_t*)dst, n);
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	XPCC_LOG_DEBUG .printf("w\n");
	eeprom.write(loc+1024, (uint8_t*)src, n);
}
