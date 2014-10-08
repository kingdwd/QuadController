#include <stdio.h>
#include <string.h>
#include "Storage.h"
#include <eeprom/eeprom.hpp>
#include <xpcc/architecture.hpp>
using namespace XpccHAL;

extern const AP_HAL::HAL& hal;

Storage::Storage()
{}

void Storage::init(void*)
{}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
	//printf("read_block %d\n", src);
	AP_HAL::Semaphore *sem = hal.i2c->get_semaphore();
	if(sem->take(1000)) {
		//hal.console->printf("eeread %x %d\n", src, n);
		eeprom.read(src+1024, (uint8_t*)dst, n);
		sem->give();
	} else {
		hal.console->println("Failed to take i2c semaphore in Storage::read_block");
	}

}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	AP_HAL::Semaphore *sem = hal.i2c->get_semaphore();
	if(sem->take(1000)) {
		//hal.console->printf("eewrite %x %d\n", loc, n);
		eeprom.write(loc+1024, (uint8_t*)src, n);
		sem->give();
	} else {
		hal.console->println("Failed to take i2c semaphore in Storage::write_block");
	}
}

