#include <stdio.h>
#include <string.h>
#include "Storage.h"
#include <eeprom/eeprom.hpp>

using namespace XpccHAL;

Storage::Storage()
{}

void Storage::init(void*)
{}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
	//printf("read_block %d\n", src);
    eeprom.read(src+1024, (uint8_t*)dst, n);

}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	//printf("write_block %d\n", loc);
	eeprom.write(loc+1024, (uint8_t*)src, n);
}

