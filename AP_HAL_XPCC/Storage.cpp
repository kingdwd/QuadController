#include <stdio.h>
#include <string.h>
#include "Storage.h"

using namespace XpccHAL;

Storage::Storage()
{}

void Storage::init(void*)
{}

void Storage::read_block(void* dst, uint16_t src, size_t n) {
	//printf("read_block %d\n", src);
    memset(dst, 0, n);
}

void Storage::write_block(uint16_t loc, const void* src, size_t n)
{
	//printf("write_block %d\n", loc);
}

