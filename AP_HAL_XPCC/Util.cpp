/*
 * Util.c
 *
 *  Created on: Oct 4, 2014
 *      Author: walmis
 */
#include "Util.h"
#include <unistd.h>

extern "C" uint8_t __heap_end;

uint16_t XpccHAL::Util::available_memory(void) {

	return &__heap_end - (uint8_t*)sbrk(0);
}

