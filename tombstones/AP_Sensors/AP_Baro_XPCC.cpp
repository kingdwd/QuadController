/*
 * AP_Baro_XPCC.cpp
 *
 *  Created on: Oct 1, 2014
 *      Author: walmis
 */

#include <AP_Sensors/AP_Baro_XPCC.h>

bool AP_Baro_XPCC::init() {
	_flags.healthy = true;

	return true;
}

uint8_t AP_Baro_XPCC::read() {
	return 1;
}

float AP_Baro_XPCC::get_pressure() {
	return 1000.0*100;
}

float AP_Baro_XPCC::get_temperature() {
	return 25.0;
}
