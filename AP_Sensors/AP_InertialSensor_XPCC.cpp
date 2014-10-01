/*
 * AP_InertialSensor_XPCC.cpp
 *
 *  Created on: Oct 1, 2014
 *      Author: walmis
 */

#include "AP_InertialSensor_XPCC.h"
#include <stdio.h>

bool AP_InertialSensor_XPCC::update() {
	return true;
}

float AP_InertialSensor_XPCC::get_gyro_drift_rate() {
	return 0;
}

bool AP_InertialSensor_XPCC::wait_for_sample(uint16_t timeout_ms) {
	return true;
}

float AP_InertialSensor_XPCC::get_delta_time() const {
	return 0.005f;
}

uint16_t AP_InertialSensor_XPCC::_init_sensor(Sample_rate sample_rate) {
	printf("%s\n", __PRETTY_FUNCTION__);
	return 0;
}


