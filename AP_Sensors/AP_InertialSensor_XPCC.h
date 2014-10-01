/*
 * AP_InertialSensor_XPCC.h
 *
 *  Created on: Oct 1, 2014
 *      Author: walmis
 */

#ifndef AP_INERTIALSENSOR_XPCC_H_
#define AP_INERTIALSENSOR_XPCC_H_

#include <AP_InertialSensor.h>


class AP_InertialSensor_XPCC : public AP_InertialSensor {
public:

	/* Concrete implementation of AP_InertialSensor functions: */
	bool                update();
	float               get_gyro_drift_rate();

	// wait for a sample to be available, with timeout in milliseconds
	bool                wait_for_sample(uint16_t timeout_ms);

	// get_delta_time returns the time period in seconds overwhich the sensor data was collected
	float            	get_delta_time() const;

	uint16_t error_count(void) const { return _error_count; }
	bool healthy(void) const { return _error_count <= 4; }
	bool get_gyro_health(uint8_t instance) const { return healthy(); }
	bool get_accel_health(uint8_t instance) const { return healthy(); }

private:
	uint16_t _error_count;

	uint16_t        _init_sensor( Sample_rate sample_rate );
};


#endif /* AP_INERTIALSENSOR_XPCC_H_ */
