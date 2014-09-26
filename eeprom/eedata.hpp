/*
 * eedata.hpp
 *
 *  Created on: May 8, 2014
 *      Author: walmis
 */

#ifndef EEDATA_HPP_
#define EEDATA_HPP_

#include <xpcc/math.hpp>
#include <RH_RF22.h>

#define TOKEN 0x70

struct EEData {
	uint8_t token;

	float rfFrequency; //radio frequency
	float afcPullIn;
	uint8_t txPower;
	uint8_t fhChannels;
	RH_RF22::ModemConfigChoice modemCfg;

	float qTrim[4];
	float qRotationOffset[4];

	float ratePIDParams[3];
	float PIDParams[3];
	float heightPIDparams[4]; //p, i, d, maxoutput

	float yawGain;

	uint8_t magnetometerCalibration[6];

	int32_t gyroBias[3];

} __attribute((packed));
//const float rpid[3] = {0.1070, 0.0015, 0};
//const float pid[3]  = {1.4700, 0.0700, 0};
//const float hpid[4] = {0, 0, 0.25, 0.2};
const EEData eeDefaults = {
		TOKEN,
		424.0f,
		0.05f,
		RH_RF22_TXPOW_1DBM,
		0,
		RH_RF22::GFSK_Rb57_6Fd28_8,
		{1.0f, 0, 0, 0},
		{1.0f, 0, 0, 0},
		{0.1070f, 0.0015f, 0.0f}, //angular rate pid params
		{1.47f, 0.070f, 0.0f}, //pid params
		{0.0, 0.0, 0.0, 0.0},
		1.7f,
		{0,0,0,0,0,0},
		{0,0,0}
};


#endif /* EEDATA_HPP_ */
