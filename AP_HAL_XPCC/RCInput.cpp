
#include "RCInput.h"
#include <radio.hpp>

#define NUM_CHANNELS 9

using namespace XpccHAL;
RCInput::RCInput()
{}

void RCInput::init(void* machtnichts)
{}

#define RC_ROLL 0
#define RC_PITCH 1
#define RC_THROTTLE 2
#define RC_YAW 3
#define RC_5 4
#define RC_6 5
#define RC_7 6
#define RC_8 7
#define RC_9 8

bool RCInput::new_input() {
    if(radio.rcPacketTimestamp != last_read) {
    	return true;
    }
	return false;
}

uint8_t RCInput::num_channels() {
    return NUM_CHANNELS;
}

uint16_t RCInput::read(uint8_t ch) {

	//if radio link is lost for 500ms center axes
	if(xpcc::Clock::now() - radio.rcPacketTimestamp > 500) {
		radio.rcData.rollCh = 1500;
		radio.rcData.pitchCh = 1500;
		radio.rcData.yawCh = 1500;
	}

	//if radio link is lost for 2s, trigger throttle failsafe
	if(xpcc::Clock::now() - radio.rcPacketTimestamp > 1500) {
		radio.rcData.throttleCh = 900;
	}

	uint16_t val = 0;
	switch(ch) {
    case RC_ROLL:
    	val = radio.rcData.rollCh;
    	break;
    case RC_PITCH:
    	val = radio.rcData.pitchCh;
    	break;
    case RC_THROTTLE:
    	val = radio.rcData.throttleCh;
    	break;
    case RC_YAW:
    	val = radio.rcData.yawCh;
    	break;
    case RC_5: {
    	uint8_t c = radio.rcData.switches >> 3;
    	switch(c) {
    	case 0:
    		return 1500;
    	case 1:
    		return 1000;
    	case 2:
    		return 2000;
    	}
    	break;
    }
    case RC_6:
    	val = radio.rcData.auxCh;
    	break;

	case RC_7:
		return (radio.rcData.switches & (1<<0)) ? 2000 : 1000;
	case RC_8:
		return (radio.rcData.switches & (1<<1)) ? 2000 : 1000;
	case RC_9:
		return (radio.rcData.switches & (1<<2)) ? 2000 : 1000;
	}

	return 990 + val;
}

uint8_t RCInput::read(uint16_t* periods, uint8_t len) {
	for (uint8_t i = 0; i < len; i++){
		periods[i] = read(i);
    }
    return len;
}

bool RCInput::set_overrides(int16_t *overrides, uint8_t len) {
    return true;
}

bool RCInput::set_override(uint8_t channel, int16_t override) {
    return true;
}

void RCInput::clear_overrides()
{}

