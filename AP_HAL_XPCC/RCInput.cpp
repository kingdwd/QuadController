
#include "RCInput.h"
#include <radio.hpp>

#define NUM_CHANNELS 6

using namespace Empty;
RCInput::RCInput()
{}

void RCInput::init(void* machtnichts)
{}

#define RC_ROLL 0
#define RC_PITCH 1
#define RC_THROTTLE 2
#define RC_YAW 3

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

