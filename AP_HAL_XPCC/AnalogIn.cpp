#include <xpcc/architecture.hpp>
#include "AnalogIn.h"
#include <stdio.h>


using namespace Empty;
using namespace xpcc::lpc17;

const uint8_t analogFunc[] = {0, 0, 1, 1, 0, 3, 2, 2};

extern const AP_HAL::HAL& hal;

AnalogSource::AnalogSource(uint8_t chan) :
    _chan(chan)
{
	int8_t p = hal.gpio->analogPinToDigitalPin(chan);

	uint8_t port = p & 0x03;
	uint8_t pin = p >> 3;
	printf("Analog source %d PORT%d PIN%d\n", chan, port, pin);

	if(analogFunc[chan]) {

		ADC::enableChannel(chan);

		Pinsel::setFunc(port, pin, analogFunc[chan]);
	}
}

float AnalogSource::read_average() {
    return 0;
}

float AnalogSource::voltage_average() {
    return 0;
}

float AnalogSource::voltage_latest() {
    return 0;
}

float AnalogSource::read_latest() {
    return ADC::getData(_chan) * (3.3f / 4096.0f);
}

void AnalogSource::set_pin(uint8_t p)
{}

void AnalogSource::set_stop_pin(uint8_t p)
{}

void AnalogSource::set_settle_time(uint16_t settle_time_ms)
{}

AnalogIn::AnalogIn()
{
	memset(channels, 0, sizeof(channels)*sizeof(AP_HAL::AnalogSource*));
}

void AnalogIn::init(void* machtnichts)
{
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {

	if(n >= 8)
		return 0;

	if(channels[n] != 0) {
		return channels[n];
	}

	int8_t pin = hal.gpio->analogPinToDigitalPin(n);
	if(pin >= 0) {
		channels[n] = new AnalogSource(n);
	}

	return channels[n];
}

float AnalogIn::board_voltage(void)
{
    return 3.3f;
}
