#include <xpcc/architecture.hpp>
#include "AnalogIn.h"

using namespace XpccHAL;
using namespace xpcc::lpc17;

extern const AP_HAL::HAL& hal;

AnalogSource::AnalogSource(int16_t chan) : _chan(-1)
{
	set_pin(chan);
}

float AnalogSource::read_average() {
    return _voltage_avg;
}

float AnalogSource::voltage_average() {
	//hal.console->printf("%d %d \n", ADC::getData(4), ADC::getData(5));
    return _voltage_avg;
}

float AnalogSource::voltage_latest() {
    return read_latest();
}

float AnalogSource::read_latest() {
	if(!_chan)
		return -1.0;

    return ADC::getData(_chan) * (3.3f / 4096.0f);
}

void AnalogSource::_tick() {
	_voltage_avg = (_voltage_avg*15 + read_latest()) / 16;
}

void AnalogSource::set_pin(uint8_t p)
{
	if(p == _chan)
		return;

	if(p > 8) {
		_chan = 0;
		return;
	}

	_chan = p;

	ADC::enableChannel(_chan);

}

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

void AnalogIn::_tick()
{
	for(int i = 0; i < 16; i++) {
		if(channels[i]) {
			channels[i]->_tick();
		}
	}
}

AP_HAL::AnalogSource* AnalogIn::channel(int16_t n) {

	for(int i = 0; i < 16; i++) {
		if(channels[i] == 0) {
			channels[i] = new AnalogSource(n);
			return channels[i];
		}
	}
	hal.console->println("Out of analog channels");
	return 0;
}
