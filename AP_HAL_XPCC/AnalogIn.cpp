#include <xpcc/architecture.hpp>
#include "AnalogIn.h"

using namespace XpccHAL;
using namespace xpcc::lpc17;

extern const AP_HAL::HAL& hal;

AnalogSource::AnalogSource(int16_t chan) : _chan(-1)
{
	set_pin(chan);
	count = 0;
	sum = 0;
	_voltage_avg = 0;
	_voltage_latest = 0;
}

float AnalogSource::read_average() {
    return _voltage_avg;
}

float AnalogSource::voltage_average() {
	//hal.console->printf("%.3f\n", _voltage_avg);

    return _voltage_avg;
}

float AnalogSource::voltage_latest() {
    return read_latest();
}

float AnalogSource::read_latest() {
	if(!_chan)
		return -1.0;

    return _voltage_latest;
}

void AnalogSource::_irq() {
	uint32_t d = ADC::getData(_chan);
	filter.append(d);
	filter.update();

	sum += filter.getValue();
	count++;

}

void AnalogSource::_tick() {
	if(count) {
		_voltage_latest = ((float)sum/count) * (3.3f / 4096.0f);
		sum = 0;
		count = 0;
	}

	_voltage_avg = (_voltage_avg*99 + _voltage_latest) / 100.0;
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
	ADC::enableChannelInt(_chan, true);
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
	ADC::init(10000);
	ADC::burstMode(true);

	NVIC_EnableIRQ(ADC_IRQn);
}

void AnalogIn::_tick()
{
	for(int i = 0; i < 16; i++) {
		if(channels[i]) {
			channels[i]->_tick();
		}
	}
}

void AnalogIn::irq() {
	for(int i = 0; i < 16; i++) {
		if(channels[i] && ADC::isDone(channels[i]->_chan)) {
			channels[i]->_irq();
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


extern "C"
void ADC_IRQHandler() {
	static_cast<AnalogIn*>(hal.analogin)->irq();

}
