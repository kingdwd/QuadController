
#include "GPIO.h"

using namespace XpccHAL;

#include <xpcc/architecture.hpp>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;

const uint8_t analogMap[8] = { 0, 0, GPIO_PIN(0, 25), GPIO_PIN(0, 26),
		GPIO_PIN(1,30), GPIO_PIN(1, 31), GPIO_PIN(0, 3), GPIO_PIN(0, 2) };

LPC_GPIO_TypeDef* const ports[] = { LPC_GPIO0, LPC_GPIO1, LPC_GPIO2 };

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
	uint8_t port = PORT(pin);
	uint8_t p = PIN(pin);

	if(port <= 3) {
		if(output) {
			ports[port]->FIODIR |= (1<<p);
		} else {
			ports[port]->FIODIR &= ~(1<<p);
		}
	}
}

int8_t GPIO::analogPinToDigitalPin(uint8_t pin)
{
	if(pin >= sizeof(analogMap))
		return -1;

	if(analogMap[pin] == 0)
		return -1;

	return analogMap[pin];
}


uint8_t GPIO::read(uint8_t pin) {
	uint8_t port = PORT(pin);
	uint8_t p = PIN(pin);

	return ports[port]->FIOPIN & (1<<p);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
	uint8_t port = PORT(pin);
	uint8_t p = PIN(pin);
	if(value)
		ports[port]->FIOSET = (1<<p);
	else
		ports[port]->FIOCLR = (1<<p);
}

void GPIO::toggle(uint8_t pin)
{
	write(pin, !read(pin));
}

/* Alternative interface: */
AP_HAL::DigitalSource* GPIO::channel(uint16_t n) {
    return new DigitalSource(n);
}

/* Interrupt interface: */
bool GPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) {

	uint8_t port = PORT(interrupt_num);
	uint8_t pin = PIN(interrupt_num);

	xpcc::IntEdge m;
    if (mode == HAL_GPIO_INTERRUPT_FALLING)
    	m = xpcc::IntEdge::FALLING_EDGE;
    else if (mode == HAL_GPIO_INTERRUPT_RISING)
    	m = xpcc::IntEdge::RISING_EDGE;
    else {
    	return false;
    }

	if(port == 2 || port == 0) {
		xpcc::GpioInt::attach(port, pin, p, m);
		return true;
	}

    return false;
}

DigitalSource::DigitalSource(uint8_t pin) :
    _pin(pin)
{}

void DigitalSource::mode(uint8_t output)
{
	hal.gpio->pinMode(_pin, output);
}

uint8_t DigitalSource::read() {
    return hal.gpio->read(_pin);
}

void DigitalSource::write(uint8_t value) {
	hal.gpio->write(_pin, value);
}

void DigitalSource::toggle() {
	hal.gpio->toggle(_pin);
}
