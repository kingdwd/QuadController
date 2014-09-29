
#include "GPIO.h"

using namespace Empty;

#include <xpcc/architecture.hpp>
#include <AP_HAL.h>

extern const AP_HAL::HAL& hal;


#define PIN(port, pin) ((pin<<3)|port)

const uint8_t analogMap[8] = {0, 0, PIN(0,25), PIN(0,26), 0/*PIN(1,30)*/, PIN(1,31), PIN(0,3), PIN(0,2)};
LPC_GPIO_TypeDef* const ports[] = {LPC_GPIO0, LPC_GPIO1, LPC_GPIO2, LPC_GPIO3};

GPIO::GPIO()
{}

void GPIO::init()
{}

void GPIO::pinMode(uint8_t pin, uint8_t output)
{
	uint8_t port = pin & 0x07;
	uint8_t p = pin >> 3;

	if(port <= 3) {
		if(output) {
			ports[port]->FIODIR |= (1<<pin);
		} else {
			ports[port]->FIODIR &= ~(1<<pin);
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
	uint8_t port = pin & 0x03;
	uint8_t p = (pin >> 3) ;

	return ports[port]->FIOPIN & (1<<pin);
}

void GPIO::write(uint8_t pin, uint8_t value)
{
	uint8_t port = pin & 0x03;
	uint8_t p = (pin >> 3) ;
	if(value)
		ports[port]->FIOSET = (1<<pin);
	else
		ports[port]->FIOCLR = (1<<pin);
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

	uint8_t port = interrupt_num & 0x03;
	uint8_t pin = (interrupt_num >> 3);

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

bool GPIO::usb_connected(void)
{
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
