
#include "UARTDriver.h"

using namespace Empty;

UARTDriver::UARTDriver(xpcc::IODevice* device) :
	_device(device), _blocking_writes(0)
{

}

void UARTDriver::begin(uint32_t b) {

}
void UARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {

}
void UARTDriver::end() {

}
void UARTDriver::flush() {
	if(_device)
		_device->flush();
}
bool UARTDriver::is_initialized() {
	return _device != 0;
}

void UARTDriver::set_blocking_writes(bool blocking) {
	_blocking_writes = blocking;
}

bool UARTDriver::tx_pending() { return false; }

int16_t UARTDriver::available() {
	if(!_device)
		return 0;

	return _device->rxAvailable();
}
int16_t UARTDriver::txspace() {
	if(!_device)
		return -1;
	return _device->txAvailable();
}
int16_t UARTDriver::read() {
	if(!_device)
		return -1;
	return _device->read();
}

size_t UARTDriver::write(uint8_t c) {
	if(!_device)
		return 0;
	if(_blocking_writes) {
		return _device->write(c);
	} else {
		if(_device->txAvailable() > 0) {
			_device->write(c);
			return 1;
		}
	}
	return 0;
}

size_t UARTDriver::write(const uint8_t *buffer, size_t size)
{
    size_t n = 0;
    while (size--) {
        n += write(*buffer++);
    }
    return n;
}
