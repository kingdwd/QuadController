
#include <AP_HAL.h>
#include "I2CDriver.h"

using namespace XpccHAL;

void I2CDriver::begin() {}
void I2CDriver::end() {}
void I2CDriver::setTimeout(uint16_t ms) {}
void I2CDriver::setHighSpeed(bool active) {}

#define I2C xpcc::lpc17::I2cMaster2
extern const AP_HAL::HAL& hal;

uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!adapter.initialize(addr, data, len, 0, 0)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}
uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;
	if(!adapter.initialize(addr, data, sizeof(data), 0, 0)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}
uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{
	uint8_t buf[len + 1];
	buf[0] = reg;
	memcpy(&buf[1], data, len);

	if(!adapter.initialize(addr, buf, len+1, 0, 0)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!adapter.initialize(addr, 0, 0, data, len)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	if(!adapter.initialize(addr, &reg, 1, data, 1)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
	if(!len) return 1;

	if(!adapter.initialize(addr, &reg, 1, data, len)) {
		return 1;
	}
	if(!I2C::start(&adapter)) {
		return 1;
	}
	while(adapter.getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		//yield();
	}
	return adapter.getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::lockup_count() {
	return 0;
}
