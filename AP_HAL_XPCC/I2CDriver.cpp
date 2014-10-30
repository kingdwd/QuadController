
#include <AP_HAL.h>
#include "I2CDriver.h"
#include <xpcc/architecture.hpp>

using namespace XpccHAL;

void I2CDriver::begin() {}
void I2CDriver::end() {}
void I2CDriver::setTimeout(uint16_t ms) {}
void I2CDriver::setHighSpeed(bool active) {}

#define I2C xpcc::lpc17::I2cMaster2
extern const AP_HAL::HAL& hal;

uint8_t I2CDriver::write(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!initialize(addr, data, len, 0, 0)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}
uint8_t I2CDriver::writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = val;
	if(!initialize(addr, data, sizeof(data), 0, 0)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}
uint8_t I2CDriver::writeRegisters(uint8_t addr, uint8_t reg,
                               uint8_t len, uint8_t* data)
{
	uint8_t buf[len + 1];
	buf[0] = reg;
	memcpy(&buf[1], data, len);

	if(!initialize(addr, buf, len+1, 0, 0)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::read(uint8_t addr, uint8_t len, uint8_t* data)
{
	if(!initialize(addr, 0, 0, data, len)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::readRegister(uint8_t addr, uint8_t reg, uint8_t* data)
{
	if(!initialize(addr, &reg, 1, data, 1)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

uint8_t I2CDriver::readRegisters(uint8_t addr, uint8_t reg,
                                      uint8_t len, uint8_t* data)
{
	if(!len) return 1;

	if(!initialize(addr, &reg, 1, data, len)) {
		return 1;
	}
	if(!I2C::start(this)) {
		return 1;
	}
	while(getState() == xpcc::I2cWriteReadAdapter::AdapterState::Busy) {
		xpcc::yield();
	}
	return getState() != xpcc::I2cWriteReadAdapter::AdapterState::Idle;
}

void I2CDriver::stopped(DetachCause cause) {
	xpcc::atomic::Lock l;
	I2cWriteReadAdapter::stopped(cause);
	if(nb_transaction) {
		if(nb_callback != 0){
			nb_transaction = false;
			nb_callback();
		}
	}
}

bool I2CDriver::readNonblocking(uint8_t addr, uint8_t reg,
                              uint8_t len, uint8_t* data,
							  AP_HAL::MemberProc callback) {
	if(nb_transaction || !len) {
		return false;
	}

	data[0] = reg;
	if(!initialize(addr, data, 1, data, len)) {
		return false;
	}
	if(!I2C::start(this)) {
		return false;
	}

	nb_callback = callback;
	nb_transaction = true;

	return true;
}

uint8_t I2CDriver::lockup_count() {
	return 0;
}
