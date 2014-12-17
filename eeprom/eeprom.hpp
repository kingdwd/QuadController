/*
 *
 *  Created on: May 7, 2014
 *      Author: walmis
 */
#ifndef EEPROM_HPP_
#define EEPROM_HPP_

#include <xpcc/architecture.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>

#include "eedata.hpp"


class Eeprom : public xpcc::I2cEeprom<xpcc::lpc17::I2cMaster2> {
public:
	Eeprom() : xpcc::I2cEeprom<xpcc::lpc17::I2cMaster2>(0x50, 128), token(0) {}
	virtual ~Eeprom(){};

	void initialize()  {
//		readByte(0, token);
//
//		if(token != TOKEN) {
//			write(0, (uint8_t*)&eeDefaults, sizeof(eeDefaults));
//		}
	}

	bool isValidToken() {
		return token == TOKEN;
	}

	void clearToken() {
		writeByte(0, 0);
	}

	void setToken() {
		writeByte(0, TOKEN);
		token = TOKEN;
	}

protected:
	uint8_t token;
};

extern Eeprom eeprom;

#endif
