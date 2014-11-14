/*
 * cmd_terminal.cpp
 *
 *  Created on: Sep 26, 2014
 *      Author: walmis
 */

#include "cmd_terminal.hpp"
#include "eeprom/eeprom.hpp"
#include "radio.hpp"
#include <xpcc/architecture.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>

extern uint32_t crashData[3];

extern const AP_HAL::HAL& hal;

using namespace xpcc;
using namespace xpcc::lpc17;

extern void sdread(int b);

void CmdTerminal::handleCommand(uint8_t nargs, char* argv[]) {
	if (cmp(argv[0], "test")) {
		sdread(atol(argv[1]));
		//XPCC_LOG_DEBUG .printf("%d\n", qController.mpu.getAccelerationZ());
	} else if (cmp(argv[0], "radio")) {
		printf("Freq: %d\n", radio.freq.get());
		printf("modemCfg: %d\n", radio.modemCfg);
		printf("FH channels: %d\n", radio.fhChannels);
		printf("TXPow: %d\n", radio.txPow);

		//XPCC_LOG_DEBUG .printf("%d\n", qController.mpu.getAccelerationZ());
	} else if (cmp(argv[0], "freq")) {
		uint32_t f = atol(argv[1]);
		XPCC_LOG_DEBUG .printf("%d\n", f);
		radio.setFrequency(f);
	}
	else if (cmp(argv[0], "showall")) {
		AP_Param::show_all(hal.console);
	}
	else if (cmp(argv[0], "i2r")) {
		uint8_t addr = toInt(argv[1]);
		uint8_t reg = toInt(argv[2]);

		uint8_t buf[3];
		buf[0] = reg;

		xpcc::I2cWriteReadAdapter adapter;
		adapter.initialize(addr, buf, 1, buf, 1);

		if(!I2cMaster2::start(&adapter)) {
			ios.printf("failed to start\n");
		}
		while(adapter.isBusy());

		ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(),
				buf[0]);

	} else if (cmp(argv[0], "i2w")) {
		uint8_t addr = toInt(argv[1]);
		uint8_t reg = toInt(argv[2]);
		uint8_t val = toInt(argv[3]);

		uint8_t buf[3];
		buf[0] = reg;
		buf[1] = val;

		xpcc::I2cWriteReadAdapter adapter;
		adapter.initialize(addr, buf, 2, buf, 0);

		I2cMaster2::start(&adapter);
		while(adapter.isBusy());

		ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(),
				buf[0]);

	}

	else if (cmp(argv[0], "scan")) {
		xpcc::I2cWriteAdapter adapter;

		uint8_t buf[3] = { 0x0F };

		XPCC_LOG_DEBUG << "Scanning i2c bus\n";
		for (int i = 0; i < 128; i++) {
			adapter.initialize(i, buf, 1);
			I2cMaster2::start(&adapter);
			while(adapter.isBusy());

			if (I2cMaster2::getErrorState()
					!= xpcc::I2cMaster::Error::AddressNack) {
				XPCC_LOG_DEBUG.printf("Found device @ 0x%x\n", i);
			}
		}
	}

	else if (cmp(argv[0], "reset")) {
		NVIC_SystemReset();
	}

	else if (cmp(argv[0], "flash")) {
		LPC_WDT->WDMOD = 0x03;
		LPC_WDT->WDFEED = 0xFF;
	}

	else if(cmp(argv[0], "eeread")) {
		uint16_t addr = toInt(argv[1]);

		uint8_t c = 0;
		if(!eeprom.readByte(addr, c)) {
			printf("Failed\n");
		} else {
			printf("> %02x\n", c);
		}
	}
	else if(cmp(argv[0], "eewrite")) {
		uint16_t addr = toInt(argv[1]);
		uint16_t b = toInt(argv[2]);

		uint8_t c = 0;
		if(!eeprom.writeByte(addr, b)) {
			printf("Failed\n");
		} else {
			printf("OK\n");
		}
	}
	else if (cmp(argv[0], "dump")) {
		if (crashData[0] == 0xFAFA5555) {
			printf("------ HARD FAULT------\n");
			XPCC_LOG_DEBUG.printf("pc  = 0x%08x\n", crashData[1]);
			XPCC_LOG_DEBUG.printf("lr  = 0x%08x\n", crashData[2]);
			printf("------\n");
		} else if (crashData[0] == 0xFAFA4444) {
			printf("------ WDT TIMEOUT------\n");
			XPCC_LOG_DEBUG.printf("pc  = 0x%08x\n", crashData[1]);
			XPCC_LOG_DEBUG.printf("lr  = 0x%08x\n", crashData[2]);
			printf("------\n");
		} else {
			printf("No dump\n");
		}
		crashData[0] = 0;

	} else if (cmp(argv[0], "flashboot")) {
		uint16_t binLen;
		uint16_t len = binLen = toInt(argv[1]);
		uint32_t checksum = toInt(argv[2]);

		if (len < 3000 || len > 4095) {
			printf("ERR: bad length\n");
			return;
		} else {
			printf("OK\n");
		}

		uint32_t flashPos = 0x00000000;

		IAP iap;

		uint16_t pos = 0;
		uint8_t buf[256];
		memset(buf, 0xFF, 256);

		while (len) {

			while (len && pos < sizeof(buf)) {
				int16_t c;
				while(!device.rxAvailable());
				buf[pos] = device.read();
				pos++;
				len--;
			}

			if (pos) {
				iap.findErasePrepareSector(flashPos);
				iap.writeData(flashPos, (unsigned int*) buf, sizeof(buf));
				printf("write %x %d\n", flashPos, pos);
				flashPos += sizeof(buf);
				pos = 0;
				memset(buf, 0xFF, 256);
			}
		}
		uint32_t sum = 0;
		uint8_t* ptr = (uint8_t*) 0x00000000;

		for (int i = 0; i < binLen; i++) {
			sum += *ptr++;
		}
		printf("checksum: %08x %08x\n", sum, checksum);

	} else if (cmp(argv[0], "dumpboot")) {
		XPCC_LOG_DEBUG.dump_buffer((uint8_t*) 0x00000000, 4096);
	} else if (cmp(argv[0], "bootsum")) {
		uint32_t sum = 0;
		uint8_t* ptr = (uint8_t*) 0x00000000;

		for (int i = 0; i < 4096; i++) {
			sum += *ptr++;
		}
		printf("checksum: %08x\n", sum);
	}
}

