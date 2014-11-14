/*
 * SDIO.hpp
 *
 *  Created on: Oct 30, 2013
 *      Author: walmis
 */

#ifndef SDIO_HPP_
#define SDIO_HPP_

#include "SDCard.h"
#include <fatfs/diskio.h>
#include <fatfs/ff.h>
#include <xpcc/driver/storage/fat.hpp>


template <typename Spi, typename Cs>
class SDCardVolume : public SDCard<Spi,Cs>, public xpcc::fat::PhysicalVolume {

	DSTATUS doInitialize () override {
		XPCC_LOG_DEBUG .printf("%s()\n", __FUNCTION__);

		return RES_OK;
	}

	DSTATUS doGetStatus () override {
		//XPCC_LOG_DEBUG .printf("%s()\n", __FUNCTION__);
		return RES_OK;
	}

	xpcc::fat::Result
	doRead(uint8_t *buffer, int32_t sectorNumber, uint32_t sectorCount) override {
		XPCC_LOG_DEBUG .printf("%s(%d, %d)\n", __FUNCTION__, sectorNumber, sectorCount);

		if(!this->semaphore()->take(200)) {
			XPCC_LOG_ERROR .printf("sd timeout\n");
			return RES_ERROR;
		}

		for(int i = 0; i < sectorCount; i++) {
			if(!this->readSingleBlock(buffer, sectorNumber+i)) {
				this->semaphore()->give();
				return RES_ERROR;
			}
			buffer+=512;
		}

		this->semaphore()->give();

		return RES_OK;
	}

	xpcc::fat::Result
	doWrite(const uint8_t *buffer, int32_t sectorNumber, uint32_t sectorCount) override {
		XPCC_LOG_DEBUG .printf("%s(%d, %d)\n", __FUNCTION__, sectorNumber, sectorCount);

		if(!this->semaphore()->take(200)) {
			XPCC_LOG_ERROR .printf("sd timeout\n");
			return RES_ERROR;
		}

		this->writeStart(sectorNumber, sectorCount);
		while(sectorCount) {
			this->writeData(buffer);
			sectorCount--;
			buffer+=512;
		}
		this->writeStop();

		this->semaphore()->give();
		return RES_OK;
	}

	xpcc::fat::Result
	doIoctl(uint8_t command, uint32_t *buffer) override {
		XPCC_LOG_DEBUG .printf("%s(%d)\n", __FUNCTION__, command);

		if(command == GET_SECTOR_COUNT) {
			*buffer = this->_sectors;
			return RES_OK;
		}

		return RES_OK;
	}

};

#endif /* SDIO_HPP_ */
