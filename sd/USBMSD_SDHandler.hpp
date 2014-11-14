/*
 * USBMSD_SDHandler.hpp
 *
 *  Created on: Oct 31, 2013
 *      Author: walmis
 */

#ifndef USBMSD_SDHANDLER_HPP_
#define USBMSD_SDHANDLER_HPP_

#include <xpcc/architecture.hpp>
#include "SDCard.h"

namespace xpcc {


template <class SDCardAsync>
class USBMSD_SDHandler : public USBMSDHandler, TickerTask {
public:
	USBMSD_SDHandler(uint8_t bulkIn = EPBULK_IN,
                    uint8_t bulkOut = EPBULK_OUT) :
                            USBMSDHandler(bulkIn, bulkOut) {
		haveBlock = -1;
		readBlock = -1;
		requestedBlock = -1;
		card = 0;
		writeBusy = false;
	}

	void assignCard(SDCardAsync& card) {
		this->card = &card;
	}

protected:
    volatile uint32_t requestedBlock;

    uint8_t buffer[512];
    uint8_t *dataptr;
    uint32_t blocksLeft;

    TransferType opType;
    uint32_t totalBlocks;
    volatile bool firstBlock;
    volatile int32_t haveBlock;
    volatile int32_t readBlock;
    volatile bool writeBusy;


    void handleTick() override {

    	if(readBlock != -1) {
			if(card->readData(buffer, 512)) {
				haveBlock = readBlock;
			}
			readBlock = -1;
    	}

    	if(requestedBlock != -1) {

    		bool res;

    		switch(opType) {
    		case READ:
    			if(firstBlock) {
    	    		if(!card->semaphore()->take_nonblocking()) {
    	    			XPCC_LOG_DEBUG .printf("*");
    	    			return;
    	    		}
    				XPCC_LOG_DEBUG .printf("TestMSD::read_begins %d a:%d d:%d\n", opType, requestedBlock, totalBlocks);
    				card->readStart(requestedBlock);
    				firstBlock = false;
    			}

    			if(haveBlock != requestedBlock) {
    				//XPCC_LOG_DEBUG.printf("-");

    				res = card->readData(dataptr, 512);

    			} else {
    				//XPCC_LOG_DEBUG.printf("+");
    				memcpy(dataptr, buffer, 512);
    				haveBlock = -1;
    				res = true;
    			}

    			if(blocksLeft == 0) {
    				card->readStop();
    				card->semaphore()->give();
    			} else {
    				//read in advance
    				readBlock = requestedBlock+1;
    			}

    			requestedBlock = -1;

				if(res) {
					disk_read_finalize(true);
				} else {
					disk_read_finalize(false);
				}

    			break;

    		case WRITE:
    			if(firstBlock) {
    	    		if(!card->semaphore()->take_nonblocking()) {
    	    			XPCC_LOG_DEBUG .printf("*");
    	    			return;
    	    		}
    				card->writeStart(requestedBlock, totalBlocks);
    				firstBlock = false;
    			}

    			memcpy(buffer, dataptr, 512);
    			requestedBlock = -1;
    			size_t left = blocksLeft;
    			disk_write_finalize(true);

    			res = card->writeData(buffer);

    			if(left == 0) {
    				card->writeStop();
    				card->semaphore()->give();
    			}
    			break;
    		}
    	}
//
//		if (requestedBlock != -1 && haveBlock == -1 && readingBlock == -1
//				&& !card->isOpInprogress()) {
//			bool res;
//			if (opType == READ) {
//				if (!card->readStart(requestedBlock)) {
//					disk_read_finalize(false);
//					requestedBlock = -1;
//					return;
//				}
//
//				readingBlock = requestedBlock;
//	    		XPCC_LOG_DEBUG .printf("X:first read %d\n", requestedBlock);
//	      		card->readAsync(buffer, 512).attach([this]{ this->onBlockRead(); });
//
//			} else if (opType == WRITE) {
//				if (!card->writeStart(requestedBlock, totalBlocks)) {
//					requestedBlock = -1;
//					disk_write_finalize(false);
//					return;
//				}
//
//				XPCC_LOG_DEBUG.printf("X: first write %d\n", requestedBlock);
//				readingBlock = requestedBlock;
//
//				uint8_t token = (totalBlocks>1)? WRITE_MULTIPLE_TOKEN : DATA_START_BLOCK;
//				card->writeDataAsync(token, buffer).attach([this]{ this->onBlockWrite(); });
//			}
//    	}
//
//    	//if(requestedBlock != -1 && v.isExpired()){
//    	if(haveBlock == requestedBlock && requestedBlock != -1 ) {
//    		//xpcc::atomic::Lock lock;
//    		if(opType == TransferType::READ) {
//				memcpy(dataptr, buffer, 512);
//
//				if(blocksLeft > 0) {
//					readingBlock = requestedBlock+1;
//					haveBlock = -1;
//					//XPCC_LOG_DEBUG .printf("X:read %d\n", readingBlock);
//					card->readAsync(buffer, 512).attach([this]{ this->onBlockRead(); });
//				} else {
//					//XPCC_LOG_DEBUG .printf("X:stop read\n");
//					card->readStop();
//				}
//				requestedBlock = -1;
//
//
//
//				disk_read_finalize(true);
//				//XPCC_LOG_DEBUG.printf("---\n");
//			} else
//    		if(opType == TransferType::WRITE) {
//    			if(blocksLeft == 0) {
//    				card->writeStop();
//    			}
//    			requestedBlock = -1;
//    			XPCC_LOG_DEBUG.printf("X: finalizing write\n");
//    			disk_write_finalize(true);
//    		}
//    	}
    }

    void transfer_begins(TransferType type, uint32_t startBlock, int numBlocks) override{
		requestedBlock = -1;
		opType = type;
		totalBlocks = numBlocks;
		firstBlock = true;
    }

    int disk_read_start(uint8_t * data, uint32_t block, uint32_t blocksLeft) override {
    	requestedBlock = block;
    	dataptr = data;
    	this->blocksLeft = blocksLeft;

		return 0;
    }

    int disk_write_start(const uint8_t * data, uint32_t block, uint32_t blocksLeft) override {
    	requestedBlock = block;
    	dataptr = (uint8_t*)data;
    	this->blocksLeft = blocksLeft;

        return 0;
    }

    int disk_initialize() override {
            XPCC_LOG_DEBUG .printf("disk initialize()\n");

            return DISK_OK;
    }

    uint32_t disk_sectors() override {
    	//return card->numSectors();
    	XPCC_LOG_DEBUG .printf("req: card sector -> %d\n", card->numSectors());
    	return card->numSectors();
    }

    uint16_t disk_sector_size() override {
            return 512;
    }

    int disk_status() override {
    	//return NO_DISK;
    	//return DISK_OK | WRITE_PROTECT;
    	if(!card)
    		return NO_DISK;

    	if(card->isInitialised()) {
            return DISK_OK /*| WRITE_PROTECT*/;
    	} else {
    		return NO_DISK;
    	}
    }

protected:

	SDCardAsync* card;
};

}


#endif /* USBMSD_SDHANDLER_HPP_ */
