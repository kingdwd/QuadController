/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory
 */

#include <AP_HAL.h>

#include <AP_HAL_XPCC/DataFlash_Xpcc.h>
#include <xpcc/architecture.hpp>
#include <fatfs/diskio.h>
#include <fatfs/ff.h>


extern const AP_HAL::HAL& hal;
extern xpcc::fat::FileSystem fs;

#define MAX_LOG_FILES 500U
#define DATAFLASH_PAGE_SIZE 1024UL

/*
  constructor
 */
void DataFlash_Xpcc::Init(const struct LogStructure *structure, uint8_t num_types)
{
    DataFlash_Class::Init(structure, num_types);
    XPCC_LOG_INFO << "dataflash init\n";
    // reserve last page for config information

//	uint32_t size;
//	if(!fs.volume()->doIoctl(GET_SECTOR_COUNT, &size)) {
//
//	}

}

bool DataFlash_Xpcc::NeedErase(void){

	return false;
}

void DataFlash_Xpcc::WriteBlock(const void* pBuffer, uint16_t size) {
	//XPCC_LOG_DEBUG .printf("log wr %d\n", size);
}

uint16_t DataFlash_Xpcc::find_last_log(void) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
	return 0;
}

void DataFlash_Xpcc::get_log_boundaries(uint16_t log_num, uint16_t& start_page,
		uint16_t& end_page) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
}

void DataFlash_Xpcc::get_log_info(uint16_t log_num, uint32_t& size,
		uint32_t& time_utc) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
}

int16_t DataFlash_Xpcc::get_log_data(uint16_t log_num, uint16_t page,
		uint32_t offset, uint16_t len, uint8_t* data) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
}

uint16_t DataFlash_Xpcc::get_num_logs(void) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
	return 0;
}

void DataFlash_Xpcc::LogReadProcess(uint16_t log_num, uint16_t start_page,
		uint16_t end_page,
		void (*printMode)(AP_HAL::BetterStream* port, uint8_t mode),
		AP_HAL::BetterStream* port) {
	XPCC_LOG_INFO << __PRETTY_FUNCTION__ << xpcc::endl;
}

void DataFlash_Xpcc::DumpPageInfo(AP_HAL::BetterStream* port) {
}

void DataFlash_Xpcc::ShowDeviceInfo(AP_HAL::BetterStream* port) {
}

void DataFlash_Xpcc::ListAvailableLogs(AP_HAL::BetterStream* port) {
}

uint16_t DataFlash_Xpcc::start_new_log(void) {

//	file_wr = new xpcc::fat::File;
//	FRESULT res;
//	if((res = file_wr->open("1.BIN", "w")) != FR_OK) {
//		XPCC_LOG_INFO .printf("failed to open log %d\n", res);
//	}
//
//	*file_wr << "Hello\n";
//	file_wr->close();



	return 0;
}

void DataFlash_Xpcc::ReadManufacturerID() {
}

bool DataFlash_Xpcc::CardInserted() {
	return true;
}
