/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

#include <DataFlash.h>
#include <xpcc/driver/storage/fat.hpp>

class DataFlash_Xpcc : public DataFlash_Class
{
public:

    void WriteBlock(const void *pBuffer, uint16_t size);
    uint16_t find_last_log(void);
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs(void);

    void LogReadProcess(uint16_t log_num,
                                uint16_t start_page, uint16_t end_page,
                                void (*printMode)(AP_HAL::BetterStream *port, uint8_t mode),
                                AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);

    uint16_t start_new_log(void);

    void        Init(const struct LogStructure *structure, uint8_t num_types);
    void        ReadManufacturerID();
    bool        CardInserted();

    bool NeedErase(void);
    void EraseAll() {}
    void ReadBlock(void *pkt, uint16_t size) {};
protected:
    xpcc::fat::File* file_wr;
};

#endif // DataFlash_File_h

