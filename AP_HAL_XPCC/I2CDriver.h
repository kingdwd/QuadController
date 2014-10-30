
#ifndef __AP_HAL_EMPTY_I2CDRIVER_H__
#define __AP_HAL_EMPTY_I2CDRIVER_H__

#include "AP_HAL_XPCC.h"
#include <xpcc/architecture.hpp>
#include <xpcc/architecture/peripheral/i2c_adapter.hpp>


class XpccHAL::I2CDriver : public AP_HAL::I2CDriver, xpcc::I2cWriteReadAdapter {
public:
    I2CDriver(AP_HAL::Semaphore* semaphore) : _semaphore(semaphore) {}
    void begin();
    void end();
    void setTimeout(uint16_t ms);
    void setHighSpeed(bool active);

    /* write: for i2c devices which do not obey register conventions */
    uint8_t write(uint8_t addr, uint8_t len, uint8_t* data);
    /* writeRegister: write a single 8-bit value to a register */
    uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val);
    /* writeRegisters: write bytes to contigious registers */
    uint8_t writeRegisters(uint8_t addr, uint8_t reg,
                                   uint8_t len, uint8_t* data);

    /* read: for i2c devices which do not obey register conventions */
    uint8_t read(uint8_t addr, uint8_t len, uint8_t* data);
    /* readRegister: read from a device register - writes the register,
     * then reads back an 8-bit value. */
    uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data);
    /* readRegister: read contigious device registers - writes the first 
     * register, then reads back multiple bytes */
    uint8_t readRegisters(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data);

    bool readNonblocking(uint8_t addr, uint8_t reg,
                                  uint8_t len, uint8_t* data,
								  AP_HAL::MemberProc callback);

    uint8_t lockup_count();

    AP_HAL::Semaphore* get_semaphore() { return _semaphore; }

private:
    AP_HAL::Semaphore* _semaphore;
    AP_HAL::MemberProc nb_callback;
    volatile bool nb_transaction;

    void stopped(DetachCause cause) override;
};

#endif // __AP_HAL_EMPTY_I2CDRIVER_H__
