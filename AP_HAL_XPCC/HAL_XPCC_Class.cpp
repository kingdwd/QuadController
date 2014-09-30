
#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_XPCC

#include "HAL_XPCC_Class.h"
#include "AP_HAL_Empty_Private.h"

using namespace Empty;

extern UARTDriver uartADriver;
extern UARTDriver uartBDriver;
extern UARTDriver uartCDriver;
extern UARTDriver uartDDriver;
extern UARTDriver uartEDriver;

static Semaphore  i2cSemaphore;
static I2CDriver  i2cDriver(&i2cSemaphore);
static SPIDeviceManager spiDeviceManager;
static AnalogIn analogIn;
static Storage storageDriver;
static GPIO gpioDriver;
static RCInput rcinDriver;
static RCOutput rcoutDriver;
static Scheduler schedulerInstance;
static Util utilInstance;

HAL_XPCC::HAL_XPCC() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        &uartDDriver,
        &uartEDriver,
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void HAL_XPCC::init(int argc,char* const argv[]) const {
    /* initialize all drivers and private members here.
     * up to the programmer to do this in the correct order.
     * Scheduler should likely come first. */
    scheduler->init(NULL);

}

const HAL_XPCC AP_HAL_XPCC;

#endif
