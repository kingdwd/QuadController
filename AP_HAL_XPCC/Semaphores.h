
#ifndef __AP_HAL_EMPTY_SEMAPHORE_H__
#define __AP_HAL_EMPTY_SEMAPHORE_H__

#include "AP_HAL_XPCC.h"

class XpccHAL::Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
private:
    bool _taken;
};

#endif // __AP_HAL_EMPTY_SEMAPHORE_H__
