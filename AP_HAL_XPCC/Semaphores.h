
#ifndef __AP_HAL_EMPTY_SEMAPHORE_H__
#define __AP_HAL_EMPTY_SEMAPHORE_H__

#include "AP_HAL_XPCC.h"

class XpccHAL::Semaphore final : public AP_HAL::Semaphore {
public:
    Semaphore() : _taken(false) {}
    bool give();
    bool take(uint32_t timeout_ms);
    bool take_nonblocking();
    bool take_async(AP_HAL::MemberProc callback);
private:

    volatile bool _taken;
    bool _take_from_mainloop(uint32_t timeout_ms);
    bool _take_nonblocking();
};

#endif // __AP_HAL_EMPTY_SEMAPHORE_H__
