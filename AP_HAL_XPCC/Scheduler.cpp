
#include "Scheduler.h"

using namespace Empty;

extern const AP_HAL::HAL& hal;

Scheduler::Scheduler()
{}

void Scheduler::init(void* machtnichts)
{}

void Scheduler::delay(uint16_t ms)
{}

uint64_t Scheduler::millis64() {
    return 10000;
}

uint64_t Scheduler::micros64() {
    return 200000;
}

uint32_t Scheduler::millis() {
    return millis64();
}

uint32_t Scheduler::micros() {
    return micros64();
}

void Scheduler::delay_microseconds(uint16_t us)
{}

void Scheduler::register_delay_callback(AP_HAL::Proc k,
            uint16_t min_time_ms)
{}

void Scheduler::register_timer_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_io_process(AP_HAL::MemberProc k)
{}

void Scheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

void Scheduler::suspend_timer_procs()
{}

void Scheduler::resume_timer_procs()
{}

bool Scheduler::in_timerprocess() {
    return false;
}

void Scheduler::begin_atomic()
{}

void Scheduler::end_atomic()
{}

bool Scheduler::system_initializing() {
    return false;
}

void Scheduler::system_initialized()
{}

void Scheduler::panic(const prog_char_t *errormsg) {
    hal.console->println_P(errormsg);
    for(;;);
}

void Scheduler::reboot(bool hold_in_bootloader) {
    for(;;);
}