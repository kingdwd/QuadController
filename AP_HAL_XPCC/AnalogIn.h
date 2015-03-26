
#ifndef __AP_HAL_EMPTY_ANALOGIN_H__
#define __AP_HAL_EMPTY_ANALOGIN_H__

#include "AP_HAL_XPCC.h"
#include <xpcc/math/filter.hpp>

class XpccHAL::AnalogSource final: public AP_HAL::AnalogSource {
public:
    AnalogSource(int16_t chan);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }

private:
    friend class XpccHAL::AnalogIn;
    void _tick();
    void _irq();

    uint32_t sum;
    uint8_t count;

    float _voltage_latest;
    float _voltage_avg;

    xpcc::filter::Median<uint32_t, 3> filter;

    uint8_t _chan;
};

class XpccHAL::AnalogIn final: public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void) {
    	return 5.0f;
    }

    friend class XpccHAL::Scheduler;

    void irq();
private:
    void _tick();

    XpccHAL::AnalogSource* channels[16];
};
#endif // __AP_HAL_EMPTY_ANALOGIN_H__
