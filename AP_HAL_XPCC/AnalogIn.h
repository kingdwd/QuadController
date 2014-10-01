
#ifndef __AP_HAL_EMPTY_ANALOGIN_H__
#define __AP_HAL_EMPTY_ANALOGIN_H__

#include "AP_HAL_XPCC.h"

class XpccHAL::AnalogSource : public AP_HAL::AnalogSource {
public:
    AnalogSource(uint8_t chan);
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric() { return voltage_average(); }

private:
    uint8_t _chan;
};

class XpccHAL::AnalogIn : public AP_HAL::AnalogIn {
public:
    AnalogIn();
    void init(void* implspecific);
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);

private:
    AP_HAL::AnalogSource* channels[8];
};
#endif // __AP_HAL_EMPTY_ANALOGIN_H__
