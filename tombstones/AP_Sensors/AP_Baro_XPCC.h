/*
 * AP_Baro_XPCC.h
 *
 *  Created on: Oct 1, 2014
 *      Author: walmis
 */

#ifndef AP_BARO_XPCC_H_
#define AP_BARO_XPCC_H_

#include <AP_Baro.h>

class AP_Baro_XPCC : public AP_Baro {
public:
    bool            init();
    uint8_t         read();
    float           get_pressure(); // in mbar*100 units
    float           get_temperature(); // in celsius degrees

};

#endif /* AP_BARO_XPCC_H_ */
