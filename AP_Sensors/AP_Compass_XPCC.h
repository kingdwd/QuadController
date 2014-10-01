/*
 * AP_Compass_XPCC.h
 *
 *  Created on: Oct 1, 2014
 *      Author: walmis
 */

#ifndef AP_COMPASS_XPCC_H_
#define AP_COMPASS_XPCC_H_

#include <Compass.h>

class AP_Compass_XPCC : public Compass {
public:
    virtual bool init();

    /// Read the compass and update the mag_ variables.
    ///
    virtual bool read(void);


    /// use spare CPU cycles to accumulate values from the compass if
    /// possible
    virtual void accumulate(void);
};


#endif /* AP_COMPASS_XPCC_H_ */
