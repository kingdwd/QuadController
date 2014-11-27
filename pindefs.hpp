/*
 * pindefs.hpp
 *
 *  Created on: Mar 15, 2013
 *      Author: walmis
 */

#ifndef PINDEFS_HPP_
#define PINDEFS_HPP_

#include <xpcc/architecture.hpp>


//GPIO__INPUT(progPin, 2, 0);

GPIO__OUTPUT(usbConnPin, 1, 18);

GPIO__OUTPUT(ledRed, 1, 1);
GPIO__OUTPUT(ledGreen, 1, 4);
GPIO__OUTPUT(ledBlue, 1, 0);

GPIO__OUTPUT(usnd0_trig, 1, 28);
GPIO__INPUT(usnd0_echo, 0, 0);

GPIO__OUTPUT(sdSel, 0, 6);

//si4432 radio
GPIO__IO(radio_sel, 0, 16);
GPIO__IO(radio_irq, 0, 22);

GPIO__INPUT(mpu_irq, 2, 6);

typedef xpcc::lpc17::SpiMaster0 radioSpiMaster;

#endif /* PINDEFS_HPP_ */
