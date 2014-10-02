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

GPIO__OUTPUT(usbConnPin, 1, 30);

GPIO__OUTPUT(ledRed, 0, 25);
GPIO__OUTPUT(ledGreen, 0, 26);

GPIO__OUTPUT(usnd0_trig, 1, 28);
GPIO__INPUT(usnd0_echo, 0, 0);

//GPIO__OUTPUT(radioRst, 4, 28);
//GPIO__OUTPUT(radioSel, 2, 0);
//GPIO__IO(radioSlpTr, 0, 6);
//GPIO__INPUT(radioIrq, 2, 1);

//si4432 radio
GPIO__IO(radio_sel, 0, 16);
GPIO__IO(radio_irq, 0, 22);

typedef xpcc::lpc17::SpiMaster0 radioSpiMaster;

#endif /* PINDEFS_HPP_ */
