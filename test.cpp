/*
 * test.cpp
 *
 *  Created on: Sep 26, 2014
 *      Author: walmis
 */

#include <xpcc/architecture.hpp>
#include <AP_MotorsQuad.h>
#include <RC_Channel.h>
#include "AP_HAL_XPCC/AP_HAL_XPCC.h"
#include <stdio.h>
#include <xpcc/container.hpp>

#include "AP_HAL_XPCC/UARTDriver.h"

using namespace xpcc::lpc17;

extern const AP_HAL::HAL& hal;// = AP_HAL_XPCC;

//RC_Channel c(0);

//AP_MotorsQuad motors(c, c, c, c, 100);

extern void setup();
extern void loop();


void tm() {

}

bool init = 0;

class AP : xpcc::TickerTask {
	void handleTick() {
		if(init) {

			loop();
			LPC_GPIO1->FIOSET = 1<<18;

			LPC_GPIO1->FIOCLR = 1<<18;
		}
	}
};

AP ap;
void ap_test(char *argv[], int argc) {
	if(strcmp(argv[1], "init") == 0) {
		hal.init(0, 0);
		setup();

		init = true;
	}

	if(strcmp(argv[1], "i2ctest") == 0) {

		AP_HAL::Semaphore *sem = hal.i2c->get_semaphore();

		if(!sem->take_nonblocking()) {
			printf("cannot take sem\n");
		}

		uint8_t reg= 0;
		bool res = hal.i2c->readRegister(0x68, 0x75, &reg);

		printf("%d whoami: %x\n", res, reg);

		sem->give();
	}




	//hal.init(0, 0);

//	printf("%d\n", hal.rcin->read(2));
//
//	Pinsel::setFunc(0, 2, 1);
//	Pinsel::setFunc(0, 3, 1);
//
//
//	hal.gpio->write(200, true);
//
//	XPCC_LOG_DEBUG .printf("uart write\n");
//
//	for(int i = 0; i < 256; i++) {
//		hal.uartA->write(i);
//	}
//
//	for(int i = 0; i < 64; i++) {
//		//uart.read();
//	}
//
//	if(strcmp(argv[1], "init")==0){
//		hal.gpio->init();
//		hal.analogin->init(0);
//		hal.rcout->init(0);
//	} else
//		if(strcmp(argv[1], "analog")==0){
//			AP_HAL::AnalogSource *src = hal.analogin->channel(7);
//			if(src)
//				printf("analog read %.5f\n", src->read_latest());
//		}


//	hal.rcout->enable_ch(0);
//	hal.rcout->enable_ch(1);
//	hal.rcout->enable_ch(2);
//	hal.rcout->enable_ch(3);
//
//	hal.rcout->write(0, 1100);
//	hal.rcout->write(1, 1200);
//	hal.rcout->write(2, 1300);
//	hal.rcout->write(3, 1400);
//
//	printf("read %d\n", hal.rcout->read(0));
//	printf("read %d\n", hal.rcout->read(1));
//	printf("read %d\n", hal.rcout->read(2));
//	printf("read %d\n", hal.rcout->read(3));




}
