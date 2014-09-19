/*
 * radio.hpp
 *
 *  Created on: Sep 18, 2014
 *      Author: walmis
 */

#ifndef RADIO_HPP_
#define RADIO_HPP_

#include <xpcc/architecture.hpp>
#include <RH_RF22.h>

using namespace xpcc;

class Radio : TickerTask, public RH_RF22 {
public:
	Radio() : RH_RF22(0, 1) {

	}

	void handleInit() {
		init();
		setFrequency(422.0, 0.05);
		setModemConfig(RH_RF22::FSK_Rb57_6Fd28_8);
		setModeRx();
	}

	void handleTick() {
		if(!transmitting()) {

			if(available()) {
				printf("rx packet\n");

				uint8_t buf[255];
				uint8_t len = 255;
				recv(buf, &len);

				XPCC_LOG_DEBUG .dump_buffer(buf, len);

			}
		}
	}

    inline uint16_t getRxBad() {
    	return _rxBad;
    }

    inline uint16_t getRxGood() {
    	return _rxGood;
    }

    inline uint16_t getTxGood() {
    	return _txGood;
    }

    void sendTest() {
    	uint8_t data[] = "Hello World!Hello World!Hello World!Hello World!Hello World!Hello World!Hello World!Hello World!";

    	send(data, sizeof(data));
    	waitPacketSent();

    }

    bool transmitting() {
    	return mode() == RHModeTx;
    }

    bool idle() {
    	return mode() == RHModeIdle;
    }

};


#endif /* RADIO_HPP_ */
