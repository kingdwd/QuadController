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
		setFrequency(429.0, 0.05);
		setTxPower(0);
		//  1c,   1f,   20,   21,   22,   23,   24,   25,   2c,   2d,   2e,   58,   69,   6e,   6f,   70,   71,   72
		const RH_RF22::ModemConfig cfg = { 0x99, 0x03, 0x6B, 0x01, 0x31, 0xd6, 0x01, 0x34,
										//  2c,   2d,   2e,   58,   69,   6e,   6f,   70,   71,   72
										   0x40, 0x0a, 0x2d, 0x80, 0x60, 0x0e, 0x56, 0x04, 0x2B, 0x5A }; // GFSK 56, 56

		//setModemRegisters(&cfg);
		setModemConfig(RH_RF22::GFSK_Rb57_6Fd28_8);

		setModeRx();
	}

	void handleTick() {
		if(!transmitting()) {

			if(available()) {
				printf("rx packet\n");

				uint8_t buf[255];
				uint8_t len = 255;
				recv(buf, &len);

				const uint8_t b[] = "OK";

				send(b, sizeof(b));

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
