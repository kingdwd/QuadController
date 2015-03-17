/*
 * radio.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#include "radio.hpp"
#include "eeprom/eeprom.hpp"
#include "pindefs.hpp"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo Radio::var_info[] {
	    AP_GROUPINFO("FREQUENCY", 0, Radio, freq, 433000),
	    AP_GROUPINFO("FH_CHANS", 1, Radio, fhChannels, 4),
	    AP_GROUPINFO("MODEM_CFG", 2, Radio, modemCfg, 18),
	    AP_GROUPINFO("MAX_FRAGM", 3, Radio, maxFragment, 64),
	    AP_GROUPINFO("TX_POWER", 4, Radio, txPow, 0),
	    AP_GROUPEND
};

void Radio::handleInit() {
	if(!init()) {
		hal.scheduler->panic("radio init failed");
	}

	freq.load();
	modemCfg.load();
	txPow.load();
	fhChannels.load();

	setFHStepSize(10);
	setFrequency(freq.get());
	setTxPower(txPow.get());
	setModemConfig((RH_RF22::ModemConfigChoice)modemCfg.get());

	setModeRx();
}

void Radio::handleTick() {

	if (!transmitting()) {
		if (rxDataLen) {
			uint8_t* buf = rxBuf;
			uint8_t len = rxDataLen;

			noiseFloor = ((uint16_t) noiseFloor * 31 + rssiRead()) / 32;

			if (len >= sizeof(Packet)) {
				Packet* inPkt = (Packet*) buf;

				switch (inPkt->id) {
				case PACKET_RC:
					if(len >= sizeof(RCPacket)) {
						rcData = *((RCPacket*) inPkt);
						rcPacketTimestamp = Clock::now();

						uint8_t payload_len = len - sizeof(RCPacket);
						if(payload_len) {
							if(inPkt->ackSeq == seq) {
								rxbuf.write(buf+sizeof(RCPacket), payload_len);
							} else {
								XPCC_LOG_DEBUG
								.printf("discard duplicate seq:%d ackSeq:%d\n", seq, inPkt->ackSeq);

							}
						}
					}

					break;
				case PACKET_RF_PARAM_SET: {
					RadioCfgPacket* cfg = (RadioCfgPacket*) inPkt;

					printf("--Radio parameters received--\n");
					printf("Freq %d\n", cfg->frequency/1000);
					printf("AfcPullIn %d\n", cfg->afcPullIn/1000);
					printf("Modem setting %d\n", cfg->modemCfg);
					printf("FH Channels %d\n", cfg->fhChannels);
					printf("TX Power %d\n", cfg->txPower);
					setModeIdle();
					if (freq != cfg->frequency/1000) {
						setFrequency(cfg->frequency/1000, cfg->afcPullIn/1000);
					}

					if (cfg->modemCfg != modemCfg.get()) {
						setModemConfig((RH_RF22::ModemConfigChoice) cfg->modemCfg);
					}

					if (txPow.get() != cfg->txPower) {
						setTxPower(cfg->txPower);
					}

					fhChannels.set_and_save_ifchanged(cfg->fhChannels);
					setModeRx();

				}
					break;
				}

				//received packet, send data back
				//if no data is available, send only ack
				if (inPkt->id >= PACKET_RC) {
					//lastAckSeq = inPkt->ackSeq; //set last acknowledged ours packet
					if (fhChannels)
						setFHChannel((inPkt->seq ^ 0x55) % fhChannels);

					//printf("*\n");
					sendAck(inPkt);

				}
			}

			rxDataLen = 0;
		}

		//XPCC_LOG_DEBUG .dump_buffer(buf, len);

	}
}

bool Radio::sendAck(Packet* inPkt) {
	uint16_t txavail = txbuf.bytes_used();
	Packet* out = (Packet*)txBuf;

	if((uint16_t)maxFragment > (RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet))) {
		maxFragment = RH_RF22_MAX_MESSAGE_LEN - sizeof(Packet);
	} else if((uint16_t)maxFragment < 32) {
		maxFragment = 32;
	}
	uint16_t maxFrag = maxFragment;

	out->noise = getNoiseFloor();
	out->rssi = getRssi();

	if(inPkt->ackSeq != seq && out->id == PACKET_DATA) {
		//retry last data transmission
		printf("Retry\n");
		out->seq = ++seq;
		out->ackSeq = inPkt->seq;

		RH_RF22::send(txBuf, dataLen);
	} else {

		out->seq = ++seq;
		out->ackSeq = inPkt->seq; //acknowledge received packet
		out->id = PACKET_DATA;

		if((txavail >= maxFrag) || (latencyTimer.isExpired() && txavail)) {

			uint8_t* buf = txBuf + sizeof(Packet);
			for(int i = 0; i < std::min(maxFrag, txavail); i++) {
				*buf++ = txbuf.read();
			}

			dataLen = buf - txBuf;
			//printf("Send data %d\n", dataLen);
			RH_RF22::send(txBuf, dataLen);

			latencyTimer.restart(latency);
		} else {
			out->id = PACKET_ACK; //no data to send, send ack
			RH_RF22::send(txBuf, sizeof(Packet));
		}
	}

	return true;
}

uint8_t Radio::spiBurstWrite0(uint8_t reg, const uint8_t* src, uint8_t len) {
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);
    status = radioSpiMaster::write(reg | RH_SPI_WRITE_MASK);

    while(len) {
    	uint8_t written = radioSpiMaster::burstWrite(src, len);
    	while(!radioSpiMaster::txFifoEmpty()) {
    		//xpcc::yield();
    	}
    	len -= written;
    	src += written;
    }
    radioSpiMaster::flushRx();
    digitalWrite(_slaveSelectPin, HIGH);
    ATOMIC_BLOCK_END;
    return status;
}

uint8_t Radio::spiBurstRead0(uint8_t reg, uint8_t* dest, uint8_t len) {
    uint8_t status = 0;
    ATOMIC_BLOCK_START;
    digitalWrite(_slaveSelectPin, LOW);

    status = radioSpiMaster::write(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off

    radioSpiMaster::flushRx();

    while(len) {
    	uint8_t n = radioSpiMaster::burstWrite(dest, len);
    	//wait until transfer finishes
    	while(radioSpiMaster::isBusy()) {
    		//xpcc::yield();
    	}
    	radioSpiMaster::burstRead(dest, len);

    	len -= n;
    	dest += n;
    }
    digitalWrite(_slaveSelectPin, HIGH);
    ATOMIC_BLOCK_END;
    return status;

}

void Radio::handleRxComplete() {
	rssi = (rssi * 7 + (uint8_t)lastRssi()) / 8;

	if(available()) {
		if(rxDataLen) {
			XPCC_LOG_DEBUG .printf("packet not cleared\n");
		}
		rxDataLen = sizeof(rxBuf);
		if(!recv(rxBuf, (uint8_t*)&rxDataLen)) {
			rxDataLen = 0;
		}
	}
}

void Radio::handleTxComplete() {
	setModeRx();
}
