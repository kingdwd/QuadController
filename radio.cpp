/*
 * radio.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: walmis
 */

#include "radio.hpp"
#include "eeprom/eeprom.hpp"
#include "pindefs.hpp"

void Radio::handleInit() {
	if(!init()) {
		printf("radio init failed\n");
	}

	eeprom.get(&EEData::rfFrequency, freq);
	eeprom.get(&EEData::afcPullIn, afc);
	eeprom.get(&EEData::txPower, txPow);
	eeprom.get(&EEData::modemCfg, modemCfg);
	eeprom.get(&EEData::fhChannels, fhChannels);

	if(afc > 0.159375f || afc < 0.0f) {
		afc = 0.05;
	}

	setFHStepSize(10);
	setFrequency(freq, afc);
	setTxPower(txPow);
	setModemConfig(modemCfg);

	setModeRx();
}
static ProfileTimer pf;
void Radio::handleTick() {
	if(!transmitting()) {
		if(available()) {
			uint8_t buf[255];
			uint8_t len = sizeof(buf);
			recv(buf, &len);

			if(len >= sizeof(Packet)) {
				Packet* inPkt = (Packet*)buf;

				switch(inPkt->id) {
				case PACKET_RC:
					if(len == sizeof(RCPacket)) {
						rcData = *((RCPacket*)inPkt);
						rcPacketTimestamp = Clock::now();
					}
					break;
				case PACKET_RF_PARAM_SET:{
					RadioCfgPacket* cfg = (RadioCfgPacket*)inPkt;

					printf("--Radio parameters received--\n");
					printf("Freq %f\n", cfg->frequency);
					printf("AfcPullIn %f\n", cfg->afcPullIn);
					printf("Modem setting %d\n", cfg->modemCfg);
					printf("FH Channels %d\n", cfg->fhChannels);
					printf("TX Power %d\n", cfg->txPower);

					//TODO: CHECK Values

					fhChannels = cfg->fhChannels;

					if(freq != cfg->frequency || afc != cfg->afcPullIn) {
						freq = cfg->frequency;
						afc = cfg->afcPullIn;
						setFrequency(cfg->frequency, cfg->afcPullIn);

						eeprom.put(&EEData::rfFrequency, freq);
						eeprom.put(&EEData::afcPullIn, afc);
					}

					if(modemCfg != cfg->modemCfg) {
						modemCfg = (RH_RF22::ModemConfigChoice)cfg->modemCfg;
						setModemConfig(modemCfg);
						eeprom.put(&EEData::modemCfg, cfg->modemCfg);
					}
					if(txPow != cfg->txPower) {
						txPow = cfg->txPower;
						setTxPower(txPow);
						eeprom.put(&EEData::txPower, cfg->txPower);
					}

					eeprom.put(&EEData::fhChannels, cfg->fhChannels);

				}
					break;
				}

				//received packet, send data back
				//if no data is available, send only ack
				if(inPkt->id >= PACKET_RC) {
					//lastAckSeq = inPkt->ackSeq; //set last acknowledged ours packet
					if(fhChannels)
						setFHChannel((inPkt->seq^0x55) % fhChannels);

					if(dataLen) {
						if(inPkt->ackSeq == seq) {
							//send next fragment
							dataPos += dataSent;
							if((dataPos - sizeof(Packet)) == dataLen) {
								dataLen = 0;
								goto sendack;
							}
						} else {
							numRetries++;
						}

						Packet *hdr = (Packet*)(packetBuf+dataPos-sizeof(Packet));

						dataSent = dataLen - (dataPos - sizeof(Packet));
						uint8_t m = maxFragment - sizeof(Packet);
						if(dataSent > m) {
							dataSent = m;
							if(dataPos == sizeof(Packet)) {
								hdr->id = PACKET_DATA_FIRST;
							} else {
								hdr->id = PACKET_DATA;
							}

						} else {
							hdr->id = PACKET_DATA_LAST;
						}
						hdr->seq = ++seq;
						hdr->ackSeq = inPkt->seq; //acknowledge received packet

						printf("send frag %d %d\n", dataSent, dataPos-sizeof(Packet));

						RH_RF22::send(packetBuf+dataPos-sizeof(Packet),
								dataSent+sizeof(Packet));

					} else { //send only ack
sendack:
						Packet p;
						p.id = PACKET_ACK;
						p.seq = ++seq;
						p.ackSeq = inPkt->seq; //acknowledge received packet
						//delay_ms(2);
						RH_RF22::send((uint8_t*)&p, sizeof(Packet));
					}
				}
			}

			//XPCC_LOG_DEBUG .dump_buffer(buf, len);

		}
	}
}

bool Radio::sendPacket(const uint8_t* data, uint8_t len) {
	if(!dataLen) {
		dataLen = len;
		if(dataLen > sizeof(packetBuf)-sizeof(Packet))
			dataLen = sizeof(packetBuf)-sizeof(Packet);
		dataSent = 0;
		dataPos = sizeof(Packet);

		memcpy(packetBuf+dataPos, data, dataLen);
		return true;
	}

	return false;
}

uint8_t Radio::spiBurstWrite0(uint8_t reg, const uint8_t* src, uint8_t len) {
    uint8_t status = 0;

    digitalWrite(_slaveSelectPin, LOW);
    status = radioSpiMaster::write(reg | RH_SPI_WRITE_MASK);

    while(len) {
    	uint8_t written = radioSpiMaster::burstWrite(src, len);
    	while(!radioSpiMaster::txFifoEmpty()) {
    		//TickerTask::yield();
    	}
    	len -= written;
    	src += written;
    }
    radioSpiMaster::flushRx();
    digitalWrite(_slaveSelectPin, HIGH);
    return status;
}

uint8_t Radio::spiBurstRead0(uint8_t reg, uint8_t* dest, uint8_t len) {
    uint8_t status = 0;

    digitalWrite(_slaveSelectPin, LOW);

    status = radioSpiMaster::write(reg & ~RH_SPI_WRITE_MASK); // Send the start address with the write mask off

    radioSpiMaster::flushRx();

    while(len) {
    	uint8_t n = radioSpiMaster::burstWrite(dest, len);
    	//wait until transfer finishes
    	while(radioSpiMaster::isBusy()) {
    		//TickerTask::yield();
    	}
    	radioSpiMaster::burstRead(dest, len);

    	len -= n;
    	dest += n;
    }
    digitalWrite(_slaveSelectPin, HIGH);

    return status;

}

void Radio::handleTxComplete() {
	setModeRx();
}
