/*
 * main.cpp
 *
 *  Created on: Feb 28, 2013
 *      Author: walmis
 */

#define TRP_DEBUG

#include <xpcc/architecture.hpp>
#include <xpcc/processing.hpp>

#include "pindefs.hpp"

#include <xpcc/driver/connectivity/usb/USBDevice.hpp>
#include <xpcc/driver/storage/i2c_eeprom.hpp>

#include <xpcc/debug.hpp>
#include <xpcc/math/filter.hpp>

#include <xpcc/io/terminal.hpp>

#include <math.h>

#include "SensorProcessor.hpp"

#include "eeprom/eeprom.hpp"
#include "QuadController.hpp"

#include <RH_RF22.h>
#include "mavlink.hpp"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.4";


#define _DEBUG
//#define _SER_DEBUG

//UARTDevice uart(460800);

USBSerial device(0xffff, 0xf3c4);
xpcc::IOStream stream(device);
xpcc::NullIODevice null;

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(device);
xpcc::log::Logger xpcc::log::debug(device);
xpcc::log::Logger xpcc::log::error(device);
xpcc::log::Logger xpcc::log::warning(device);
#else
#ifdef _SER_DEBUG
xpcc::log::Logger xpcc::log::info(uart);
xpcc::log::Logger xpcc::log::debug(uart);
xpcc::log::Logger xpcc::log::error(uart);
xpcc::log::Logger xpcc::log::warning(uart);
#else
xpcc::log::Logger xpcc::log::info(null);
xpcc::log::Logger xpcc::log::debug(null);
xpcc::log::Logger xpcc::log::error(null);
#endif
#endif

enum PacketType {
	PACKET_RC = 100,
	PACKET_RF_PARAM_SET,
	PACKET_DATA_FIRST, //first data fragment
	PACKET_DATA, //data fragment
	PACKET_DATA_LAST, //last data fragment
	PACKET_ACK
};

struct Packet {
	uint8_t id = PACKET_RC;
	uint8_t seq; //sequence number
	uint8_t ackSeq; //rx acknowledged seq number
} __attribute__((packed));

struct RCPacket : Packet {
	int16_t yawCh;
	int16_t pitchCh;
	int16_t rollCh;
	int16_t throttleCh;
	uint16_t auxCh;
	uint8_t switches;
} __attribute__((packed));

class Radio : TickerTask, public RH_RF22 {
public:
	Radio() : RH_RF22(0, 1) {
		dataPos = 0;
		dataLen = 0;
		seq = 0;
		dataSent = 0;
		lastAckSeq = 0;
	}

	void handleInit() {
		init();

		float freq, afc;
		uint8_t txPow;
		RH_RF22::ModemConfigChoice modemCfg;

		eeprom.get(&EEData::rfFrequency, freq);
		eeprom.get(&EEData::afcPullIn, afc);
		eeprom.get(&EEData::txPower, txPow);
		eeprom.get(&EEData::modemCfg, modemCfg);

		setFrequency(freq, afc);
		setTxPower(txPow);
		setModemConfig(modemCfg);

		setModeRx();
	}

	void handleTick() {
		if(!transmitting()) {

			if(available()) {
				//printf("rx packet\n");
				uint8_t buf[_bufLen];
				uint8_t len = sizeof(buf);
				recv(buf, &len);

				if(len >= sizeof(Packet)) {
					Packet* inPkt = (Packet*)buf;

					switch(inPkt->id) {
					case PACKET_RC:

						break;
					}

					//received packet, send data back
					//if no data is available, send only ack
					if(inPkt->id >= PACKET_RC) {
						lastAckSeq = inPkt->ackSeq;

						if(dataLen) {
							if(lastAckSeq == seq) {
								//send next fragment
								dataPos += dataSent;
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
								dataLen = 0; //packet is sent,
											 //clear dataLen
							}
							hdr->seq = ++seq;
							hdr->ackSeq = inPkt->seq;

							printf("send frag %d %d\n", dataSent, dataPos-sizeof(Packet));

							RH_RF22::send(packetBuf+dataPos-sizeof(Packet),
									dataSent+sizeof(Packet));

						} else { //send only ack
							Packet p;
							p.id = PACKET_ACK;
							p.seq = ++seq;
							p.ackSeq = inPkt->seq;

							RH_RF22::send((uint8_t*)&p, sizeof(Packet));
						}
					}
				}

				//XPCC_LOG_DEBUG .dump_buffer(buf, len);

			}
		}
	}

	bool sendData(const uint8_t* data, uint8_t len) {
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

	uint8_t maxFragment = 64;

	uint8_t packetBuf[255];

	uint8_t dataLen; //packet size
	uint8_t dataSent; //data sent
	uint8_t dataPos;

	uint8_t lastAckSeq; //last acknowledged sequence number

	uint32_t numRetries;

	uint8_t seq;

    inline uint16_t getRxBad() {
    	return _rxBad;
    }

    inline uint16_t getRxGood() {
    	return _rxGood;
    }

    inline uint16_t getTxGood() {
    	return _txGood;
    }

    bool transmitting() {
    	return mode() == RHModeTx;
    }

    bool idle() {
    	return mode() == RHModeIdle;
    }

};

Radio radio;

QuadController qController;

class CmdTerminal : public Terminal {
public:
	CmdTerminal(IODevice& device) : Terminal(device), ios(device) {

	};

protected:
	xpcc::IOStream ios;

	void handleCommand(uint8_t nargs, char* argv[]) {

		if(cmp(argv[0], "pid")) {

			float kp = 0;
			float kd = 0;
			float ki = 0;

			kp = toFloat(argv[1]);
			ki = toFloat(argv[2]);
			kd = toFloat(argv[3]);

			stream.printf("PID Kp=%.3f Ki=%.3f Kd=%.3f\n", kp, ki, kd);

			qController.setRatePID(kp, ki, kd);
		}

		else if(cmp(argv[0], "hpid")) {
			float kp = 0;
			float kd = 0;
			float ki = 0;
			float max = 0.2;


			kp = toFloat(argv[1]);
			ki = toFloat(argv[2]);
			kd = toFloat(argv[3]);
			max = toFloat(argv[4]);

			stream.printf("Height PID Kp=%.3f Ki=%.3f Kd=%.3f max=%.3f\n", kp, ki, kd, max);

			qController.setHeightPID(kp, ki, kd, max, true);

		}
		else if(cmp(argv[0], "test")) {

			//XPCC_LOG_DEBUG .printf("%d\n", qController.mpu.getAccelerationZ());
		}
		else if(cmp(argv[0], "i2r")) {
			uint8_t addr = to_int(argv[1]);
			uint8_t reg = to_int(argv[2]);

			uint8_t buf[3];
			buf[0] = reg;

			xpcc::I2cWriteReadAdapter adapter;
			adapter.initialize(addr, buf, 1, buf, 1);

			I2cMaster2::startBlocking(&adapter);

			ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

		}
		else if(cmp(argv[0], "i2w")) {
			uint8_t addr = to_int(argv[1]);
			uint8_t reg = to_int(argv[2]);
			uint8_t val = to_int(argv[3]);

			uint8_t buf[3];
			buf[0] = reg;
			buf[1] = val;

			xpcc::I2cWriteReadAdapter adapter;
			adapter.initialize(addr, buf, 2, buf, 0);

			I2cMaster2::startBlocking(&adapter);

			ios.printf("status:%d, value: %x\n", I2cMaster2::getErrorState(), buf[0]);

		}

		else if(cmp(argv[0], "throttle")) {
			float t = to_int(argv[1]) / 100.0;
			qController.throttle = t;
		}

		else if(cmp(argv[0], "arm")) {
			if(cmp(argv[1], "true")) {
				qController.arm(true);
			} else {
				qController.arm(false);
			}
		}

		else if(cmp(argv[0], "speed")) {
			float spd[4];
			spd[0] = to_int(argv[1]) / 100.0;
			spd[1] = to_int(argv[2]) / 100.0;
			spd[2] = to_int(argv[3]) / 100.0;
			spd[3] = to_int(argv[4]) / 100.0;

			ios.printf("Motor speed (%.2f,%.2f,%.2f,%.2f)\n", spd[0], spd[1], spd[2], spd[3]);
			qController.setMotorOutput(spd);
		}

		else if(cmp(argv[0], "zero")) {
			ios.printf("Zeroing sensors\n");
			qController.zero();
		}


		else if(cmp(argv[0], "input_calib")) {
			if(cmp(argv[1], "start")) {
				ios.printf("Calibrating\n");
				//qController.pwmInputs.startCalibration();
			} else {
				ios.printf("OK\n");
				//qController.pwmInputs.stopCalibration();
			}
		}



//		else if(cmp(argv[0], "baro")) {
//			if(!qController.baro.initialize(0x77)) {
//				XPCC_LOG_DEBUG .printf("baro init failed\n");
//			} else {
//				XPCC_LOG_DEBUG .printf("baro init OK\n");
//			}
//			XPCC_LOG_DEBUG .dump_buffer((uint8_t*)&qController.baro.calReg, sizeof(qController.baro.calReg));
//		}

		else if(cmp(argv[0], "scan")) {
			xpcc::I2cWriteAdapter adapter;

			uint8_t buf[3] = {0x0F};

			XPCC_LOG_DEBUG << "Scanning i2c bus\n";
			for(int i = 0; i < 128; i++) {
				adapter.initialize(i, buf, 1);
				I2cMaster2::startBlocking(&adapter);

				if(I2cMaster2::getErrorState() != xpcc::I2cMaster::Error::AddressNack) {
					XPCC_LOG_DEBUG .printf("Found device @ 0x%x\n", i);
				}
			}
		}
		else if(cmp(argv[0], "reset")) {
			NVIC_SystemReset();
		}

		else if(cmp(argv[0], "flash")) {

			LPC_WDT->WDFEED = 0x56;
		}
		else if(cmp(argv[0], "compass_stop")) {

			qController.mag.stopCalibration();
		}
		else if(cmp(argv[0], "compass_start")) {

			qController.mag.startCalibration();
		}

		else if(cmp(argv[0], "sendPacket")) {

			radio.sendData((uint8_t*)4, 200);

		}

		else if(cmp(argv[0], "dump")) {
			extern uint32_t crashData[3];
			if(crashData[0]) {
				XPCC_LOG_DEBUG .printf("pc  = 0x%08x\n", crashData[1]);
				XPCC_LOG_DEBUG .printf("lr  = 0x%08x\n", crashData[2]);
				delay_ms(500);
			}
		}
	}

};

CmdTerminal cmd(device);
//CmdTerminal ucmd(uart);


//rf230::Driver<SpiMaster1, radioRst, radioSel, radioSlpTr, radioIrq> radioDriver;

//class SendTask;
//class QuadWireless : public TinyRadioProtocol<typeof(radioDriver), AES_CCM_32> {
//public:
//	typedef TinyRadioProtocol<typeof(radioDriver), AES_CCM_32> Base;
//
//	enum ReqType {
//		SET_THROTTLE = USER_REQ0,
//		DATA_PKT,
//		SET_PID_RATE,
//		SET_PID,
//		ARM,
//		DISARM,
//		NULL_AXIS,
//		SET_ORIENTATION,
//		SET_TRIM,
//		SET_NULLQ,
//		SET_CHANNEL
//	};
//
//	QuadWireless() : Base(radioDriver) {
//		zeroing = false;
//	}
//
//	void prepareBeacon(BeaconFrame& frm) override {
//		strcpy(frm.name, "QuadRotor");
//	}
//
//	void requestHandler(MacFrame &frm, uint16_t address,
//				uint8_t request_type, uint8_t* data, uint8_t len) {
//
//		XPCC_LOG_DEBUG .printf("handle request %d\n", request_type);
//
//	}
//
//	void eventHandler(uint16_t address, EventType event) {
//		if(event == EventType::ASSOCIATION_EVENT) {
//			sendCalibrationData(address);
//		}
//	}
//
//	void sendCalibrationData(uint16_t address) {
//
//		struct {
//			Quaternion<float> qRotationOffset;
//			Quaternion<float> qTrim;
//		} calibData;
//
//		calibData.qRotationOffset = qController.qRotationOffset;
//		calibData.qTrim = qController.qTrim;
//
//		send(address,
//				(uint8_t*) &calibData,
//				sizeof(calibData),
//				QuadWireless::NULL_AXIS, FrameType::DATA,
//				TX_ACKREQ);
//	}
//
//	void dataHandler(MacFrame &frm, FrameHdr& hdr, uint16_t address,
//				uint8_t *data, uint8_t len) {
//
//		XPCC_LOG_DEBUG .printf("handle data %d\n", hdr.req_id);
//
//		switch(hdr.req_id) {
//			case SET_THROTTLE:{
//				uint8_t throttle = data[0];
//				//XPCC_LOG_DEBUG .printf("throttle %d\n", throttle);
//
//				qController.throttle = data[0] / 100.0;
//
//			}
//			break;
//
//			case SET_PID_RATE: {
//				float* d = (float*)data;
//				XPCC_LOG_DEBUG .printf("setratepid %.3f %.3f %.3f\n", d[0], d[1], d[2]);
//				qController.setRatePID(d[0], d[1], d[2]);
//			}
//			break;
//
//			case SET_PID: {
//				float* d = (float*)data;
//				XPCC_LOG_DEBUG .printf("setpid %.3f %.3f %.3f\n", d[0], d[1], d[2]);
//				qController.setPID(d[0], d[1], d[2]);
//			}
//			break;
//
//			case ARM:
//				qController.arm(true);
//			break;
//
//			case DISARM: {
//				qController.arm(false);
//			}
//			break;
//
//			case NULL_AXIS:
//				qController.zeroRotation();
//				zeroing = true;
//			break;
//
//			case SET_ORIENTATION:{
//				float* vals = (float*)data;
//
//				qController.qTarget = Quaternion<float>(vals[0], vals[1], vals[2], vals[3]);
//				//XPCC_LOG_DEBUG << "t " << qController.qTarget << endl;
//			}
//			break;
//
//			case SET_TRIM:{
//				float* vals = (float*)data;
//				auto q = Quaternion<float>(0, vals[0], vals[1]);
//				XPCC_LOG_DEBUG << "trim " << q << "\n";
//				qController.applyTrim(q);
//			}
//			break;
//
//			case SET_CHANNEL: {
//				uint8_t* val = (uint8_t*)data;
//
//				radioDriver.setChannel(*val);
//			}
//
////			case SET_NULLQ:{
////				float* vals = (float*)data;
////				Quaternion<float> q;
////				q.w = vals[0];
////				q.x = vals[1];
////				q.y = vals[2];
////				q.z = vals[3];
////
////				qController.setRotationOffset(q);
////			}
//
////			case GET_CALIBRATION:
////
////				struct {
////					Quaternion<float> rotOfs;
////				} data;
////
////				data.rotOfs = qController.qRotationOffset;
////
////				send(address, (uint8_t*)&data, sizeof(data), GET_CALIBRATION);
//		}
//
//		if(!isAssociated(address)) {
//			associate(address);
//		}
//
//	}
//	friend class SendTask;
//
//protected:
//	bool zeroing;
//
//};

//class SendTask: public TickerTask {
//public:
//	SendTask(QuadWireless *q) :
//			parent(q) {
//	}

	//QuadWireless *parent;
//	void handleTick() {
//		static PeriodicTimer<> t(100);
//
//		if (parent->zeroing && qController.zeroingComplete()) {
//			for (auto node : parent->connectedNodes) {
//				parent->sendCalibrationData(node->address);
//
//				parent->zeroing = false;
//			}
//		}
//
//		if (t.isExpired()) {
//			{
//				//PROFILE();
//				for (auto node : parent->connectedNodes) {
//					qController.fillPacket(qController.packet);
//					parent->send(node->address,
//							(uint8_t*) &qController.packet,
//							sizeof(qController.packet),
//							QuadWireless::DATA_PKT, FrameType::DATA, 0);
//
//				}
//			}
//		}
//	}
//};


//QuadWireless radio;

GPIO__OUTPUT(test, 0, 16);

void idle() {
	test::reset();
	__WFI();
	test::set();
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {
		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
}

void panic(const char* msg) {

}

int main() {
	test::setOutput(false);
	//debugIrq = true;
	ledRed::setOutput(false);
	ledGreen::setOutput(false);

	lpc17::RitClock::initialize();

	lpc17::SysTickTimer::enable();
	//lpc17::SysTickTimer::attachInterrupt(sysTick);

/////
	SpiMaster1::initialize(SpiMaster1::Mode::MODE_0, 8000000);
	Pinsel::setFunc(0, 7, 2); //SCK1
	Pinsel::setFunc(0, 8, 2); //MISO1
	Pinsel::setFunc(0, 9, 2); //MOSI1
/////

/////
	SpiMaster0::initialize(SpiMaster0::Mode::MODE_0, 8000000);
	Pinsel::setFunc(0, 15, 2); //SCK0
	Pinsel::setFunc(0, 17, 2); //MISO0
	Pinsel::setFunc(0, 18, 2); //MOSI0
////

	xpcc::Random::seed();

/////
	lpc17::I2cMaster2::initialize<xpcc::I2cMaster::DataRate::Fast>();
	Pinsel::setFunc(0, 10, 2); //I2C2
	Pinsel::setFunc(0, 11, 2); //I2C2
/////

	usbConnPin::setOutput(true);
	device.connect();

	//initialize eeprom
	eeprom.initialize();

	NVIC_SetPriority(USB_IRQn, 10);
	NVIC_SetPriority(EINT3_IRQn, 0);

	TickerTask::tasksRun(idle);
}
