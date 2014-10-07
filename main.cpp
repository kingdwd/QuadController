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

#include "cmd_terminal.hpp"

#include <math.h>

#include "eeprom/eeprom.hpp"


#include <RH_RF22.h>
#include "mavlink.hpp"
#include "radio.hpp"
#include "AP_HAL_XPCC/UARTDriver.h"

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.4";


#define _DEBUG
//#define _SER_DEBUG
//UARTDevice uart(460800);

USBSerial device(0xffff, 0xf3c4);
xpcc::IOStream stream(device);
xpcc::NullIODevice null;

BufferedUart<Uart0> uart0(115200, 512, 128);

XpccHAL::UARTDriver uartADriver(&uart0);
XpccHAL::UARTDriver uartBDriver(0);
XpccHAL::UARTDriver uartCDriver(0);
XpccHAL::UARTDriver uartDDriver(0);
XpccHAL::UARTDriver uartEDriver(0);
XpccHAL::UARTDriver uartConsoleDriver(&device);

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


Mavlink mavlink;
Radio radio;

//QuadController qController;



CmdTerminal terminal(device);
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

GPIO__OUTPUT(test, 1, 18);

#include "AP_HAL_XPCC/Semaphores.h"

XpccHAL::Semaphore sem;


void idle() {
	//test::toggle();
	//__WFI();
	//test::set();

	static PeriodicTimer<> t(500);

	if(t.isExpired()) {
		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}

}

void wd_init() {
	LPC_WDT->WDMOD = 0x1;
	LPC_WDT->WDTC = 5000000;
	LPC_WDT->WDFEED = 0xAA;
	LPC_WDT->WDFEED = 0x55;
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);
}

void panic(const char* msg) {
	printf("PANIC: %s\n", msg);
	while(1);
}

int main() {
	wd_init();

	test::setOutput(false);
	//debugIrq = true;
	ledRed::setOutput(true);
	ledGreen::setOutput(true);

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

	lpc17::ADC::init();
	lpc17::ADC::burstMode(true);

	//initialize eeprom
	eeprom.initialize();

	NVIC_SetPriority(USB_IRQn, 10);
	NVIC_SetPriority(EINT3_IRQn, 0);

	TickerTask::tasksRun(idle);
}
