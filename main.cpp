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
#include "radio.hpp"
#include "AP_HAL_XPCC/UARTDriver.h"

#include <xpcc/driver/storage/sd/SDCardVolume.hpp>
#include <xpcc/driver/storage/sd/USBMSD_VolumeHandler.hpp>

extern const AP_HAL::HAL& hal;

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.4";

SDCardVolume<SpiMaster1, sdSel> sdCard;
fat::FileSystem fs(&sdCard);

//#define _DEBUG
#define _SER_DEBUG
//UARTDevice uart(460800);

//typedef USBMSD_VolumeHandler MSDHandler;
typedef CoopTask<USBMSD_VolumeHandler, 512> MSDHandler;

Radio radio;
//USBCDCMSD<MSDHandler> usbSerial(0xffff, 0xf3c4, 0, &sdCard);
USB2xCDCMSD<MSDHandler> usb(0xffff, 0xf3c4, 0, &sdCard);

xpcc::IOStream stream(usb.serial1);
xpcc::NullIODevice null;

BufferedUart<Uart3> uartGps(38400, 16, 256);
BufferedUart<Uart0> uart0(460800, 512, 128);
IODeviceWrapper<Uart0> uart0raw;

XpccHAL::UARTDriver uartADriver(0);
XpccHAL::UARTDriver uartBDriver(&uartGps);
XpccHAL::UARTDriver uartCDriver(&radio);
XpccHAL::UARTDriver uartDDriver(0);
XpccHAL::UARTDriver uartEDriver(0);
XpccHAL::UARTDriver uartConsoleDriver(&usb.serial1);

void XpccHAL::UARTDriver::setBaud(uint32_t baud, xpcc::IODevice* device) {
	if(device == &uartGps) {
		uartGps.setBaud(baud);
	}
}

XpccHAL::UARTDriver::flow_control XpccHAL::UARTDriver::get_flow_control(xpcc::IODevice* device) {
	if(device == &radio) {
		return XpccHAL::UARTDriver::flow_control::FLOW_CONTROL_ENABLE;
	}

	return XpccHAL::UARTDriver::flow_control::FLOW_CONTROL_DISABLE;
}

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(usbSerial);
xpcc::log::Logger xpcc::log::debug(usbSerial);
xpcc::log::Logger xpcc::log::error(uart0raw);
xpcc::log::Logger xpcc::log::warning(usbSerial);
#else
#ifdef _SER_DEBUG
xpcc::log::Logger xpcc::log::info(uart0);
xpcc::log::Logger xpcc::log::debug(uart0);
xpcc::log::Logger xpcc::log::error(uart0raw);
xpcc::log::Logger xpcc::log::warning(uart0);
#else
xpcc::log::Logger xpcc::log::info(null);
xpcc::log::Logger xpcc::log::debug(null);
xpcc::log::Logger xpcc::log::error(null);
#endif
#endif

uint8_t buf[512];
void sdread(int block) {
	XPCC_LOG_DEBUG .printf("sd test\n");

	memset(buf, 0, 512);
//
	sdCard.doWrite(buf, 8, 1);
	sdCard.doWrite(buf, 10, 1);
	sdCard.doWrite(buf, 12, 1);
	sdCard.doRead(buf, 0, 1);

	XPCC_LOG_DEBUG .printf("ok\n");

	//XPCC_LOG_DEBUG.dump_buffer(buf, 512);

//	sdCard.doWrite(buf, 10, 1);
//	sdCard.doRead(buf, 10, 1);
//
//	XPCC_LOG_DEBUG.dump_buffer(buf, 512);


}


CmdTerminal terminal(usb.serial1);
//CmdTerminal ucmd(uart);

void dbgset() {
	LPC_GPIO1->FIOSET |= 1<<20;
}

void dbgclr() {
	LPC_GPIO1->FIOCLR |= 1<<20;
}
void dbgtgl() {
	LPC_GPIO1->FIOPIN ^= 1<<20;
}

void idle() {
	//test::toggle();
	//__WFI();
	//test::set();
	//dbgset();
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {
		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}

	dbgtgl();
}

void set_wd_timeout(uint32_t seconds) {
	LPC_WDT->WDTC = (seconds*1000000000UL) /
			(1000000000UL/(CLKPwr::getPCLK(CLKPwr::ClkType::WDT)/4));
}

void wd_init() {
	LPC_WDT->WDMOD = 0x1;
	LPC_WDT->WDFEED = 0xAA;
	LPC_WDT->WDFEED = 0x55;
	NVIC_SetPriority(WDT_IRQn, 0);
	NVIC_EnableIRQ(WDT_IRQn);
}

void panic(const char* msg) {
	printf("PANIC: %s\n", msg);
	while(1);
}

extern void setup();
extern void loop();

bool apm_initialized = false;
class APM final : xpcc::TickerTask {
	void handleTick() {
		//dbgtgl();
		if(!apm_initialized) {
			setup();
			apm_initialized = true;
		} else {
			//dbgset();
			loop();
			//dbgclr();
		}
	}
};


class Test : xpcc::TickerTask {
public:
	void handleTick() {
		while(1) {
			XPCC_LOG_DEBUG .printf("aaaaa\n");
			xpcc::sleep(100);
			ledBlue::toggle();
			XPCC_LOG_DEBUG .printf("bbbbb %x\n", LPC_SSP1->CR0);
			xpcc::sleep(100);
		}
	}
};


//CoopWrapper<MainTest, 512> testas;
//MainTest test;
//CoopTask<Test, 512> testTask;
//Test test2;

const APM apm;

int main() {
	//set uart0 pins
	Pinsel::setFunc(0, 2, 1);
	Pinsel::setFunc(0, 3, 1);
	usbConnPin::setOutput(false);

	XPCC_LOG_DEBUG .printf("----- starting -----\n");
	XPCC_LOG_DEBUG .printf("sdCard %x\n", &sdCard);

	LPC_GPIO1->FIODIR |= 1<<20;
	set_wd_timeout(6);
	wd_init();

	NVIC_SetPriority(USB_IRQn, 4);
	NVIC_SetPriority(DMA_IRQn, 0);
	NVIC_SetPriority(I2C2_IRQn, 2);
	//NVIC_SetPriority(EINT3_IRQn, 2);
	NVIC_SetPriority(UART0_IRQn, 5);

	//debugIrq = true;
	ledRed::setOutput(true);
	ledGreen::setOutput(true);
	ledBlue::setOutput(0);

	lpc17::RitClock::initialize();

	lpc17::SysTickTimer::enable();
	//lpc17::SysTickTimer::attachInterrupt(sysTick);

/////
	SpiMaster1::initialize(SpiMaster1::Mode::MODE_0, 1000000);
	Pinsel::setFunc(0, 7, 2); //SCK1
	Pinsel::setFunc(0, 8, 2); //MISO1
	Pinsel::setFunc(0, 9, 2); //MOSI1
/////

	if(sdCard.initialise()) {
		fs.mount();
	} else {
		XPCC_LOG_ERROR .printf("SD init failed\n");
	}

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

	lpc17::ADC::init(10000);
	lpc17::ADC::burstMode(true);

	//set uart3(gps) pins
	Pinsel::setFunc(0, 0, 2);
	Pinsel::setFunc(0, 1, 2);

	usb.connect();
	usbConnPin::set();

	hal.init(0, 0);

	set_wd_timeout(1);
	TickerTask::tasksRun(idle);
}
