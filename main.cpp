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

extern const AP_HAL::HAL& hal;

using namespace xpcc;
using namespace xpcc::lpc17;

const char fwversion[16] __attribute__((used, section(".fwversion"))) = "QuadV0.4";


#define _DEBUG
//#define _SER_DEBUG
//UARTDevice uart(460800);

Radio radio;
USBSerial usbSerial(0xffff, 0xf3c4);
xpcc::IOStream stream(usbSerial);
xpcc::NullIODevice null;

BufferedUart<Uart3> uartGps(115200, 256, 16);
BufferedUart<Uart0> uart0(115200, 512, 128);
IODeviceWrapper<Uart0> uart0raw;

XpccHAL::UARTDriver uartADriver(0);
XpccHAL::UARTDriver uartBDriver(&uartGps);
XpccHAL::UARTDriver uartCDriver(&radio);
XpccHAL::UARTDriver uartDDriver(0);
XpccHAL::UARTDriver uartEDriver(0);
XpccHAL::UARTDriver uartConsoleDriver(&usbSerial);

void XpccHAL::UARTDriver::setBaud(uint32_t baud, xpcc::IODevice* device) {
	if(device == &uartGps) {
		XPCC_LOG_DEBUG .printf("baud %d\n", baud);
		//uartGps.setBaud(baud);
	}
}

#ifdef _DEBUG
xpcc::log::Logger xpcc::log::info(usbSerial);
xpcc::log::Logger xpcc::log::debug(usbSerial);
xpcc::log::Logger xpcc::log::error(uart0raw);
xpcc::log::Logger xpcc::log::warning(usbSerial);
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




CmdTerminal terminal(usbSerial);
//CmdTerminal ucmd(uart);

void dbgset() {
	LPC_GPIO1->FIOSET |= 1<<20;
}

void dbgclr() {
	LPC_GPIO1->FIOCLR |= 1<<20;
}

void idle() {
	//test::toggle();
	//__WFI();
	//test::set();
	dbgset();
	static PeriodicTimer<> t(500);

	if(t.isExpired()) {
		LPC_WDT->WDFEED = 0xAA;
		LPC_WDT->WDFEED = 0x55;
	}
	dbgclr();

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

class APM final : xpcc::TickerTask {
	void handleTick() {
		loop();
	}
};

const APM apm;

int main() {
	LPC_GPIO1->FIODIR |= 1<<20;
	set_wd_timeout(15);
	wd_init();

	NVIC_SetPriority(USB_IRQn, 4);
	//NVIC_SetPriority(EINT3_IRQn, 2);
	//NVIC_SetPriority(UART0_IRQn, 5);
	//NVIC_SetPriority(I2C2_IRQn, 0);

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

	lpc17::ADC::init();
	lpc17::ADC::burstMode(true);

	//set uart0 pins
	Pinsel::setFunc(0, 2, 1);
	Pinsel::setFunc(0, 3, 1);

	//set uart3(gps) pins
	Pinsel::setFunc(0, 0, 2);
	Pinsel::setFunc(0, 1, 2);

	usbConnPin::setOutput(true);
	usbSerial.connect();

	//initialize eeprom
	eeprom.initialize();

	hal.init(0, 0);

	setup();

	set_wd_timeout(1);
	TickerTask::tasksRun(idle);
}
