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

extern const AP_HAL::HAL& hal;

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


Radio radio;

CmdTerminal terminal(device);
//CmdTerminal ucmd(uart);

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

extern void loop();
class APM : xpcc::TickerTask {
	void handleTick() {
		loop();
	}
};

const APM apm;

int main() {
	wd_init();

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

	hal.init(0, 0);

	extern void setup();
	setup();

	TickerTask::tasksRun(idle);
}
