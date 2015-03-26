/*
 * fault.cpp
 *
 *  Created on: Apr 11, 2014
 *      Author: walmis
 */

#include <xpcc/debug.hpp>
#include <xpcc/architecture.hpp>
#include "pindefs.hpp"


enum { r0, r1, r2, r3, r12, lr, pc, psr};

void boot_jump( uint32_t address ){
   __asm("LDR SP, [R0]\n"
   "LDR PC, [R0, #4]");
}


extern "C" __attribute((naked)) void HardFault_Handler(void)
{
	register uint32_t* msp = (uint32_t*)__get_MSP();
	register uint32_t* psp = (uint32_t*)__get_PSP();

	asm volatile("push {lr}");

	XPCC_LOG_ERROR .flush();

	XPCC_LOG_ERROR << "Hard Fault\nMSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", msp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", msp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", msp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", msp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", msp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", msp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", msp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", msp[psr]);

	XPCC_LOG_ERROR << "PSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", psp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", psp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", psp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", psp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", psp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", psp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", psp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", psp[psr]);

	XPCC_LOG_ERROR .flush();

	LPC_WDT->WDMOD = 0x3;
	LPC_WDT->WDFEED = 0xFF;

	while(1) {
		ledRed::set();
		ledGreen::set();
	}

}
//extern "C" void UsageFault_Handler(void)
//{
//  asm volatile("MRS r0, MSP;"
//		       "B Hard_Fault_Handler");
//}
//extern "C" void BusFault_Handler(void)
//{
//  asm volatile("MRS r0, MSP;"
//		       "B Hard_Fault_Handler");
//}

uint32_t crashData[3] __attribute__((section(".noinit")));

extern "C" __attribute((naked)) void WDT_IRQHandler(void) {
	register uint32_t* msp = (uint32_t*)__get_MSP();
	register uint32_t* psp = (uint32_t*)__get_PSP();

	asm volatile("push {lr}");

	XPCC_LOG_ERROR .flush();

	XPCC_LOG_ERROR << "WDT Timeout\nMSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", msp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", msp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", msp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", msp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", msp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", msp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", msp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", msp[psr]);

	XPCC_LOG_ERROR << "PSP\n";
	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", psp[r0]);
	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", psp[r1]);
	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", psp[r2]);
	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", psp[r3]);
	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", psp[r12]);
	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", psp[lr]);
	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", psp[pc]);
	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", psp[psr]);

	XPCC_LOG_ERROR .flush();

	LPC_WDT->WDMOD = 0x3;
	LPC_WDT->WDFEED = 0xFF;

	while(1) {
		ledRed::set();
		ledGreen::set();
	}
}



//extern "C"
//void Hard_Fault_Handler(uint32_t stack[]) {
//
//	//register uint32_t* stack = (uint32_t*)__get_MSP();
//
//	crashData[0] = 0xFAFA5555;
//	crashData[1] = stack[pc];
//	crashData[2] = stack[lr];
//
//	XPCC_LOG_ERROR .flush();
//
//	XPCC_LOG_ERROR .printf("Hard Fault\n");
//
//	XPCC_LOG_ERROR .printf("r0  = 0x%08x\n", stack[r0]);
//	XPCC_LOG_ERROR .printf("r1  = 0x%08x\n", stack[r1]);
//	XPCC_LOG_ERROR .printf("r2  = 0x%08x\n", stack[r2]);
//	XPCC_LOG_ERROR .printf("r3  = 0x%08x\n", stack[r3]);
//	XPCC_LOG_ERROR .printf("r12 = 0x%08x\n", stack[r12]);
//	XPCC_LOG_ERROR .printf("lr  = 0x%08x\n", stack[lr]);
//	XPCC_LOG_ERROR .printf("pc  = 0x%08x\n", stack[pc]);
//	XPCC_LOG_ERROR .printf("psr = 0x%08x\n", stack[psr]);
//
//	XPCC_LOG_ERROR .flush();
//
//	LPC_WDT->WDMOD = 0x3;
//	LPC_WDT->WDFEED = 0xFF;
//
//	while(1) {
//		ledRed::set();
//		ledGreen::set();
//	}
//
//	//for(int i = 0; i < 10000; i++) {}
//	//NVIC_SystemReset();
//
//}

void a(char* a) {
	XPCC_LOG_ERROR .printf("a(%s)\n", a);
	xpcc::sleep(300);
}

void b(char *b) {
	XPCC_LOG_ERROR .printf("b(%s)\n", b);
	a("recursive");
	xpcc::sleep(200);
}
