#include <xpcc/architecture.hpp>
#include "RCOutput.h"
#include <stdio.h>

using namespace xpcc::lpc17;
using namespace XpccHAL;

const uint8_t chMap[] = {3, 4, 5, 6};

#define TIMER_PRESCALE 10

void RCOutput::init(void* machtnichts) {
	Pinsel::setFunc(2,2,1);
	Pinsel::setFunc(2,3,1);
	Pinsel::setFunc(2,4,1);
	Pinsel::setFunc(2,5,1);

	PWM::initTimer(TIMER_PRESCALE);

	set_freq(0xFF, 100);

	PWM::configureMatch(0, PWM::MatchFlags::RESET_ON_MATCH);

	PWM::enable();

	while(LPC_PWM1->LER);
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
	prescale = SystemCoreClock / TIMER_PRESCALE / freq_hz;

	PWM::matchUpdate(0, prescale);
}

uint16_t RCOutput::get_freq(uint8_t ch) {
    return (SystemCoreClock/TIMER_PRESCALE) / prescale;
}

void RCOutput::enable_ch(uint8_t ch)
{
	if(ch > 3) return;
	PWM::channelEnable(chMap[ch]);
}

void RCOutput::disable_ch(uint8_t ch)
{
	PWM::channelEnable(chMap[ch], false);
}

void RCOutput::write(uint8_t ch, uint16_t period_us)
{
	PWM::matchUpdate(chMap[ch], (prescale*1000) / (10000000/period_us), PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
}

void RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
	auto m = PWM::multiMatchUpdate();
	for(int i = 0; i < len; i++) {
		m.set(chMap[ch], (prescale*100) / (1000000/period_us[i]));
	}
	m.commit(PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
}

uint16_t RCOutput::read(uint8_t ch) {
    return PWM::readMatchRegister(chMap[ch])*(TIMER_PRESCALE*1000) / prescale;
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
	for(int i = 0; i < len; i++) {
		period_us[i] = read(i);
	}
}

