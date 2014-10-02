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
	printf("%s(%d)\n", __PRETTY_FUNCTION__, freq_hz);
	prescale = SystemCoreClock / TIMER_PRESCALE / freq_hz;

	PWM::matchUpdate(0, prescale);

	//write(0, _channels, 8);
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
	_channels[ch] = period_us;
	PWM::matchUpdate(chMap[ch], SystemCoreClock / 10 / (1000000 / period_us), PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
}

void RCOutput::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
	for(int i = ch; i < len; i++) {
		_channels[i] = period_us[i];
	}

	auto m = PWM::multiMatchUpdate();
	for(int i = ch; i < std::min(len, (uint8_t)4); i++) {
		m.set(chMap[i], SystemCoreClock / 10 / (1000000 / period_us[i]));
	}
	m.commit(PWM::UpdateType::PWM_MATCH_UPDATE_NEXT_RST);
}

uint16_t RCOutput::read(uint8_t ch) {
    return _channels[ch];
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
	for(int i = 0; i < len; i++) {
		period_us[i] = _channels[i];
	}
}

