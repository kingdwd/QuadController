/*
 * Ultrasonic.hpp
 *
 *  Created on: Apr 18, 2014
 *      Author: walmis
 */

#include <AP_RangeFinder.h>
#include <AP_RangeFinder_HC04.h>
#include <stdio.h>
#include <xpcc/architecture.hpp>
#include <DerivativeFilter.h>
#include <xpcc/math/filter.hpp>
//namespace xpcc {
//
//template<typename Trig, typename Echo, typename Clock = xpcc::Clock>
//class Ultrasonic : TickerTask {
//public:
//	Ultrasonic() : tMeasure(0) {
//		Trig::setOutput(0);
//		Echo::setInput();
//
//		timestamp = 0;
//		time = 0;
//
//
//	}
//
//	void ping() {
//		Trig::set();
//		tMeasure.restart(1);
//	}
//
//	uint32_t getTime() {
//		filter.append(time.getTime());
//		filter.update();
//		return filter.getValue();
//	}
//
//	void handleTick() override {
//		if(tMeasure.isActive() && tMeasure.isExpired()) {
//			Trig::reset();
//			tMeasure.stop();
//		}
//	}
//
//private:
//	Timestamp timestamp;
//	Timeout<> tMeasure;
//
//	xpcc::filter::Median<uint32_t, 3> filter;
//	Timestamp time;
//
//};
//}

extern const AP_HAL::HAL& hal;

class HC04;
extern HC04 rangefinder;

GPIO__OUTPUT(snd_trig, 1, 29);
GPIO__INPUT(snd_echo, 1, 26); //CAP0[0]

using namespace xpcc::lpc17;

class HC04 : xpcc::TickerTask {
public:
	HC04() : tPinger(50) {
		AP_RangeFinder_HC04::setProxy(&HC04::update);


	}
private:

	void handleInit() {
		//Timer0 should be running by now,
		//it is initialized by AP_HAL::Scheduler
		snd_trig::setOutput(0);
		Pinsel::setFunc<snd_echo>(3);
		Timer0::configureCapture(0,
				(Timer0::CaptureFlags) (Timer0::CaptureFlags::CAP_RISING
						| Timer0::CaptureFlags::CAP_FALLING
						| Timer0::CaptureFlags::INT_ON_CAP));
		NVIC_EnableIRQ(TIMER0_IRQn);

	}

	xpcc::PeriodicTimer<> tPinger;

	volatile uint32_t micros_start;
	volatile uint32_t micros_end;

	xpcc::filter::Median<int32_t, 5> median;

	xpcc::Timestamp t_last_valid_sample;
	int32_t sample;

	void handleTick() override {
		if(snd_trig::read()) {
			snd_trig::reset();
		}

		//(t / (float)SystemCoreClock) * 334.0/2.0;
		if(tPinger.isExpired()) {
			micros_end = Timer0::getCaptureValue(0);

			int32_t tdiff = micros_end - micros_start;
			if(tdiff > 0 && tdiff < 50000) {
				median.append(tdiff);
				median.update();
				sample = median.getValue();
				t_last_valid_sample = xpcc::Clock::now();
				//printf("t %d %.2f\n", sample, getHeight() );
			}

			snd_trig::set();
		}

	}

	float getHeight() {
		return sample*1.0e-6f * 334.0/2;
	}

	void handleInterrupt(int irqn) {
		if(irqn == TIMER0_IRQn) {
			if(Timer0::getIntStatus(Timer0::IntType::TIM_CR0_INT)) {
				if(snd_echo::read()) {
					micros_start = Timer0::getCounterValue();
				} else {
					micros_end = Timer0::getCounterValue();
				}
				Timer0::clearIntPending(Timer0::IntType::TIM_CR0_INT);
			}
		}
	}

	static void update(RangeFinder::RangeFinder_State &s)
	{
		//printf("%u\n", s.healthy);
		if(xpcc::Clock::now() - rangefinder.t_last_valid_sample > 150) {
			//printf("!\n");
			s.healthy = false;
		} else {
			s.distance_cm =  lroundf(rangefinder.getHeight()*100.0f);
			s.healthy = true;
		}
	}
};

HC04 rangefinder;

