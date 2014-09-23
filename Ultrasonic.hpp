/*
 * Ultrasonic.hpp
 *
 *  Created on: Apr 18, 2014
 *      Author: walmis
 */

#ifndef ULTRASONIC_HPP_
#define ULTRASONIC_HPP_

#include <xpcc/architecture.hpp>

namespace xpcc {

template<typename Trig, typename Echo, typename Clock = xpcc::Clock>
class Ultrasonic : TickerTask {
public:
	Ultrasonic() : tMeasure(0) {
		Trig::setOutput(0);
		Echo::setInput();

		timestamp = 0;
		time = 0;


	}

	void ping() {
		Trig::set();
		tMeasure.restart(1);
	}

	uint32_t getTime() {
		filter.append(time.getTime());
		filter.update();
		return filter.getValue();
	}

	void handleTick() override {
		if(tMeasure.isActive() && tMeasure.isExpired()) {
			Trig::reset();
			tMeasure.stop();
		}
	}

private:
	Timestamp timestamp;
	Timeout<> tMeasure;

	xpcc::filter::Median<uint32_t, 3> filter;
	Timestamp time;

};
}

#endif /* ULTRASONIC_HPP_ */
