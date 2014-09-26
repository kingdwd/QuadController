/*
 * mavlink.hpp
 *
 *  Created on: Sep 22, 2014
 *      Author: walmis
 */

#ifndef MAVLINK_HPP_
#define MAVLINK_HPP_

#include <xpcc/architecture.hpp>
#include <mavlink/common/mavlink.h>
#include "radio.hpp"

class Mavlink : xpcc::TickerTask {
public:

	void handleInit() {
		mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
		mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
		mavlink_system.type = MAV_TYPE_QUADROTOR;   ///< This system is an airplane / fixed wing

		//mavlink_msg_param_value_pack()
	}

	void handleTick() {
		static Timeout<> tHearbeat(1000);

		if(tHearbeat.isExpired() && !radio.isSendingPacket()) {
			mavlink_msg_heartbeat_pack(mavlink_system.sysid,
					mavlink_system.compid, &msg,
					mavlink_system.type, autopilot_type,
					system_mode, custom_mode, system_state);


			uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
			radio.sendPacket(buf, len);

			tHearbeat.restart(1000);
		}

	}

	mavlink_message_t msg;
	uint8_t buf[MAVLINK_MAX_PACKET_LEN];
	mavlink_system_t mavlink_system;

	uint8_t system_type = MAV_TYPE_QUADROTOR;
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

	uint8_t system_mode = MAV_MODE_STABILIZE_DISARMED;
	uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
	uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
};



#endif /* MAVLINK_HPP_ */
