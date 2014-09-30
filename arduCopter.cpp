/*
 * arduCopter.cpp
 *
 *  Created on: Sep 30, 2014
 *      Author: walmis
 */

#include <AP_Param.h>

#include "AP_HAL_XPCC/AP_HAL_XPCC.h"


#include "ArduCopter/ArduCopter.pde"
#include "ArduCopter/AP_State.pde"
#include "ArduCopter/Attitude.pde"
#include "ArduCopter/GCS_Mavlink.pde"
#include "ArduCopter/Log.pde"
#include "ArduCopter/Parameters.pde"
#include "ArduCopter/UserCode.pde"
#include "ArduCopter/commands.pde"
#include "ArduCopter/commands_logic.pde"
#include "ArduCopter/compassmot.pde"
#include "ArduCopter/compat.pde"
#include "ArduCopter/control_acro.pde"
#include "ArduCopter/control_althold.pde"
#include "ArduCopter/control_auto.pde"
#include "ArduCopter/control_autotune.pde"
#include "ArduCopter/control_circle.pde"
#include "ArduCopter/control_drift.pde"
#include "ArduCopter/control_flip.pde"
#include "ArduCopter/control_guided.pde"
#include "ArduCopter/control_land.pde"
#include "ArduCopter/control_loiter.pde"
#include "ArduCopter/control_ofloiter.pde"
#include "ArduCopter/control_poshold.pde"
#include "ArduCopter/control_rtl.pde"
#include "ArduCopter/control_sport.pde"
#include "ArduCopter/control_stabilize.pde"
#include "ArduCopter/crash_check.pde"
#include "ArduCopter/ekf_check.pde"
#include "ArduCopter/esc_calibration.pde"
#include "ArduCopter/events.pde"
#include "ArduCopter/failsafe.pde"
#include "ArduCopter/fence.pde"
#include "ArduCopter/flight_mode.pde"
#include "ArduCopter/heli.pde"
#include "ArduCopter/heli_control_acro.pde"
#include "ArduCopter/heli_control_stabilize.pde"
#include "ArduCopter/inertia.pde"
#include "ArduCopter/leds.pde"
#include "ArduCopter/motor_test.pde"
#include "ArduCopter/motors.pde"
#include "ArduCopter/navigation.pde"
#include "ArduCopter/perf_info.pde"
#include "ArduCopter/position_vector.pde"
#include "ArduCopter/radio.pde"
#include "ArduCopter/sensors.pde"
#include "ArduCopter/setup.pde"
#include "ArduCopter/switches.pde"
#include "ArduCopter/system.pde"
#include "ArduCopter/test.pde"

