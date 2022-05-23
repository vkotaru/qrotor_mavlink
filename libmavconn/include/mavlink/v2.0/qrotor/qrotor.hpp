/** @file
 *	@brief MAVLink comm protocol generated from qrotor.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <array>
#include <cstdint>
#include <sstream>

#ifndef MAVLINK_STX
#define MAVLINK_STX 253
#endif

#include "../message.hpp"

namespace mavlink {
namespace qrotor {

/**
 * Array of msg_entry needed for @p mavlink_parse_char() (through @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 8> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {180, 99, 25, 25, 0, 0, 0}, {181, 60, 32, 32, 0, 0, 0}, {182, 153, 44, 44, 0, 0, 0}, {183, 208, 24, 24, 0, 0, 0}, {184, 149, 24, 24, 0, 0, 0}, {185, 17, 12, 12, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief  */
enum class OFFBOARD_CONTROL_MODE : uint8_t
{
    MODE_PASS_THROUGH_SI=0, /* Pass moment values in [Nm] directly to the mixer | */
    MODE_THRUST_VECTOR_YAW=1, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw setpoint in [rad] | */
    MODE_THRUST_VECTOR_YAWRATE=2, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw-rate setpoint in [rad/s] | */
    MODE_ROLL_PITCH_YAWRATE_THRUST=3, /* Command roll angle, pitch angle, yaw rate, and thrust norm in SI units | */
    MODE_ROLLRATE_PITCHTATE_YAWRATE_THRUST=4, /* Command roll angle rate, pitch angle rate, yaw rate, and thrust norm in SI units | */
    MODE_POSITION_TARGET_YAW=5, /* Command position target in meters and yaw in SI units | */
    MODE_PASS_THROUGH_PWM=6, /* Pass PWM VALUES directly | */
};

//! OFFBOARD_CONTROL_MODE ENUM_END
constexpr auto OFFBOARD_CONTROL_MODE_ENUM_END = 7;


} // namespace qrotor
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_offboard_control.hpp"
#include "./mavlink_msg_power_readings.hpp"
#include "./mavlink_msg_onboard_imu.hpp"
#include "./mavlink_msg_input_si.hpp"
#include "./mavlink_msg_input_pwm.hpp"
#include "./mavlink_msg_system_status.hpp"

// base include
#include "../minimal/minimal.hpp"
