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
 * Array of msg_entry needed for @p mavlink_parse_char() (trought @p mavlink_get_msg_entry())
 */
constexpr std::array<mavlink_msg_entry_t, 3> MESSAGE_ENTRIES {{ {0, 50, 9, 9, 0, 0, 0}, {180, 21, 21, 21, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0} }};

//! MAVLINK VERSION
constexpr auto MAVLINK_VERSION = 3;


// ENUM DEFINITIONS


/** @brief  */
enum class OFFBOARD_CONTROL_MODE : uint8_t
{
    MODE_PASS_THROUGH=0, /* Pass moment values in [Nm] directly to the mixer | */
    MODE_THRUST_VECTOR_YAW=1, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw setpoint in [rad] | */
    MODE_THRUST_VECTOR_YAWRATE=2, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw-rate setpoint in [rad/s] | */
    MODE_ROLL_PITCH_YAWRATE_THRUST=3, /* Command roll angle, pitch angle, yaw rate, and thrust norm in SI units | */
    MODE_ROLLRATE_PITCHTATE_YAWRATE_THRUST=4, /* Command roll angle rate, pitch angle rate, yaw rate, and thrust norm in SI units | */
    MODE_POSITION_TARGET_YAW=5, /* Command position target in meters and yaw in SI units | */
};

//! OFFBOARD_CONTROL_MODE ENUM_END
constexpr auto OFFBOARD_CONTROL_MODE_ENUM_END = 6;


} // namespace qrotor
} // namespace mavlink

// MESSAGE DEFINITIONS
#include "./mavlink_msg_offboard_control.hpp"

// base include
#include "../minimal/minimal.hpp"
