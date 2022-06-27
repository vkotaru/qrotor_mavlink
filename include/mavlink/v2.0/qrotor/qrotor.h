/** @file
 *  @brief MAVLink comm protocol generated from qrotor.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_QROTOR_H
#define MAVLINK_QROTOR_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_QROTOR.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH -34105699551980724

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 50, 9, 9, 0, 0, 0}, {180, 83, 26, 26, 0, 0, 0}, {181, 60, 32, 32, 0, 0, 0}, {182, 153, 44, 44, 0, 0, 0}, {183, 208, 24, 24, 0, 0, 0}, {184, 149, 24, 24, 0, 0, 0}, {185, 17, 12, 12, 0, 0, 0}, {300, 217, 22, 22, 0, 0, 0}}
#endif

#include "mavlink/v2.0/protocol.h"

#define MAVLINK_ENABLED_QROTOR

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_OFFBOARD_CONTROL_MODE
#define HAVE_ENUM_OFFBOARD_CONTROL_MODE
typedef enum OFFBOARD_CONTROL_MODE
{
   MODE_PASS_THROUGH_SI=0, /* Pass moment values in [Nm] directly to the mixer | */
   MODE_THRUST_VECTOR_YAW=1, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw setpoint in [rad] | */
   MODE_THRUST_VECTOR_YAWRATE=2, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw-rate setpoint in [rad/s] | */
   MODE_ROLL_PITCH_YAWRATE_THRUST=3, /* Command roll angle, pitch angle, yaw rate, and thrust norm in SI units | */
   MODE_ROLLRATE_PITCHTATE_YAWRATE_THRUST=4, /* Command roll angle rate, pitch angle rate, yaw rate, and thrust norm in SI units | */
   MODE_POSITION_TARGET_YAW=5, /* Command position target in meters and yaw in SI units | */
   MODE_PASS_THROUGH_PWM=6, /* Pass PWM VALUES directly | */
   OFFBOARD_CONTROL_MODE_ENUM_END=7, /*  | */
} OFFBOARD_CONTROL_MODE;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_offboard_control.h"
#include "./mavlink_msg_power_readings.h"
#include "./mavlink_msg_onboard_imu.h"
#include "./mavlink_msg_input_si.h"
#include "./mavlink_msg_input_pwm.h"
#include "./mavlink_msg_system_status.h"

// base include
#include "mavlink/v2.0/minimal/minimal.h"

#undef MAVLINK_THIS_XML_HASH
#define MAVLINK_THIS_XML_HASH -34105699551980724

#if MAVLINK_THIS_XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL, MAVLINK_MESSAGE_INFO_POWER_READINGS, MAVLINK_MESSAGE_INFO_ONBOARD_IMU, MAVLINK_MESSAGE_INFO_INPUT_SI, MAVLINK_MESSAGE_INFO_INPUT_PWM, MAVLINK_MESSAGE_INFO_SYSTEM_STATUS, MAVLINK_MESSAGE_INFO_PROTOCOL_VERSION}
# define MAVLINK_MESSAGE_NAMES {{ "HEARTBEAT", 0 }, { "INPUT_PWM", 184 }, { "INPUT_SI", 183 }, { "OFFBOARD_CONTROL", 180 }, { "ONBOARD_IMU", 182 }, { "POWER_READINGS", 181 }, { "PROTOCOL_VERSION", 300 }, { "SYSTEM_STATUS", 185 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_QROTOR_H
