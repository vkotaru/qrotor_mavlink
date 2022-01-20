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

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{180, 21, 21, 21, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_QROTOR

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_OFFBOARD_CONTROL_MODE
#define HAVE_ENUM_OFFBOARD_CONTROL_MODE
typedef enum OFFBOARD_CONTROL_MODE
{
   MODE_PASS_THROUGH=0, /* Pass moment values in [Nm] directly to the mixer | */
   MODE_THRUST_VECTOR_YAW=1, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw setpoint in [rad] | */
   MODE_THRUST_VECTOR_YAWRATE=2, /* Command thrust vector (Fx, Fy, Fz) in newtons, and yaw-rate setpoint in [rad/s] | */
   MODE_ROLL_PITCH_YAWRATE_THRUST=3, /* Command roll angle, pitch angle, yaw rate, and thrust norm in SI units | */
   MODE_ROLLRATE_PITCHTATE_YAWRATE_THRUST=4, /* Command roll angle rate, pitch angle rate, yaw rate, and thrust norm in SI units | */
   MODE_POSITION_TARGET_YAW=5, /* Command position target in meters and yaw in SI units | */
   OFFBOARD_CONTROL_MODE_ENUM_END=6, /*  | */
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

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 0

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL}
# define MAVLINK_MESSAGE_NAMES {{ "OFFBOARD_CONTROL", 180 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_QROTOR_H
