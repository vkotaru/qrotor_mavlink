/** @file
 *    @brief MAVLink comm protocol testsuite generated from all.xml
 *    @see https://mavlink.io/en/
 */
#pragma once
#ifndef ALL_TESTSUITE_H
#define ALL_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_ardupilotmega(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ASLUAV(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_development(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_icarous(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_minimal(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_python_array_test(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_standard(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_test(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_ualberta(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_uAvionix(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_storm32(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_AVSSUAS(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_all(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
    mavlink_test_ardupilotmega(system_id, component_id, last_msg);
    mavlink_test_ASLUAV(system_id, component_id, last_msg);
    mavlink_test_common(system_id, component_id, last_msg);
    mavlink_test_development(system_id, component_id, last_msg);
    mavlink_test_icarous(system_id, component_id, last_msg);
    mavlink_test_minimal(system_id, component_id, last_msg);
    mavlink_test_python_array_test(system_id, component_id, last_msg);
    mavlink_test_standard(system_id, component_id, last_msg);
    mavlink_test_test(system_id, component_id, last_msg);
    mavlink_test_ualberta(system_id, component_id, last_msg);
    mavlink_test_uAvionix(system_id, component_id, last_msg);
    mavlink_test_storm32(system_id, component_id, last_msg);
    mavlink_test_AVSSUAS(system_id, component_id, last_msg);
    mavlink_test_all(system_id, component_id, last_msg);
}
#endif

#include "mavlink/v2.0/ardupilotmega/testsuite.h"
#include "mavlink/v2.0/ASLUAV/testsuite.h"
#include "mavlink/v2.0/common/testsuite.h"
#include "mavlink/v2.0/development/testsuite.h"
#include "mavlink/v2.0/icarous/testsuite.h"
#include "mavlink/v2.0/minimal/testsuite.h"
#include "mavlink/v2.0/python_array_test/testsuite.h"
#include "mavlink/v2.0/standard/testsuite.h"
#include "mavlink/v2.0/test/testsuite.h"
#include "mavlink/v2.0/ualberta/testsuite.h"
#include "mavlink/v2.0/uAvionix/testsuite.h"
#include "mavlink/v2.0/storm32/testsuite.h"
#include "mavlink/v2.0/AVSSUAS/testsuite.h"



static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // ALL_TESTSUITE_H
