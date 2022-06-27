/** @file
 *	@brief MAVLink comm testsuite protocol generated from qrotor.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "qrotor.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(qrotor, OFFBOARD_CONTROL)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet_in{};
    packet_in.mode = 77;
    packet_in.w = 17.0;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.thrust = 129.0;
    packet_in.yaw = 157.0;
    packet_in.act = 144;

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet1{};
    mavlink::qrotor::msg::OFFBOARD_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.w, packet2.w);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
    EXPECT_EQ(packet1.act, packet2.act);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, OFFBOARD_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_offboard_control_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 157.0, 77, 144
    };

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet_in{};
    packet_in.mode = 77;
    packet_in.w = 17.0;
    packet_in.x = 45.0;
    packet_in.y = 73.0;
    packet_in.z = 101.0;
    packet_in.thrust = 129.0;
    packet_in.yaw = 157.0;
    packet_in.act = 144;

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet2{};

    mavlink_msg_offboard_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.w, packet2.w);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);
    EXPECT_EQ(packet_in.act, packet2.act);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(qrotor, POWER_READINGS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::POWER_READINGS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.voltage = 185.0;
    packet_in.v_adc = 93372036854776311ULL;
    packet_in.current = 213.0;
    packet_in.c_adc = 93372036854776815ULL;

    mavlink::qrotor::msg::POWER_READINGS packet1{};
    mavlink::qrotor::msg::POWER_READINGS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.voltage, packet2.voltage);
    EXPECT_EQ(packet1.v_adc, packet2.v_adc);
    EXPECT_EQ(packet1.current, packet2.current);
    EXPECT_EQ(packet1.c_adc, packet2.c_adc);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, POWER_READINGS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_power_readings_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 93372036854776815ULL, 185.0, 213.0
    };

    mavlink::qrotor::msg::POWER_READINGS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.voltage = 185.0;
    packet_in.v_adc = 93372036854776311ULL;
    packet_in.current = 213.0;
    packet_in.c_adc = 93372036854776815ULL;

    mavlink::qrotor::msg::POWER_READINGS packet2{};

    mavlink_msg_power_readings_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.voltage, packet2.voltage);
    EXPECT_EQ(packet_in.v_adc, packet2.v_adc);
    EXPECT_EQ(packet_in.current, packet2.current);
    EXPECT_EQ(packet_in.c_adc, packet2.c_adc);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(qrotor, ONBOARD_IMU)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::ONBOARD_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;

    mavlink::qrotor::msg::ONBOARD_IMU packet1{};
    mavlink::qrotor::msg::ONBOARD_IMU packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.xacc, packet2.xacc);
    EXPECT_EQ(packet1.yacc, packet2.yacc);
    EXPECT_EQ(packet1.zacc, packet2.zacc);
    EXPECT_EQ(packet1.xgyro, packet2.xgyro);
    EXPECT_EQ(packet1.ygyro, packet2.ygyro);
    EXPECT_EQ(packet1.zgyro, packet2.zgyro);
    EXPECT_EQ(packet1.xmag, packet2.xmag);
    EXPECT_EQ(packet1.ymag, packet2.ymag);
    EXPECT_EQ(packet1.zmag, packet2.zmag);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, ONBOARD_IMU)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_onboard_imu_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0, 185.0, 213.0, 241.0, 269.0, 297.0
    };

    mavlink::qrotor::msg::ONBOARD_IMU packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.xacc = 73.0;
    packet_in.yacc = 101.0;
    packet_in.zacc = 129.0;
    packet_in.xgyro = 157.0;
    packet_in.ygyro = 185.0;
    packet_in.zgyro = 213.0;
    packet_in.xmag = 241.0;
    packet_in.ymag = 269.0;
    packet_in.zmag = 297.0;

    mavlink::qrotor::msg::ONBOARD_IMU packet2{};

    mavlink_msg_onboard_imu_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.xacc, packet2.xacc);
    EXPECT_EQ(packet_in.yacc, packet2.yacc);
    EXPECT_EQ(packet_in.zacc, packet2.zacc);
    EXPECT_EQ(packet_in.xgyro, packet2.xgyro);
    EXPECT_EQ(packet_in.ygyro, packet2.ygyro);
    EXPECT_EQ(packet_in.zgyro, packet2.zgyro);
    EXPECT_EQ(packet_in.xmag, packet2.xmag);
    EXPECT_EQ(packet_in.ymag, packet2.ymag);
    EXPECT_EQ(packet_in.zmag, packet2.zmag);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(qrotor, INPUT_SI)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::INPUT_SI packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.thrust = 157.0;

    mavlink::qrotor::msg::INPUT_SI packet1{};
    mavlink::qrotor::msg::INPUT_SI packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, INPUT_SI)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_input_si_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0
    };

    mavlink::qrotor::msg::INPUT_SI packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.x = 73.0;
    packet_in.y = 101.0;
    packet_in.z = 129.0;
    packet_in.thrust = 157.0;

    mavlink::qrotor::msg::INPUT_SI packet2{};

    mavlink_msg_input_si_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(qrotor, INPUT_PWM)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::INPUT_PWM packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.w = 73.0;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;

    mavlink::qrotor::msg::INPUT_PWM packet1{};
    mavlink::qrotor::msg::INPUT_PWM packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.w, packet2.w);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, INPUT_PWM)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_input_pwm_t packet_c {
         93372036854775807ULL, 73.0, 101.0, 129.0, 157.0
    };

    mavlink::qrotor::msg::INPUT_PWM packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.w = 73.0;
    packet_in.x = 101.0;
    packet_in.y = 129.0;
    packet_in.z = 157.0;

    mavlink::qrotor::msg::INPUT_PWM packet2{};

    mavlink_msg_input_pwm_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.w, packet2.w);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif

TEST(qrotor, SYSTEM_STATUS)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::qrotor::msg::SYSTEM_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.w = 73.0;

    mavlink::qrotor::msg::SYSTEM_STATUS packet1{};
    mavlink::qrotor::msg::SYSTEM_STATUS packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.time_usec, packet2.time_usec);
    EXPECT_EQ(packet1.w, packet2.w);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, SYSTEM_STATUS)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_system_status_t packet_c {
         93372036854775807ULL, 73.0
    };

    mavlink::qrotor::msg::SYSTEM_STATUS packet_in{};
    packet_in.time_usec = 93372036854775807ULL;
    packet_in.w = 73.0;

    mavlink::qrotor::msg::SYSTEM_STATUS packet2{};

    mavlink_msg_system_status_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.time_usec, packet2.time_usec);
    EXPECT_EQ(packet_in.w, packet2.w);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
