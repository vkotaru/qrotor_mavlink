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
    packet_in.mode = 65;
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.thrust = 101.0;
    packet_in.yaw = 129.0;

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet1{};
    mavlink::qrotor::msg::OFFBOARD_CONTROL packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.mode, packet2.mode);
    EXPECT_EQ(packet1.x, packet2.x);
    EXPECT_EQ(packet1.y, packet2.y);
    EXPECT_EQ(packet1.z, packet2.z);
    EXPECT_EQ(packet1.thrust, packet2.thrust);
    EXPECT_EQ(packet1.yaw, packet2.yaw);
}

#ifdef TEST_INTEROP
TEST(qrotor_interop, OFFBOARD_CONTROL)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_offboard_control_t packet_c {
         17.0, 45.0, 73.0, 101.0, 129.0, 65
    };

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet_in{};
    packet_in.mode = 65;
    packet_in.x = 17.0;
    packet_in.y = 45.0;
    packet_in.z = 73.0;
    packet_in.thrust = 101.0;
    packet_in.yaw = 129.0;

    mavlink::qrotor::msg::OFFBOARD_CONTROL packet2{};

    mavlink_msg_offboard_control_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.mode, packet2.mode);
    EXPECT_EQ(packet_in.x, packet2.x);
    EXPECT_EQ(packet_in.y, packet2.y);
    EXPECT_EQ(packet_in.z, packet2.z);
    EXPECT_EQ(packet_in.thrust, packet2.thrust);
    EXPECT_EQ(packet_in.yaw, packet2.yaw);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
