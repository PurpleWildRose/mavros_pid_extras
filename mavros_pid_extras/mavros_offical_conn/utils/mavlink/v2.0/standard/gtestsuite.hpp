/** @file
 *	@brief MAVLink comm testsuite protocol generated from standard.xml
 *	@see http://mavlink.org
 */

#pragma once

#include <gtest/gtest.h>
#include "standard.hpp"

#ifdef TEST_INTEROP
using namespace mavlink;
#undef MAVLINK_HELPER
#include "mavlink.h"
#endif


TEST(standard, AUTOPILOT_VERSION)
{
    mavlink::mavlink_message_t msg;
    mavlink::MsgMap map1(msg);
    mavlink::MsgMap map2(msg);

    mavlink::standard::msg::AUTOPILOT_VERSION packet_in{};
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.flight_sw_version = 963498296;
    packet_in.middleware_sw_version = 963498504;
    packet_in.os_sw_version = 963498712;
    packet_in.board_version = 963498920;
    packet_in.flight_custom_version = {{ 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.middleware_custom_version = {{ 137, 138, 139, 140, 141, 142, 143, 144 }};
    packet_in.os_custom_version = {{ 161, 162, 163, 164, 165, 166, 167, 168 }};
    packet_in.vendor_id = 18899;
    packet_in.product_id = 19003;
    packet_in.uid = 93372036854776311ULL;
    packet_in.uid2 = {{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }};

    mavlink::standard::msg::AUTOPILOT_VERSION packet1{};
    mavlink::standard::msg::AUTOPILOT_VERSION packet2{};

    packet1 = packet_in;

    //std::cout << packet1.to_yaml() << std::endl;

    packet1.serialize(map1);

    mavlink::mavlink_finalize_message(&msg, 1, 1, packet1.MIN_LENGTH, packet1.LENGTH, packet1.CRC_EXTRA);

    packet2.deserialize(map2);

    EXPECT_EQ(packet1.capabilities, packet2.capabilities);
    EXPECT_EQ(packet1.flight_sw_version, packet2.flight_sw_version);
    EXPECT_EQ(packet1.middleware_sw_version, packet2.middleware_sw_version);
    EXPECT_EQ(packet1.os_sw_version, packet2.os_sw_version);
    EXPECT_EQ(packet1.board_version, packet2.board_version);
    EXPECT_EQ(packet1.flight_custom_version, packet2.flight_custom_version);
    EXPECT_EQ(packet1.middleware_custom_version, packet2.middleware_custom_version);
    EXPECT_EQ(packet1.os_custom_version, packet2.os_custom_version);
    EXPECT_EQ(packet1.vendor_id, packet2.vendor_id);
    EXPECT_EQ(packet1.product_id, packet2.product_id);
    EXPECT_EQ(packet1.uid, packet2.uid);
    EXPECT_EQ(packet1.uid2, packet2.uid2);
}

#ifdef TEST_INTEROP
TEST(standard_interop, AUTOPILOT_VERSION)
{
    mavlink_message_t msg;

    // to get nice print
    memset(&msg, 0, sizeof(msg));

    mavlink_autopilot_version_t packet_c {
         93372036854775807ULL, 93372036854776311ULL, 963498296, 963498504, 963498712, 963498920, 18899, 19003, { 113, 114, 115, 116, 117, 118, 119, 120 }, { 137, 138, 139, 140, 141, 142, 143, 144 }, { 161, 162, 163, 164, 165, 166, 167, 168 }, { 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }
    };

    mavlink::standard::msg::AUTOPILOT_VERSION packet_in{};
    packet_in.capabilities = 93372036854775807ULL;
    packet_in.flight_sw_version = 963498296;
    packet_in.middleware_sw_version = 963498504;
    packet_in.os_sw_version = 963498712;
    packet_in.board_version = 963498920;
    packet_in.flight_custom_version = {{ 113, 114, 115, 116, 117, 118, 119, 120 }};
    packet_in.middleware_custom_version = {{ 137, 138, 139, 140, 141, 142, 143, 144 }};
    packet_in.os_custom_version = {{ 161, 162, 163, 164, 165, 166, 167, 168 }};
    packet_in.vendor_id = 18899;
    packet_in.product_id = 19003;
    packet_in.uid = 93372036854776311ULL;
    packet_in.uid2 = {{ 185, 186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 199, 200, 201, 202 }};

    mavlink::standard::msg::AUTOPILOT_VERSION packet2{};

    mavlink_msg_autopilot_version_encode(1, 1, &msg, &packet_c);

    // simulate message-handling callback
    [&packet2](const mavlink_message_t *cmsg) {
        MsgMap map2(cmsg);

        packet2.deserialize(map2);
    } (&msg);

    EXPECT_EQ(packet_in.capabilities, packet2.capabilities);
    EXPECT_EQ(packet_in.flight_sw_version, packet2.flight_sw_version);
    EXPECT_EQ(packet_in.middleware_sw_version, packet2.middleware_sw_version);
    EXPECT_EQ(packet_in.os_sw_version, packet2.os_sw_version);
    EXPECT_EQ(packet_in.board_version, packet2.board_version);
    EXPECT_EQ(packet_in.flight_custom_version, packet2.flight_custom_version);
    EXPECT_EQ(packet_in.middleware_custom_version, packet2.middleware_custom_version);
    EXPECT_EQ(packet_in.os_custom_version, packet2.os_custom_version);
    EXPECT_EQ(packet_in.vendor_id, packet2.vendor_id);
    EXPECT_EQ(packet_in.product_id, packet2.product_id);
    EXPECT_EQ(packet_in.uid, packet2.uid);
    EXPECT_EQ(packet_in.uid2, packet2.uid2);

#ifdef PRINT_MSG
    PRINT_MSG(msg);
#endif
}
#endif
