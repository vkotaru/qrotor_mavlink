// MESSAGE OFFBOARD_CONTROL support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief OFFBOARD_CONTROL message
 *
 * Custom offboard control; Send command mode type and values
 */
struct OFFBOARD_CONTROL : mavlink::Message {
    static constexpr msgid_t MSG_ID = 180;
    static constexpr size_t LENGTH = 21;
    static constexpr size_t MIN_LENGTH = 21;
    static constexpr uint8_t CRC_EXTRA = 21;
    static constexpr auto NAME = "OFFBOARD_CONTROL";


    uint8_t mode; /*<  Offboard control mode, see OFFBOARD_CONTROL_MODE */
    float x; /*<  X-channel, (typically, thrust in X-direction [N]) */
    float y; /*<  Y-channel, (typically, thrust in Y-direction [N]) */
    float z; /*<  Z-channel, (typically, thrust in Z-direction [N]) */
    float thrust; /*<  Thrust, (typically, thrust-scale [N]) */
    float yaw; /*<  Yaw setpoint [rad] */


    inline std::string get_name(void) const override
    {
            return NAME;
    }

    inline Info get_message_info(void) const override
    {
            return { MSG_ID, LENGTH, MIN_LENGTH, CRC_EXTRA };
    }

    inline std::string to_yaml(void) const override
    {
        std::stringstream ss;

        ss << NAME << ":" << std::endl;
        ss << "  mode: " << +mode << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;
        ss << "  thrust: " << thrust << std::endl;
        ss << "  yaw: " << yaw << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << x;                             // offset: 0
        map << y;                             // offset: 4
        map << z;                             // offset: 8
        map << thrust;                        // offset: 12
        map << yaw;                           // offset: 16
        map << mode;                          // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> x;                             // offset: 0
        map >> y;                             // offset: 4
        map >> z;                             // offset: 8
        map >> thrust;                        // offset: 12
        map >> yaw;                           // offset: 16
        map >> mode;                          // offset: 20
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink
