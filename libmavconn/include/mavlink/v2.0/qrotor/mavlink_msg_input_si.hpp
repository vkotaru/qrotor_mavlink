// MESSAGE INPUT_SI support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief INPUT_SI message
 *
 * The Final inputs applied to the drone in body frame
 */
struct INPUT_SI : mavlink::Message {
    static constexpr msgid_t MSG_ID = 183;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 208;
    static constexpr auto NAME = "INPUT_SI";


    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    float x; /*<  X-channel, (typically, thrust in X-direction [N]) */
    float y; /*<  Y-channel, (typically, thrust in Y-direction [N]) */
    float z; /*<  Z-channel, (typically, thrust in Z-direction [N]) */
    float thrust; /*<  Thrust, (typically, thrust-scale [N]) */


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
        ss << "  time_usec: " << time_usec << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;
        ss << "  thrust: " << thrust << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << x;                             // offset: 8
        map << y;                             // offset: 12
        map << z;                             // offset: 16
        map << thrust;                        // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> x;                             // offset: 8
        map >> y;                             // offset: 12
        map >> z;                             // offset: 16
        map >> thrust;                        // offset: 20
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink
