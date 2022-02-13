// MESSAGE INPUT_PWM support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief INPUT_PWM message
 *
 * The Final PWM applied to the drone
 */
struct INPUT_PWM : mavlink::Message {
    static constexpr msgid_t MSG_ID = 184;
    static constexpr size_t LENGTH = 24;
    static constexpr size_t MIN_LENGTH = 24;
    static constexpr uint8_t CRC_EXTRA = 149;
    static constexpr auto NAME = "INPUT_PWM";


    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    float w; /*<  pwm1 */
    float x; /*<  pwm2 */
    float y; /*<  pwm3 */
    float z; /*<  pwm4 */


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
        ss << "  w: " << w << std::endl;
        ss << "  x: " << x << std::endl;
        ss << "  y: " << y << std::endl;
        ss << "  z: " << z << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << w;                             // offset: 8
        map << x;                             // offset: 12
        map << y;                             // offset: 16
        map << z;                             // offset: 20
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> w;                             // offset: 8
        map >> x;                             // offset: 12
        map >> y;                             // offset: 16
        map >> z;                             // offset: 20
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink
