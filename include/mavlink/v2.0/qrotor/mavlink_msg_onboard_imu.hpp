// MESSAGE ONBOARD_IMU support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief ONBOARD_IMU message
 *
 * The IMU readings in SI units in NWU body frame
 */
struct ONBOARD_IMU : mavlink::Message {
    static constexpr msgid_t MSG_ID = 182;
    static constexpr size_t LENGTH = 44;
    static constexpr size_t MIN_LENGTH = 44;
    static constexpr uint8_t CRC_EXTRA = 153;
    static constexpr auto NAME = "ONBOARD_IMU";


    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    float xacc; /*< [m/s/s] X acceleration */
    float yacc; /*< [m/s/s] Y acceleration */
    float zacc; /*< [m/s/s] Z acceleration */
    float xgyro; /*< [rad/s] Angular speed around X axis in body frame */
    float ygyro; /*< [rad/s] Angular speed around Y axis in body frame */
    float zgyro; /*< [rad/s] Angular speed around Z axis in body frame */
    float xmag; /*< [gauss] X Magnetic field */
    float ymag; /*< [gauss] Y Magnetic field */
    float zmag; /*< [gauss] Z Magnetic field */


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
        ss << "  xacc: " << xacc << std::endl;
        ss << "  yacc: " << yacc << std::endl;
        ss << "  zacc: " << zacc << std::endl;
        ss << "  xgyro: " << xgyro << std::endl;
        ss << "  ygyro: " << ygyro << std::endl;
        ss << "  zgyro: " << zgyro << std::endl;
        ss << "  xmag: " << xmag << std::endl;
        ss << "  ymag: " << ymag << std::endl;
        ss << "  zmag: " << zmag << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << xacc;                          // offset: 8
        map << yacc;                          // offset: 12
        map << zacc;                          // offset: 16
        map << xgyro;                         // offset: 20
        map << ygyro;                         // offset: 24
        map << zgyro;                         // offset: 28
        map << xmag;                          // offset: 32
        map << ymag;                          // offset: 36
        map << zmag;                          // offset: 40
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> xacc;                          // offset: 8
        map >> yacc;                          // offset: 12
        map >> zacc;                          // offset: 16
        map >> xgyro;                         // offset: 20
        map >> ygyro;                         // offset: 24
        map >> zgyro;                         // offset: 28
        map >> xmag;                          // offset: 32
        map >> ymag;                          // offset: 36
        map >> zmag;                          // offset: 40
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink
