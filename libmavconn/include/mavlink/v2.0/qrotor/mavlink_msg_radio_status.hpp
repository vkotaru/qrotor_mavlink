// MESSAGE RADIO_STATUS support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief RADIO_STATUS message
 *
 * Status generated by radio and injected into MAVLink stream.
 */
struct RADIO_STATUS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 109;
    static constexpr size_t LENGTH = 9;
    static constexpr size_t MIN_LENGTH = 9;
    static constexpr uint8_t CRC_EXTRA = 185;
    static constexpr auto NAME = "RADIO_STATUS";


    uint8_t rssi; /*<  Local (message sender) recieved signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown. */
    uint8_t remrssi; /*<  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown. */
    uint8_t txbuf; /*< [%] Remaining free transmitter buffer space. */
    uint8_t noise; /*<  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown. */
    uint8_t remnoise; /*<  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown. */
    uint16_t rxerrors; /*<  Count of radio packet receive errors (since boot). */
    uint16_t fixed; /*<  Count of error corrected radio packets (since boot). */


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
        ss << "  rssi: " << +rssi << std::endl;
        ss << "  remrssi: " << +remrssi << std::endl;
        ss << "  txbuf: " << +txbuf << std::endl;
        ss << "  noise: " << +noise << std::endl;
        ss << "  remnoise: " << +remnoise << std::endl;
        ss << "  rxerrors: " << rxerrors << std::endl;
        ss << "  fixed: " << fixed << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << rxerrors;                      // offset: 0
        map << fixed;                         // offset: 2
        map << rssi;                          // offset: 4
        map << remrssi;                       // offset: 5
        map << txbuf;                         // offset: 6
        map << noise;                         // offset: 7
        map << remnoise;                      // offset: 8
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> rxerrors;                      // offset: 0
        map >> fixed;                         // offset: 2
        map >> rssi;                          // offset: 4
        map >> remrssi;                       // offset: 5
        map >> txbuf;                         // offset: 6
        map >> noise;                         // offset: 7
        map >> remnoise;                      // offset: 8
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink