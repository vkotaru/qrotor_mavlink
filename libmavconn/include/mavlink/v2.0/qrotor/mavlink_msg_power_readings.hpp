// MESSAGE POWER_READINGS support class

#pragma once

namespace mavlink {
namespace qrotor {
namespace msg {

/**
 * @brief POWER_READINGS message
 *
 * Power module readings
 */
struct POWER_READINGS : mavlink::Message {
    static constexpr msgid_t MSG_ID = 181;
    static constexpr size_t LENGTH = 32;
    static constexpr size_t MIN_LENGTH = 32;
    static constexpr uint8_t CRC_EXTRA = 60;
    static constexpr auto NAME = "POWER_READINGS";


    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. */
    float voltage; /*<  Voltage in Volts */
    uint64_t v_adc; /*<  Voltage ADC readings */
    float current; /*<  Current in Amperes */
    uint64_t c_adc; /*<  Current ADC readings */


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
        ss << "  voltage: " << voltage << std::endl;
        ss << "  v_adc: " << v_adc << std::endl;
        ss << "  current: " << current << std::endl;
        ss << "  c_adc: " << c_adc << std::endl;

        return ss.str();
    }

    inline void serialize(mavlink::MsgMap &map) const override
    {
        map.reset(MSG_ID, LENGTH);

        map << time_usec;                     // offset: 0
        map << v_adc;                         // offset: 8
        map << c_adc;                         // offset: 16
        map << voltage;                       // offset: 24
        map << current;                       // offset: 28
    }

    inline void deserialize(mavlink::MsgMap &map) override
    {
        map >> time_usec;                     // offset: 0
        map >> v_adc;                         // offset: 8
        map >> c_adc;                         // offset: 16
        map >> voltage;                       // offset: 24
        map >> current;                       // offset: 28
    }
};

} // namespace msg
} // namespace qrotor
} // namespace mavlink
