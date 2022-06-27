#pragma once
// MESSAGE POWER_READINGS PACKING

#define MAVLINK_MSG_ID_POWER_READINGS 181


typedef struct __mavlink_power_readings_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 uint64_t v_adc; /*<  Voltage ADC readings*/
 uint64_t c_adc; /*<  Current ADC readings*/
 float voltage; /*<  Voltage in Volts*/
 float current; /*<  Current in Amperes*/
} mavlink_power_readings_t;

#define MAVLINK_MSG_ID_POWER_READINGS_LEN 32
#define MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN 32
#define MAVLINK_MSG_ID_181_LEN 32
#define MAVLINK_MSG_ID_181_MIN_LEN 32

#define MAVLINK_MSG_ID_POWER_READINGS_CRC 60
#define MAVLINK_MSG_ID_181_CRC 60



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_POWER_READINGS { \
    181, \
    "POWER_READINGS", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_power_readings_t, time_usec) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_power_readings_t, voltage) }, \
         { "v_adc", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_power_readings_t, v_adc) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_power_readings_t, current) }, \
         { "c_adc", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_power_readings_t, c_adc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_POWER_READINGS { \
    "POWER_READINGS", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_power_readings_t, time_usec) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_power_readings_t, voltage) }, \
         { "v_adc", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_power_readings_t, v_adc) }, \
         { "current", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_power_readings_t, current) }, \
         { "c_adc", NULL, MAVLINK_TYPE_UINT64_T, 0, 16, offsetof(mavlink_power_readings_t, c_adc) }, \
         } \
}
#endif

/**
 * @brief Pack a power_readings message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param voltage  Voltage in Volts
 * @param v_adc  Voltage ADC readings
 * @param current  Current in Amperes
 * @param c_adc  Current ADC readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_power_readings_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float voltage, uint64_t v_adc, float current, uint64_t c_adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_READINGS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, v_adc);
    _mav_put_uint64_t(buf, 16, c_adc);
    _mav_put_float(buf, 24, voltage);
    _mav_put_float(buf, 28, current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POWER_READINGS_LEN);
#else
    mavlink_power_readings_t packet;
    packet.time_usec = time_usec;
    packet.v_adc = v_adc;
    packet.c_adc = c_adc;
    packet.voltage = voltage;
    packet.current = current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POWER_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POWER_READINGS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
}

/**
 * @brief Pack a power_readings message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param voltage  Voltage in Volts
 * @param v_adc  Voltage ADC readings
 * @param current  Current in Amperes
 * @param c_adc  Current ADC readings
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_power_readings_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float voltage,uint64_t v_adc,float current,uint64_t c_adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_READINGS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, v_adc);
    _mav_put_uint64_t(buf, 16, c_adc);
    _mav_put_float(buf, 24, voltage);
    _mav_put_float(buf, 28, current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_POWER_READINGS_LEN);
#else
    mavlink_power_readings_t packet;
    packet.time_usec = time_usec;
    packet.v_adc = v_adc;
    packet.c_adc = c_adc;
    packet.voltage = voltage;
    packet.current = current;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_POWER_READINGS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_POWER_READINGS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
}

/**
 * @brief Encode a power_readings struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param power_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_power_readings_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_power_readings_t* power_readings)
{
    return mavlink_msg_power_readings_pack(system_id, component_id, msg, power_readings->time_usec, power_readings->voltage, power_readings->v_adc, power_readings->current, power_readings->c_adc);
}

/**
 * @brief Encode a power_readings struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param power_readings C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_power_readings_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_power_readings_t* power_readings)
{
    return mavlink_msg_power_readings_pack_chan(system_id, component_id, chan, msg, power_readings->time_usec, power_readings->voltage, power_readings->v_adc, power_readings->current, power_readings->c_adc);
}

/**
 * @brief Send a power_readings message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param voltage  Voltage in Volts
 * @param v_adc  Voltage ADC readings
 * @param current  Current in Amperes
 * @param c_adc  Current ADC readings
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_power_readings_send(mavlink_channel_t chan, uint64_t time_usec, float voltage, uint64_t v_adc, float current, uint64_t c_adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_POWER_READINGS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, v_adc);
    _mav_put_uint64_t(buf, 16, c_adc);
    _mav_put_float(buf, 24, voltage);
    _mav_put_float(buf, 28, current);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_READINGS, buf, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
#else
    mavlink_power_readings_t packet;
    packet.time_usec = time_usec;
    packet.v_adc = v_adc;
    packet.c_adc = c_adc;
    packet.voltage = voltage;
    packet.current = current;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_READINGS, (const char *)&packet, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
#endif
}

/**
 * @brief Send a power_readings message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_power_readings_send_struct(mavlink_channel_t chan, const mavlink_power_readings_t* power_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_power_readings_send(chan, power_readings->time_usec, power_readings->voltage, power_readings->v_adc, power_readings->current, power_readings->c_adc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_READINGS, (const char *)power_readings, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
#endif
}

#if MAVLINK_MSG_ID_POWER_READINGS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_power_readings_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float voltage, uint64_t v_adc, float current, uint64_t c_adc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_uint64_t(buf, 8, v_adc);
    _mav_put_uint64_t(buf, 16, c_adc);
    _mav_put_float(buf, 24, voltage);
    _mav_put_float(buf, 28, current);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_READINGS, buf, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
#else
    mavlink_power_readings_t *packet = (mavlink_power_readings_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->v_adc = v_adc;
    packet->c_adc = c_adc;
    packet->voltage = voltage;
    packet->current = current;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_POWER_READINGS, (const char *)packet, MAVLINK_MSG_ID_POWER_READINGS_MIN_LEN, MAVLINK_MSG_ID_POWER_READINGS_LEN, MAVLINK_MSG_ID_POWER_READINGS_CRC);
#endif
}
#endif

#endif

// MESSAGE POWER_READINGS UNPACKING


/**
 * @brief Get field time_usec from power_readings message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_power_readings_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field voltage from power_readings message
 *
 * @return  Voltage in Volts
 */
static inline float mavlink_msg_power_readings_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field v_adc from power_readings message
 *
 * @return  Voltage ADC readings
 */
static inline uint64_t mavlink_msg_power_readings_get_v_adc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field current from power_readings message
 *
 * @return  Current in Amperes
 */
static inline float mavlink_msg_power_readings_get_current(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field c_adc from power_readings message
 *
 * @return  Current ADC readings
 */
static inline uint64_t mavlink_msg_power_readings_get_c_adc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  16);
}

/**
 * @brief Decode a power_readings message into a struct
 *
 * @param msg The message to decode
 * @param power_readings C-struct to decode the message contents into
 */
static inline void mavlink_msg_power_readings_decode(const mavlink_message_t* msg, mavlink_power_readings_t* power_readings)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    power_readings->time_usec = mavlink_msg_power_readings_get_time_usec(msg);
    power_readings->v_adc = mavlink_msg_power_readings_get_v_adc(msg);
    power_readings->c_adc = mavlink_msg_power_readings_get_c_adc(msg);
    power_readings->voltage = mavlink_msg_power_readings_get_voltage(msg);
    power_readings->current = mavlink_msg_power_readings_get_current(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_POWER_READINGS_LEN? msg->len : MAVLINK_MSG_ID_POWER_READINGS_LEN;
        memset(power_readings, 0, MAVLINK_MSG_ID_POWER_READINGS_LEN);
    memcpy(power_readings, _MAV_PAYLOAD(msg), len);
#endif
}
