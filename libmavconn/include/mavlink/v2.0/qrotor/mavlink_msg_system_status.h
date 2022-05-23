#pragma once
// MESSAGE SYSTEM_STATUS PACKING

#define MAVLINK_MSG_ID_SYSTEM_STATUS 185


typedef struct __mavlink_system_status_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float w; /*<  freq*/
} mavlink_system_status_t;

#define MAVLINK_MSG_ID_SYSTEM_STATUS_LEN 12
#define MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN 12
#define MAVLINK_MSG_ID_185_LEN 12
#define MAVLINK_MSG_ID_185_MIN_LEN 12

#define MAVLINK_MSG_ID_SYSTEM_STATUS_CRC 17
#define MAVLINK_MSG_ID_185_CRC 17



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYSTEM_STATUS { \
    185, \
    "SYSTEM_STATUS", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_status_t, time_usec) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_system_status_t, w) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYSTEM_STATUS { \
    "SYSTEM_STATUS", \
    2, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_system_status_t, time_usec) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_system_status_t, w) }, \
         } \
}
#endif

/**
 * @brief Pack a system_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  freq
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN);
#else
    mavlink_system_status_t packet;
    packet.time_usec = time_usec;
    packet.w = w;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
}

/**
 * @brief Pack a system_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  freq
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN);
#else
    mavlink_system_status_t packet;
    packet.time_usec = time_usec;
    packet.w = w;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
}

/**
 * @brief Encode a system_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_status_t* system_status)
{
    return mavlink_msg_system_status_pack(system_id, component_id, msg, system_status->time_usec, system_status->w);
}

/**
 * @brief Encode a system_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_system_status_t* system_status)
{
    return mavlink_msg_system_status_pack_chan(system_id, component_id, chan, msg, system_status->time_usec, system_status->w);
}

/**
 * @brief Send a system_status message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  freq
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_status_send(mavlink_channel_t chan, uint64_t time_usec, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, buf, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
#else
    mavlink_system_status_t packet;
    packet.time_usec = time_usec;
    packet.w = w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
#endif
}

/**
 * @brief Send a system_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_system_status_send_struct(mavlink_channel_t chan, const mavlink_system_status_t* system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_system_status_send(chan, system_status->time_usec, system_status->w);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, (const char *)system_status, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYSTEM_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_system_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float w)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, buf, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
#else
    mavlink_system_status_t *packet = (mavlink_system_status_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->w = w;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_STATUS, (const char *)packet, MAVLINK_MSG_ID_SYSTEM_STATUS_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN, MAVLINK_MSG_ID_SYSTEM_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SYSTEM_STATUS UNPACKING


/**
 * @brief Get field time_usec from system_status message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_system_status_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field w from system_status message
 *
 * @return  freq
 */
static inline float mavlink_msg_system_status_get_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a system_status message into a struct
 *
 * @param msg The message to decode
 * @param system_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_status_decode(const mavlink_message_t* msg, mavlink_system_status_t* system_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    system_status->time_usec = mavlink_msg_system_status_get_time_usec(msg);
    system_status->w = mavlink_msg_system_status_get_w(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYSTEM_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SYSTEM_STATUS_LEN;
        memset(system_status, 0, MAVLINK_MSG_ID_SYSTEM_STATUS_LEN);
    memcpy(system_status, _MAV_PAYLOAD(msg), len);
#endif
}
