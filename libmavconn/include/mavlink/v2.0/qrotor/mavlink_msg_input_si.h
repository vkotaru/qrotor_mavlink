#pragma once
// MESSAGE INPUT_SI PACKING

#define MAVLINK_MSG_ID_INPUT_SI 183


typedef struct __mavlink_input_si_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float x; /*<  X-channel, (typically, thrust in X-direction [N])*/
 float y; /*<  Y-channel, (typically, thrust in Y-direction [N])*/
 float z; /*<  Z-channel, (typically, thrust in Z-direction [N])*/
 float thrust; /*<  Thrust, (typically, thrust-scale [N])*/
} mavlink_input_si_t;

#define MAVLINK_MSG_ID_INPUT_SI_LEN 24
#define MAVLINK_MSG_ID_INPUT_SI_MIN_LEN 24
#define MAVLINK_MSG_ID_183_LEN 24
#define MAVLINK_MSG_ID_183_MIN_LEN 24

#define MAVLINK_MSG_ID_INPUT_SI_CRC 208
#define MAVLINK_MSG_ID_183_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INPUT_SI { \
    183, \
    "INPUT_SI", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_input_si_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_input_si_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_input_si_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_input_si_t, z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_input_si_t, thrust) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INPUT_SI { \
    "INPUT_SI", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_input_si_t, time_usec) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_input_si_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_input_si_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_input_si_t, z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_input_si_t, thrust) }, \
         } \
}
#endif

/**
 * @brief Pack a input_si message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_si_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float x, float y, float z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_SI_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_SI_LEN);
#else
    mavlink_input_si_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_SI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_SI;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
}

/**
 * @brief Pack a input_si message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_si_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float x,float y,float z,float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_SI_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, thrust);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_SI_LEN);
#else
    mavlink_input_si_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_SI_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_SI;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
}

/**
 * @brief Encode a input_si struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param input_si C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_si_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_input_si_t* input_si)
{
    return mavlink_msg_input_si_pack(system_id, component_id, msg, input_si->time_usec, input_si->x, input_si->y, input_si->z, input_si->thrust);
}

/**
 * @brief Encode a input_si struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param input_si C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_si_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_input_si_t* input_si)
{
    return mavlink_msg_input_si_pack_chan(system_id, component_id, chan, msg, input_si->time_usec, input_si->x, input_si->y, input_si->z, input_si->thrust);
}

/**
 * @brief Send a input_si message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_input_si_send(mavlink_channel_t chan, uint64_t time_usec, float x, float y, float z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_SI_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_SI, buf, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
#else
    mavlink_input_si_t packet;
    packet.time_usec = time_usec;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_SI, (const char *)&packet, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
#endif
}

/**
 * @brief Send a input_si message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_input_si_send_struct(mavlink_channel_t chan, const mavlink_input_si_t* input_si)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_input_si_send(chan, input_si->time_usec, input_si->x, input_si->y, input_si->z, input_si->thrust);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_SI, (const char *)input_si, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
#endif
}

#if MAVLINK_MSG_ID_INPUT_SI_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_input_si_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float x, float y, float z, float thrust)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, x);
    _mav_put_float(buf, 12, y);
    _mav_put_float(buf, 16, z);
    _mav_put_float(buf, 20, thrust);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_SI, buf, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
#else
    mavlink_input_si_t *packet = (mavlink_input_si_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->thrust = thrust;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_SI, (const char *)packet, MAVLINK_MSG_ID_INPUT_SI_MIN_LEN, MAVLINK_MSG_ID_INPUT_SI_LEN, MAVLINK_MSG_ID_INPUT_SI_CRC);
#endif
}
#endif

#endif

// MESSAGE INPUT_SI UNPACKING


/**
 * @brief Get field time_usec from input_si message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_input_si_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field x from input_si message
 *
 * @return  X-channel, (typically, thrust in X-direction [N])
 */
static inline float mavlink_msg_input_si_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field y from input_si message
 *
 * @return  Y-channel, (typically, thrust in Y-direction [N])
 */
static inline float mavlink_msg_input_si_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field z from input_si message
 *
 * @return  Z-channel, (typically, thrust in Z-direction [N])
 */
static inline float mavlink_msg_input_si_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field thrust from input_si message
 *
 * @return  Thrust, (typically, thrust-scale [N])
 */
static inline float mavlink_msg_input_si_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a input_si message into a struct
 *
 * @param msg The message to decode
 * @param input_si C-struct to decode the message contents into
 */
static inline void mavlink_msg_input_si_decode(const mavlink_message_t* msg, mavlink_input_si_t* input_si)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    input_si->time_usec = mavlink_msg_input_si_get_time_usec(msg);
    input_si->x = mavlink_msg_input_si_get_x(msg);
    input_si->y = mavlink_msg_input_si_get_y(msg);
    input_si->z = mavlink_msg_input_si_get_z(msg);
    input_si->thrust = mavlink_msg_input_si_get_thrust(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INPUT_SI_LEN? msg->len : MAVLINK_MSG_ID_INPUT_SI_LEN;
        memset(input_si, 0, MAVLINK_MSG_ID_INPUT_SI_LEN);
    memcpy(input_si, _MAV_PAYLOAD(msg), len);
#endif
}
