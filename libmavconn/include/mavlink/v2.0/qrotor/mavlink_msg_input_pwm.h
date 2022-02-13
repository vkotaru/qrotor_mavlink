#pragma once
// MESSAGE INPUT_PWM PACKING

#define MAVLINK_MSG_ID_INPUT_PWM 184


typedef struct __mavlink_input_pwm_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float w; /*<  pwm1*/
 float x; /*<  pwm2*/
 float y; /*<  pwm3*/
 float z; /*<  pwm4*/
} mavlink_input_pwm_t;

#define MAVLINK_MSG_ID_INPUT_PWM_LEN 24
#define MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN 24
#define MAVLINK_MSG_ID_184_LEN 24
#define MAVLINK_MSG_ID_184_MIN_LEN 24

#define MAVLINK_MSG_ID_INPUT_PWM_CRC 149
#define MAVLINK_MSG_ID_184_CRC 149



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INPUT_PWM { \
    184, \
    "INPUT_PWM", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_input_pwm_t, time_usec) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_input_pwm_t, w) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_input_pwm_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_input_pwm_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_input_pwm_t, z) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INPUT_PWM { \
    "INPUT_PWM", \
    5, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_input_pwm_t, time_usec) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_input_pwm_t, w) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_input_pwm_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_input_pwm_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_input_pwm_t, z) }, \
         } \
}
#endif

/**
 * @brief Pack a input_pwm message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  pwm1
 * @param x  pwm2
 * @param y  pwm3
 * @param z  pwm4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_pwm_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float w, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_PWM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_PWM_LEN);
#else
    mavlink_input_pwm_t packet;
    packet.time_usec = time_usec;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_PWM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_PWM;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
}

/**
 * @brief Pack a input_pwm message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  pwm1
 * @param x  pwm2
 * @param y  pwm3
 * @param z  pwm4
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_pwm_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float w,float x,float y,float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_PWM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_PWM_LEN);
#else
    mavlink_input_pwm_t packet;
    packet.time_usec = time_usec;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_PWM_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_PWM;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
}

/**
 * @brief Encode a input_pwm struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param input_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_pwm_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_input_pwm_t* input_pwm)
{
    return mavlink_msg_input_pwm_pack(system_id, component_id, msg, input_pwm->time_usec, input_pwm->w, input_pwm->x, input_pwm->y, input_pwm->z);
}

/**
 * @brief Encode a input_pwm struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param input_pwm C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_pwm_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_input_pwm_t* input_pwm)
{
    return mavlink_msg_input_pwm_pack_chan(system_id, component_id, chan, msg, input_pwm->time_usec, input_pwm->w, input_pwm->x, input_pwm->y, input_pwm->z);
}

/**
 * @brief Send a input_pwm message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param w  pwm1
 * @param x  pwm2
 * @param y  pwm3
 * @param z  pwm4
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_input_pwm_send(mavlink_channel_t chan, uint64_t time_usec, float w, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_PWM_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_PWM, buf, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
#else
    mavlink_input_pwm_t packet;
    packet.time_usec = time_usec;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_PWM, (const char *)&packet, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
#endif
}

/**
 * @brief Send a input_pwm message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_input_pwm_send_struct(mavlink_channel_t chan, const mavlink_input_pwm_t* input_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_input_pwm_send(chan, input_pwm->time_usec, input_pwm->w, input_pwm->x, input_pwm->y, input_pwm->z);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_PWM, (const char *)input_pwm, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
#endif
}

#if MAVLINK_MSG_ID_INPUT_PWM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_input_pwm_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float w, float x, float y, float z)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, w);
    _mav_put_float(buf, 12, x);
    _mav_put_float(buf, 16, y);
    _mav_put_float(buf, 20, z);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_PWM, buf, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
#else
    mavlink_input_pwm_t *packet = (mavlink_input_pwm_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->w = w;
    packet->x = x;
    packet->y = y;
    packet->z = z;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_PWM, (const char *)packet, MAVLINK_MSG_ID_INPUT_PWM_MIN_LEN, MAVLINK_MSG_ID_INPUT_PWM_LEN, MAVLINK_MSG_ID_INPUT_PWM_CRC);
#endif
}
#endif

#endif

// MESSAGE INPUT_PWM UNPACKING


/**
 * @brief Get field time_usec from input_pwm message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_input_pwm_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field w from input_pwm message
 *
 * @return  pwm1
 */
static inline float mavlink_msg_input_pwm_get_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field x from input_pwm message
 *
 * @return  pwm2
 */
static inline float mavlink_msg_input_pwm_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field y from input_pwm message
 *
 * @return  pwm3
 */
static inline float mavlink_msg_input_pwm_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field z from input_pwm message
 *
 * @return  pwm4
 */
static inline float mavlink_msg_input_pwm_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Decode a input_pwm message into a struct
 *
 * @param msg The message to decode
 * @param input_pwm C-struct to decode the message contents into
 */
static inline void mavlink_msg_input_pwm_decode(const mavlink_message_t* msg, mavlink_input_pwm_t* input_pwm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    input_pwm->time_usec = mavlink_msg_input_pwm_get_time_usec(msg);
    input_pwm->w = mavlink_msg_input_pwm_get_w(msg);
    input_pwm->x = mavlink_msg_input_pwm_get_x(msg);
    input_pwm->y = mavlink_msg_input_pwm_get_y(msg);
    input_pwm->z = mavlink_msg_input_pwm_get_z(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INPUT_PWM_LEN? msg->len : MAVLINK_MSG_ID_INPUT_PWM_LEN;
        memset(input_pwm, 0, MAVLINK_MSG_ID_INPUT_PWM_LEN);
    memcpy(input_pwm, _MAV_PAYLOAD(msg), len);
#endif
}
