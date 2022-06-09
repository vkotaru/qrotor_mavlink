#pragma once
// MESSAGE OFFBOARD_CONTROL PACKING

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL 180


typedef struct __mavlink_offboard_control_t {
 float w; /*<   Extra channel */
 float x; /*<  X-channel, (typically, thrust in X-direction [N])*/
 float y; /*<  Y-channel, (typically, thrust in Y-direction [N])*/
 float z; /*<  Z-channel, (typically, thrust in Z-direction [N])*/
 float thrust; /*<  Thrust, (typically, thrust-scale [N])*/
 float yaw; /*<  Yaw setpoint [rad]*/
 uint8_t mode; /*<  Offboard control mode, see OFFBOARD_CONTROL_MODE*/
 uint8_t act; /*<  Acutator bit map*/
} mavlink_offboard_control_t;

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN 26
#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN 26
#define MAVLINK_MSG_ID_180_LEN 26
#define MAVLINK_MSG_ID_180_MIN_LEN 26

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC 83
#define MAVLINK_MSG_ID_180_CRC 83



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
    180, \
    "OFFBOARD_CONTROL", \
    8, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_offboard_control_t, mode) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_offboard_control_t, w) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_offboard_control_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_offboard_control_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_offboard_control_t, z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_offboard_control_t, thrust) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_offboard_control_t, yaw) }, \
         { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_offboard_control_t, act) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
    "OFFBOARD_CONTROL", \
    8, \
    {  { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_offboard_control_t, mode) }, \
         { "w", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_offboard_control_t, w) }, \
         { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_offboard_control_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_offboard_control_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_offboard_control_t, z) }, \
         { "thrust", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_offboard_control_t, thrust) }, \
         { "yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_offboard_control_t, yaw) }, \
         { "act", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_offboard_control_t, act) }, \
         } \
}
#endif

/**
 * @brief Pack a offboard_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param w   Extra channel 
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 * @param yaw  Yaw setpoint [rad]
 * @param act  Acutator bit map
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t mode, float w, float x, float y, float z, float thrust, float yaw, uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_float(buf, 0, w);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, thrust);
    _mav_put_float(buf, 20, yaw);
    _mav_put_uint8_t(buf, 24, mode);
    _mav_put_uint8_t(buf, 25, act);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
    mavlink_offboard_control_t packet;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;
    packet.yaw = yaw;
    packet.mode = mode;
    packet.act = act;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
}

/**
 * @brief Pack a offboard_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param w   Extra channel 
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 * @param yaw  Yaw setpoint [rad]
 * @param act  Acutator bit map
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t mode,float w,float x,float y,float z,float thrust,float yaw,uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_float(buf, 0, w);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, thrust);
    _mav_put_float(buf, 20, yaw);
    _mav_put_uint8_t(buf, 24, mode);
    _mav_put_uint8_t(buf, 25, act);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
    mavlink_offboard_control_t packet;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;
    packet.yaw = yaw;
    packet.mode = mode;
    packet.act = act;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
}

/**
 * @brief Encode a offboard_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
    return mavlink_msg_offboard_control_pack(system_id, component_id, msg, offboard_control->mode, offboard_control->w, offboard_control->x, offboard_control->y, offboard_control->z, offboard_control->thrust, offboard_control->yaw, offboard_control->act);
}

/**
 * @brief Encode a offboard_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
    return mavlink_msg_offboard_control_pack_chan(system_id, component_id, chan, msg, offboard_control->mode, offboard_control->w, offboard_control->x, offboard_control->y, offboard_control->z, offboard_control->thrust, offboard_control->yaw, offboard_control->act);
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 *
 * @param mode  Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param w   Extra channel 
 * @param x  X-channel, (typically, thrust in X-direction [N])
 * @param y  Y-channel, (typically, thrust in Y-direction [N])
 * @param z  Z-channel, (typically, thrust in Z-direction [N])
 * @param thrust  Thrust, (typically, thrust-scale [N])
 * @param yaw  Yaw setpoint [rad]
 * @param act  Acutator bit map
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_offboard_control_send(mavlink_channel_t chan, uint8_t mode, float w, float x, float y, float z, float thrust, float yaw, uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
    _mav_put_float(buf, 0, w);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, thrust);
    _mav_put_float(buf, 20, yaw);
    _mav_put_uint8_t(buf, 24, mode);
    _mav_put_uint8_t(buf, 25, act);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    mavlink_offboard_control_t packet;
    packet.w = w;
    packet.x = x;
    packet.y = y;
    packet.z = z;
    packet.thrust = thrust;
    packet.yaw = yaw;
    packet.mode = mode;
    packet.act = act;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_offboard_control_send_struct(mavlink_channel_t chan, const mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_offboard_control_send(chan, offboard_control->mode, offboard_control->w, offboard_control->x, offboard_control->y, offboard_control->z, offboard_control->thrust, offboard_control->yaw, offboard_control->act);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)offboard_control, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_offboard_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, float w, float x, float y, float z, float thrust, float yaw, uint8_t act)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, w);
    _mav_put_float(buf, 4, x);
    _mav_put_float(buf, 8, y);
    _mav_put_float(buf, 12, z);
    _mav_put_float(buf, 16, thrust);
    _mav_put_float(buf, 20, yaw);
    _mav_put_uint8_t(buf, 24, mode);
    _mav_put_uint8_t(buf, 25, act);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    mavlink_offboard_control_t *packet = (mavlink_offboard_control_t *)msgbuf;
    packet->w = w;
    packet->x = x;
    packet->y = y;
    packet->z = z;
    packet->thrust = thrust;
    packet->yaw = yaw;
    packet->mode = mode;
    packet->act = act;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_MIN_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE OFFBOARD_CONTROL UNPACKING


/**
 * @brief Get field mode from offboard_control message
 *
 * @return  Offboard control mode, see OFFBOARD_CONTROL_MODE
 */
static inline uint8_t mavlink_msg_offboard_control_get_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field w from offboard_control message
 *
 * @return   Extra channel 
 */
static inline float mavlink_msg_offboard_control_get_w(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field x from offboard_control message
 *
 * @return  X-channel, (typically, thrust in X-direction [N])
 */
static inline float mavlink_msg_offboard_control_get_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field y from offboard_control message
 *
 * @return  Y-channel, (typically, thrust in Y-direction [N])
 */
static inline float mavlink_msg_offboard_control_get_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field z from offboard_control message
 *
 * @return  Z-channel, (typically, thrust in Z-direction [N])
 */
static inline float mavlink_msg_offboard_control_get_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field thrust from offboard_control message
 *
 * @return  Thrust, (typically, thrust-scale [N])
 */
static inline float mavlink_msg_offboard_control_get_thrust(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field yaw from offboard_control message
 *
 * @return  Yaw setpoint [rad]
 */
static inline float mavlink_msg_offboard_control_get_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field act from offboard_control message
 *
 * @return  Acutator bit map
 */
static inline uint8_t mavlink_msg_offboard_control_get_act(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Decode a offboard_control message into a struct
 *
 * @param msg The message to decode
 * @param offboard_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_offboard_control_decode(const mavlink_message_t* msg, mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    offboard_control->w = mavlink_msg_offboard_control_get_w(msg);
    offboard_control->x = mavlink_msg_offboard_control_get_x(msg);
    offboard_control->y = mavlink_msg_offboard_control_get_y(msg);
    offboard_control->z = mavlink_msg_offboard_control_get_z(msg);
    offboard_control->thrust = mavlink_msg_offboard_control_get_thrust(msg);
    offboard_control->yaw = mavlink_msg_offboard_control_get_yaw(msg);
    offboard_control->mode = mavlink_msg_offboard_control_get_mode(msg);
    offboard_control->act = mavlink_msg_offboard_control_get_act(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN;
        memset(offboard_control, 0, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
    memcpy(offboard_control, _MAV_PAYLOAD(msg), len);
#endif
}
