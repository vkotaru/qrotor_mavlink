#pragma once
// MESSAGE ONBOARD_IMU PACKING

#define MAVLINK_MSG_ID_ONBOARD_IMU 182


typedef struct __mavlink_onboard_imu_t {
 uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
 float xacc; /*< [m/s/s] X acceleration*/
 float yacc; /*< [m/s/s] Y acceleration*/
 float zacc; /*< [m/s/s] Z acceleration*/
 float xgyro; /*< [rad/s] Angular speed around X axis in body frame*/
 float ygyro; /*< [rad/s] Angular speed around Y axis in body frame*/
 float zgyro; /*< [rad/s] Angular speed around Z axis in body frame*/
 float xmag; /*< [gauss] X Magnetic field*/
 float ymag; /*< [gauss] Y Magnetic field*/
 float zmag; /*< [gauss] Z Magnetic field*/
} mavlink_onboard_imu_t;

#define MAVLINK_MSG_ID_ONBOARD_IMU_LEN 44
#define MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN 44
#define MAVLINK_MSG_ID_182_LEN 44
#define MAVLINK_MSG_ID_182_MIN_LEN 44

#define MAVLINK_MSG_ID_ONBOARD_IMU_CRC 153
#define MAVLINK_MSG_ID_182_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ONBOARD_IMU { \
    182, \
    "ONBOARD_IMU", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_onboard_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_onboard_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_onboard_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_onboard_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_onboard_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_onboard_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_onboard_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_onboard_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_onboard_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_onboard_imu_t, zmag) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ONBOARD_IMU { \
    "ONBOARD_IMU", \
    10, \
    {  { "time_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_onboard_imu_t, time_usec) }, \
         { "xacc", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_onboard_imu_t, xacc) }, \
         { "yacc", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_onboard_imu_t, yacc) }, \
         { "zacc", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_onboard_imu_t, zacc) }, \
         { "xgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_onboard_imu_t, xgyro) }, \
         { "ygyro", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_onboard_imu_t, ygyro) }, \
         { "zgyro", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_onboard_imu_t, zgyro) }, \
         { "xmag", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_onboard_imu_t, xmag) }, \
         { "ymag", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_onboard_imu_t, ymag) }, \
         { "zmag", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_onboard_imu_t, zmag) }, \
         } \
}
#endif

/**
 * @brief Pack a onboard_imu message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_imu_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_IMU_LEN);
#else
    mavlink_onboard_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_IMU;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
}

/**
 * @brief Pack a onboard_imu message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_onboard_imu_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t time_usec,float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ONBOARD_IMU_LEN);
#else
    mavlink_onboard_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ONBOARD_IMU_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ONBOARD_IMU;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
}

/**
 * @brief Encode a onboard_imu struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param onboard_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_imu_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_onboard_imu_t* onboard_imu)
{
    return mavlink_msg_onboard_imu_pack(system_id, component_id, msg, onboard_imu->time_usec, onboard_imu->xacc, onboard_imu->yacc, onboard_imu->zacc, onboard_imu->xgyro, onboard_imu->ygyro, onboard_imu->zgyro, onboard_imu->xmag, onboard_imu->ymag, onboard_imu->zmag);
}

/**
 * @brief Encode a onboard_imu struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param onboard_imu C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_onboard_imu_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_onboard_imu_t* onboard_imu)
{
    return mavlink_msg_onboard_imu_pack_chan(system_id, component_id, chan, msg, onboard_imu->time_usec, onboard_imu->xacc, onboard_imu->yacc, onboard_imu->zacc, onboard_imu->xgyro, onboard_imu->ygyro, onboard_imu->zgyro, onboard_imu->xmag, onboard_imu->ymag, onboard_imu->zmag);
}

/**
 * @brief Send a onboard_imu message
 * @param chan MAVLink channel to send the message
 *
 * @param time_usec [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 * @param xacc [m/s/s] X acceleration
 * @param yacc [m/s/s] Y acceleration
 * @param zacc [m/s/s] Z acceleration
 * @param xgyro [rad/s] Angular speed around X axis in body frame
 * @param ygyro [rad/s] Angular speed around Y axis in body frame
 * @param zgyro [rad/s] Angular speed around Z axis in body frame
 * @param xmag [gauss] X Magnetic field
 * @param ymag [gauss] Y Magnetic field
 * @param zmag [gauss] Z Magnetic field
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_onboard_imu_send(mavlink_channel_t chan, uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ONBOARD_IMU_LEN];
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_IMU, buf, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
#else
    mavlink_onboard_imu_t packet;
    packet.time_usec = time_usec;
    packet.xacc = xacc;
    packet.yacc = yacc;
    packet.zacc = zacc;
    packet.xgyro = xgyro;
    packet.ygyro = ygyro;
    packet.zgyro = zgyro;
    packet.xmag = xmag;
    packet.ymag = ymag;
    packet.zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_IMU, (const char *)&packet, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
#endif
}

/**
 * @brief Send a onboard_imu message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_onboard_imu_send_struct(mavlink_channel_t chan, const mavlink_onboard_imu_t* onboard_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_onboard_imu_send(chan, onboard_imu->time_usec, onboard_imu->xacc, onboard_imu->yacc, onboard_imu->zacc, onboard_imu->xgyro, onboard_imu->ygyro, onboard_imu->zgyro, onboard_imu->xmag, onboard_imu->ymag, onboard_imu->zmag);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_IMU, (const char *)onboard_imu, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
#endif
}

#if MAVLINK_MSG_ID_ONBOARD_IMU_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_onboard_imu_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t time_usec, float xacc, float yacc, float zacc, float xgyro, float ygyro, float zgyro, float xmag, float ymag, float zmag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_usec);
    _mav_put_float(buf, 8, xacc);
    _mav_put_float(buf, 12, yacc);
    _mav_put_float(buf, 16, zacc);
    _mav_put_float(buf, 20, xgyro);
    _mav_put_float(buf, 24, ygyro);
    _mav_put_float(buf, 28, zgyro);
    _mav_put_float(buf, 32, xmag);
    _mav_put_float(buf, 36, ymag);
    _mav_put_float(buf, 40, zmag);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_IMU, buf, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
#else
    mavlink_onboard_imu_t *packet = (mavlink_onboard_imu_t *)msgbuf;
    packet->time_usec = time_usec;
    packet->xacc = xacc;
    packet->yacc = yacc;
    packet->zacc = zacc;
    packet->xgyro = xgyro;
    packet->ygyro = ygyro;
    packet->zgyro = zgyro;
    packet->xmag = xmag;
    packet->ymag = ymag;
    packet->zmag = zmag;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ONBOARD_IMU, (const char *)packet, MAVLINK_MSG_ID_ONBOARD_IMU_MIN_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_LEN, MAVLINK_MSG_ID_ONBOARD_IMU_CRC);
#endif
}
#endif

#endif

// MESSAGE ONBOARD_IMU UNPACKING


/**
 * @brief Get field time_usec from onboard_imu message
 *
 * @return [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.
 */
static inline uint64_t mavlink_msg_onboard_imu_get_time_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field xacc from onboard_imu message
 *
 * @return [m/s/s] X acceleration
 */
static inline float mavlink_msg_onboard_imu_get_xacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field yacc from onboard_imu message
 *
 * @return [m/s/s] Y acceleration
 */
static inline float mavlink_msg_onboard_imu_get_yacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field zacc from onboard_imu message
 *
 * @return [m/s/s] Z acceleration
 */
static inline float mavlink_msg_onboard_imu_get_zacc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field xgyro from onboard_imu message
 *
 * @return [rad/s] Angular speed around X axis in body frame
 */
static inline float mavlink_msg_onboard_imu_get_xgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field ygyro from onboard_imu message
 *
 * @return [rad/s] Angular speed around Y axis in body frame
 */
static inline float mavlink_msg_onboard_imu_get_ygyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field zgyro from onboard_imu message
 *
 * @return [rad/s] Angular speed around Z axis in body frame
 */
static inline float mavlink_msg_onboard_imu_get_zgyro(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field xmag from onboard_imu message
 *
 * @return [gauss] X Magnetic field
 */
static inline float mavlink_msg_onboard_imu_get_xmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ymag from onboard_imu message
 *
 * @return [gauss] Y Magnetic field
 */
static inline float mavlink_msg_onboard_imu_get_ymag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field zmag from onboard_imu message
 *
 * @return [gauss] Z Magnetic field
 */
static inline float mavlink_msg_onboard_imu_get_zmag(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a onboard_imu message into a struct
 *
 * @param msg The message to decode
 * @param onboard_imu C-struct to decode the message contents into
 */
static inline void mavlink_msg_onboard_imu_decode(const mavlink_message_t* msg, mavlink_onboard_imu_t* onboard_imu)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    onboard_imu->time_usec = mavlink_msg_onboard_imu_get_time_usec(msg);
    onboard_imu->xacc = mavlink_msg_onboard_imu_get_xacc(msg);
    onboard_imu->yacc = mavlink_msg_onboard_imu_get_yacc(msg);
    onboard_imu->zacc = mavlink_msg_onboard_imu_get_zacc(msg);
    onboard_imu->xgyro = mavlink_msg_onboard_imu_get_xgyro(msg);
    onboard_imu->ygyro = mavlink_msg_onboard_imu_get_ygyro(msg);
    onboard_imu->zgyro = mavlink_msg_onboard_imu_get_zgyro(msg);
    onboard_imu->xmag = mavlink_msg_onboard_imu_get_xmag(msg);
    onboard_imu->ymag = mavlink_msg_onboard_imu_get_ymag(msg);
    onboard_imu->zmag = mavlink_msg_onboard_imu_get_zmag(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ONBOARD_IMU_LEN? msg->len : MAVLINK_MSG_ID_ONBOARD_IMU_LEN;
        memset(onboard_imu, 0, MAVLINK_MSG_ID_ONBOARD_IMU_LEN);
    memcpy(onboard_imu, _MAV_PAYLOAD(msg), len);
#endif
}
