// MESSAGE PX4IO_PARAM PACKING

#define MAVLINK_MSG_ID_PX4IO_PARAM 189

typedef struct __mavlink_px4io_param_t
{
 float param_value; /*< Param value*/
 uint16_t param_id; /*< Onboard parameter id. See PX4IO_PARAMETERS enum*/
 uint8_t system; /*< System ID*/
 uint8_t component; /*< Component ID*/
 uint8_t param_action; /*< Param action. See the MAV_PARAM_ACTION enum*/
 uint8_t status; /*< Message status. See PX4IO_STATUS enum*/
} mavlink_px4io_param_t;

#define MAVLINK_MSG_ID_PX4IO_PARAM_LEN 10
#define MAVLINK_MSG_ID_189_LEN 10

#define MAVLINK_MSG_ID_PX4IO_PARAM_CRC 177
#define MAVLINK_MSG_ID_189_CRC 177



#define MAVLINK_MESSAGE_INFO_PX4IO_PARAM { \
	"PX4IO_PARAM", \
	6, \
	{  { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4io_param_t, param_value) }, \
         { "param_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_px4io_param_t, param_id) }, \
         { "system", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_px4io_param_t, system) }, \
         { "component", NULL, MAVLINK_TYPE_UINT8_T, 0, 7, offsetof(mavlink_px4io_param_t, component) }, \
         { "param_action", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_px4io_param_t, param_action) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 9, offsetof(mavlink_px4io_param_t, status) }, \
         } \
}


/**
 * @brief Pack a px4io_param message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param system System ID
 * @param component Component ID
 * @param param_action Param action. See the MAV_PARAM_ACTION enum
 * @param param_id Onboard parameter id. See PX4IO_PARAMETERS enum
 * @param param_value Param value
 * @param status Message status. See PX4IO_STATUS enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4io_param_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t system, uint8_t component, uint8_t param_action, uint16_t param_id, float param_value, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4IO_PARAM_LEN];
	_mav_put_float(buf, 0, param_value);
	_mav_put_uint16_t(buf, 4, param_id);
	_mav_put_uint8_t(buf, 6, system);
	_mav_put_uint8_t(buf, 7, component);
	_mav_put_uint8_t(buf, 8, param_action);
	_mav_put_uint8_t(buf, 9, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#else
	mavlink_px4io_param_t packet;
	packet.param_value = param_value;
	packet.param_id = param_id;
	packet.system = system;
	packet.component = component;
	packet.param_action = param_action;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PX4IO_PARAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
}

/**
 * @brief Pack a px4io_param message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system System ID
 * @param component Component ID
 * @param param_action Param action. See the MAV_PARAM_ACTION enum
 * @param param_id Onboard parameter id. See PX4IO_PARAMETERS enum
 * @param param_value Param value
 * @param status Message status. See PX4IO_STATUS enum
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4io_param_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t system,uint8_t component,uint8_t param_action,uint16_t param_id,float param_value,uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4IO_PARAM_LEN];
	_mav_put_float(buf, 0, param_value);
	_mav_put_uint16_t(buf, 4, param_id);
	_mav_put_uint8_t(buf, 6, system);
	_mav_put_uint8_t(buf, 7, component);
	_mav_put_uint8_t(buf, 8, param_action);
	_mav_put_uint8_t(buf, 9, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#else
	mavlink_px4io_param_t packet;
	packet.param_value = param_value;
	packet.param_id = param_id;
	packet.system = system;
	packet.component = component;
	packet.param_action = param_action;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PX4IO_PARAM;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
}

/**
 * @brief Encode a px4io_param struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4io_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4io_param_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4io_param_t* px4io_param)
{
	return mavlink_msg_px4io_param_pack(system_id, component_id, msg, px4io_param->system, px4io_param->component, px4io_param->param_action, px4io_param->param_id, px4io_param->param_value, px4io_param->status);
}

/**
 * @brief Encode a px4io_param struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4io_param C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4io_param_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4io_param_t* px4io_param)
{
	return mavlink_msg_px4io_param_pack_chan(system_id, component_id, chan, msg, px4io_param->system, px4io_param->component, px4io_param->param_action, px4io_param->param_id, px4io_param->param_value, px4io_param->status);
}

/**
 * @brief Send a px4io_param message
 * @param chan MAVLink channel to send the message
 *
 * @param system System ID
 * @param component Component ID
 * @param param_action Param action. See the MAV_PARAM_ACTION enum
 * @param param_id Onboard parameter id. See PX4IO_PARAMETERS enum
 * @param param_value Param value
 * @param status Message status. See PX4IO_STATUS enum
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4io_param_send(mavlink_channel_t chan, uint8_t system, uint8_t component, uint8_t param_action, uint16_t param_id, float param_value, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PX4IO_PARAM_LEN];
	_mav_put_float(buf, 0, param_value);
	_mav_put_uint16_t(buf, 4, param_id);
	_mav_put_uint8_t(buf, 6, system);
	_mav_put_uint8_t(buf, 7, component);
	_mav_put_uint8_t(buf, 8, param_action);
	_mav_put_uint8_t(buf, 9, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
#else
	mavlink_px4io_param_t packet;
	packet.param_value = param_value;
	packet.param_id = param_id;
	packet.system = system;
	packet.component = component;
	packet.param_action = param_action;
	packet.status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, (const char *)&packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, (const char *)&packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PX4IO_PARAM_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4io_param_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t system, uint8_t component, uint8_t param_action, uint16_t param_id, float param_value, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, param_value);
	_mav_put_uint16_t(buf, 4, param_id);
	_mav_put_uint8_t(buf, 6, system);
	_mav_put_uint8_t(buf, 7, component);
	_mav_put_uint8_t(buf, 8, param_action);
	_mav_put_uint8_t(buf, 9, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, buf, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
#else
	mavlink_px4io_param_t *packet = (mavlink_px4io_param_t *)msgbuf;
	packet->param_value = param_value;
	packet->param_id = param_id;
	packet->system = system;
	packet->component = component;
	packet->param_action = param_action;
	packet->status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, (const char *)packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN, MAVLINK_MSG_ID_PX4IO_PARAM_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4IO_PARAM, (const char *)packet, MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PX4IO_PARAM UNPACKING


/**
 * @brief Get field system from px4io_param message
 *
 * @return System ID
 */
static inline uint8_t mavlink_msg_px4io_param_get_system(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field component from px4io_param message
 *
 * @return Component ID
 */
static inline uint8_t mavlink_msg_px4io_param_get_component(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  7);
}

/**
 * @brief Get field param_action from px4io_param message
 *
 * @return Param action. See the MAV_PARAM_ACTION enum
 */
static inline uint8_t mavlink_msg_px4io_param_get_param_action(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Get field param_id from px4io_param message
 *
 * @return Onboard parameter id. See PX4IO_PARAMETERS enum
 */
static inline uint16_t mavlink_msg_px4io_param_get_param_id(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field param_value from px4io_param message
 *
 * @return Param value
 */
static inline float mavlink_msg_px4io_param_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field status from px4io_param message
 *
 * @return Message status. See PX4IO_STATUS enum
 */
static inline uint8_t mavlink_msg_px4io_param_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  9);
}

/**
 * @brief Decode a px4io_param message into a struct
 *
 * @param msg The message to decode
 * @param px4io_param C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4io_param_decode(const mavlink_message_t* msg, mavlink_px4io_param_t* px4io_param)
{
#if MAVLINK_NEED_BYTE_SWAP
	px4io_param->param_value = mavlink_msg_px4io_param_get_param_value(msg);
	px4io_param->param_id = mavlink_msg_px4io_param_get_param_id(msg);
	px4io_param->system = mavlink_msg_px4io_param_get_system(msg);
	px4io_param->component = mavlink_msg_px4io_param_get_component(msg);
	px4io_param->param_action = mavlink_msg_px4io_param_get_param_action(msg);
	px4io_param->status = mavlink_msg_px4io_param_get_status(msg);
#else
	memcpy(px4io_param, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PX4IO_PARAM_LEN);
#endif
}
