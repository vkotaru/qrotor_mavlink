#include "qrotor_mavlink.h"

namespace qrotor_mavlink {

QrotorMavlink::QrotorMavlink() {
  server = std::make_shared<MAVConnUDP>();
  client = std::make_shared<MAVConnUDP>();
}

QrotorMavlink::QrotorMavlink(uint8_t system_id, uint8_t component_id,
                             std::string bind_host, unsigned short bind_port,
                             std::string remote_host,
                             unsigned short remote_port) {
  client = std::make_shared<MAVConnUDP>(system_id, component_id, bind_host,
                                        bind_port, remote_host, remote_port);
}

QrotorMavlink::~QrotorMavlink() = default;

void QrotorMavlink::recv_message(const mavlink_message_t *message,
                                 const Framing framing) {
  // printf("Got message %u, len: %u, framing: %d\n", message->msgid, message->len,
  //        int(framing));
  message_buffer_mutex_.lock();       
  msg_buffer_queue_.push(*message);
  message_buffer_mutex_.unlock();
  cond.notify_one();
}

bool QrotorMavlink::wait_one() {
  std::unique_lock<std::mutex> lock(mutex);
  return cond.wait_for(lock, std::chrono::seconds(2)) ==
         std::cv_status::no_timeout;
}

void QrotorMavlink::decode() {
  while (!msg_buffer_queue_.empty()) {
    mavlink_message_t buf_msg = msg_buffer_queue_.front();

    switch (buf_msg.msgid) {
    case mavlink_msg::HEARTBEAT::MSG_ID:
      this->deserialize_heartbeat(buf_msg);
      break;
    case mavlink_msg::ATTITUDE_QUATERNION::MSG_ID:
      this->deserialize_attitude_quaternion(buf_msg);
      break;
    case mavlink_msg::SCALED_IMU::MSG_ID:
      this->deserialize_imu(buf_msg);
      break;
    case mavlink_msg::BATTERY_STATUS::MSG_ID:
      this->deserialize_battery_status(buf_msg);
      break;
    case mavlink_msg::OFFBOARD_CONTROL::MSG_ID:
      this->deserialize_offboard_control(buf_msg);
      break;

    default:
      break;
    }
    msg_buffer_queue_.pop();
  }
}

bool QrotorMavlink::init() {

  try {
    client->connect(std::bind(&QrotorMavlink::recv_message, this,
                              std::placeholders::_1, std::placeholders::_2));
    initialized = true;

  } catch (const std::exception &e) {
    std::cout << " Exception in QrotorMavlink::init: '" << e.what() << "'\n";
    initialized = false;
  }
  return initialized;
}

void QrotorMavlink::send_heartbeat(const uint8_t base_mode,
                                   const uint32_t custom_mode,
                                   const uint8_t system_status) {
  mavlink_msg::HEARTBEAT hb = {};
  hb.type = int(mavlink::minimal::MAV_TYPE::QUADROTOR);
  hb.autopilot = int(mavlink::minimal::MAV_AUTOPILOT::GENERIC);
  hb.base_mode = base_mode;
  hb.custom_mode = custom_mode;
  hb.system_status = system_status;
  client->send_message(hb);
}

void QrotorMavlink::send_imu(uint64_t timestamp_us,
                             const Eigen::Vector3f &accel,
                             const Eigen::Vector3f &gyro,
                             const Eigen::Vector3f &mag) {

  mavlink_msg::SCALED_IMU imu = {};
  imu.time_boot_ms = timestamp_us;
  imu.xacc = accel(0);
  imu.yacc = accel(1);
  imu.zacc = accel(2);
  imu.xgyro = gyro(0);
  imu.ygyro = gyro(1);
  imu.zgyro = gyro(2);
  imu.xmag = mag(0);
  imu.ymag = mag(1);
  imu.zmag = mag(2);
  client->send_message(imu);
}

void QrotorMavlink::send_attitude_quaternion(
    uint64_t timestamp_us, const Eigen::Quaternionf &q,
    const Eigen::Vector3f &angular_velocity) {

  mavlink_msg::ATTITUDE_QUATERNION att = {};
  att.time_boot_ms = timestamp_us;
  att.q1 = q.w();
  att.q2 = q.x();
  att.q3 = q.y();
  att.q4 = q.z();
  att.rollspeed = angular_velocity(0);
  att.pitchspeed = angular_velocity(1);
  att.yawspeed = angular_velocity(2);
  client->send_message(att);
}

void QrotorMavlink::send_battery_status(const uint16_t voltage,
                                        const int16_t current) {
  mavlink_msg::BATTERY_STATUS batt = {};
  batt.voltages[0] = (voltage);
  batt.current_battery = current;
  client->send_message(batt);
}

void QrotorMavlink::send_offboard_control(const uint8_t mode, const float x,
                                          const float y, const float z,
                                          const float thrust, const float yaw) {
  mavlink_msg::OFFBOARD_CONTROL offb_ctrl = {};
  offb_ctrl.mode = mode;
  offb_ctrl.x = x;
  offb_ctrl.y = y;
  offb_ctrl.z = z;
  offb_ctrl.thrust = thrust;
  offb_ctrl.yaw = yaw;
  client->send_message(offb_ctrl);
}

mavlink_msg::HEARTBEAT
QrotorMavlink::deserialize_heartbeat(mavlink_message_t msg) {
  mavlink::MsgMap map(msg);
  mavlink_msg::HEARTBEAT s;
  s.deserialize(map);
  // std::cout << s.to_yaml() << std::endl;
  return s;
}

mavlink_msg::ATTITUDE_QUATERNION
QrotorMavlink::deserialize_attitude_quaternion(mavlink_message_t msg) {
  mavlink::MsgMap map(msg);
  mavlink_msg::ATTITUDE_QUATERNION s;
  s.deserialize(map);
  // std::cout << s.to_yaml() << std::endl;
  return s;
}

mavlink_msg::SCALED_IMU QrotorMavlink::deserialize_imu(mavlink_message_t msg) {
  mavlink::MsgMap map(msg);
  mavlink_msg::SCALED_IMU s;
  s.deserialize(map);
  // std::cout << s.to_yaml() << std::endl;
  return s;
}

mavlink_msg::BATTERY_STATUS
QrotorMavlink::deserialize_battery_status(mavlink_message_t msg) {
  mavlink::MsgMap map(msg);
  mavlink_msg::BATTERY_STATUS s;
  s.deserialize(map);
  // std::cout << s.to_yaml() << std::endl;
  return s;
}

mavlink_msg::OFFBOARD_CONTROL
QrotorMavlink::deserialize_offboard_control(mavlink_message_t msg) {
  mavlink::MsgMap map(msg);
  mavlink_msg::OFFBOARD_CONTROL s;
  s.deserialize(map);
  // std::cout << s.to_yaml() << std::endl;
  return s;
}

} // namespace qrotor_mavlink
