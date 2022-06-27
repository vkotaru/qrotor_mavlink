#include "qrotor_mavlink.h"

namespace qrotor_mavlink {

QrotorMavlink::QrotorMavlink(uint8_t system_id, uint8_t component_id,
                             std::string bind_host, unsigned short bind_port,
                             std::string remote_host,
                             unsigned short remote_port)
    : QrotorMavlink(system_id, component_id, bind_host, bind_port, remote_host,
                    remote_port, 100) {}

QrotorMavlink::QrotorMavlink(uint8_t system_id, uint8_t component_id,
                             std::string bind_host, unsigned short bind_port,
                             std::string remote_host,
                             unsigned short remote_port,
                             size_t message_buffer_size)
    : server(bind_port), client(remote_port, remote_host), message_buffer_size(message_buffer_size) {
  message_buffer_.resize(message_buffer_size);
}

QrotorMavlink::~QrotorMavlink() = default;

bool QrotorMavlink::init() {
  int bind_status = server.socket_bind();
  // Return if there is an issue binding
  if (bind_status) {
    return false;
  }
  receive_thread = std::thread(&QrotorMavlink::receive, this);
  return true;
}

void QrotorMavlink::receive() {
  while (!stop_receiving) {
    memset(buf, 0, BUFFER_LENGTH);
    ssize_t recsize = server.receive(buf);
    if (recsize > 0) {
      // Something received - print out all bytes and parse packet
      mavlink_message_t msg;
      mavlink_status_t status;

      printf("Bytes Received: %d\nDatagram: ", (int) recsize);
      for (int i = 0; i < recsize; ++i) {
        temp = buf[i];
        printf("%02x ", (unsigned char) temp);
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          // Packet received
          printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid,
                 msg.len, msg.msgid);
        }
      }

      message_buffer_mutex_.lock();
      message_buffer_.push_back(msg);
      message_buffer_mutex_.unlock();
      printf("\n");
    }
    memset(buf, 0, BUFFER_LENGTH);
  }
}

void QrotorMavlink::send_message(const std::string &message) {
  auto send_status = client.send_message(message);
  std::cout << "\'Sent message: " << message << "\' send status " << send_status << std::endl;
}

void QrotorMavlink::send_message(const mavlink_message_t msg) {

  uint8_t buf[BUFFER_LENGTH];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  client.send_message(buf, len);

}

//
//void QrotorMavlink::recv_message(const mavlink_message_t *message,
//                                 const Framing framing) {
//  // printf("Got message %u, len: %u, framing: %d\n", message->msgid,
//  // message->len, int(framing));
//  message_buffer_mutex_.lock();
//  message_buffer_.push_back(*message);
//  message_buffer_mutex_.unlock();
//  cond.notify_one();
//}
//
//bool QrotorMavlink::wait_one() {
//  std::unique_lock<std::mutex> lock(mutex);
//  return cond.wait_for(lock, std::chrono::seconds(2)) ==
//      std::cv_status::no_timeout;
//}
//
//void QrotorMavlink::decode() {
//  while (!message_buffer_.empty()) {
//    mavlink_message_t buf_msg = message_buffer_.front();
//    message_buffer_.pop_front();
//
//    switch (buf_msg.msgid) {
//    case mavlink_msg::HEARTBEAT::MSG_ID:this->deserialize_heartbeat(buf_msg);
//      break;
//    case mavlink_msg::ATTITUDE_QUATERNION::MSG_ID:this->deserialize_attitude_quaternion(buf_msg);
//      break;
//    case mavlink_msg::SCALED_IMU::MSG_ID:this->deserialize_imu(buf_msg);
//      break;
//    case mavlink_msg::BATTERY_STATUS::MSG_ID:this->deserialize_battery_status(buf_msg);
//      break;
//    case mavlink_msg::OFFBOARD_CONTROL::MSG_ID:this->deserialize_offboard_control(buf_msg);
//      break;
//
//    default:break;
//    }
//  }
//}
//
//void QrotorMavlink::send_heartbeat(const uint8_t base_mode,
//                                   const uint32_t custom_mode,
//                                   const uint8_t system_status) {
//  mavlink_msg::HEARTBEAT hb = {};
//  hb.type = int(mavlink::minimal::MAV_TYPE::QUADROTOR);
//  hb.autopilot = int(mavlink::minimal::MAV_AUTOPILOT::GENERIC);
//  hb.base_mode = base_mode;
//  hb.custom_mode = custom_mode;
//  hb.system_status = system_status;
//  client->send_message(hb);
//}
//
//void QrotorMavlink::send_imu(uint64_t timestamp_us,
//                             const Eigen::Vector3f &accel,
//                             const Eigen::Vector3f &gyro,
//                             const Eigen::Vector3f &mag) {
//
//  mavlink::qrotor::msg::ONBOARD_IMU imu = {};
//  imu.time_usec = timestamp_us;
//  imu.xacc = accel(0);
//  imu.yacc = accel(1);
//  imu.zacc = accel(2);
//  imu.xgyro = gyro(0);
//  imu.ygyro = gyro(1);
//  imu.zgyro = gyro(2);
//  imu.xmag = mag(0);
//  imu.ymag = mag(1);
//  imu.zmag = mag(2);
//  client->send_message(imu);
//}
//
//void QrotorMavlink::send_attitude_quaternion(
//    uint64_t timestamp_us, const Eigen::Quaternionf &q,
//    const Eigen::Vector3f &angular_velocity) {
//
//  mavlink_msg::ATTITUDE_QUATERNION att = {};
//  att.time_boot_ms = timestamp_us;
//  att.q1 = q.w();
//  att.q2 = q.x();
//  att.q3 = q.y();
//  att.q4 = q.z();
//  att.rollspeed = angular_velocity(0);
//  att.pitchspeed = angular_velocity(1);
//  att.yawspeed = angular_velocity(2);
//  client->send_message(att);
//}
//
//void QrotorMavlink::send_battery_status(const uint16_t voltage,
//                                        const int16_t current) {
//  mavlink::qrotor::msg::POWER_READINGS batt = {};
//  // batt.voltages[0] = (voltage);
//  // batt.current_battery = current;
//  client->send_message(batt);
//}
//
//void QrotorMavlink::send_offboard_control(const uint8_t mode, const float x,
//                                          const float y, const float z,
//                                          const float thrust, const float yaw) {
//  mavlink_msg::OFFBOARD_CONTROL offb_ctrl = {};
//  offb_ctrl.mode = mode;
//  offb_ctrl.x = x;
//  offb_ctrl.y = y;
//  offb_ctrl.z = z;
//  offb_ctrl.thrust = thrust;
//  offb_ctrl.yaw = yaw;
//  client->send_message(offb_ctrl);
//}
//
//void QrotorMavlink::send_odometry(
//    const uint64_t time_us, const uint8_t frame_id,
//    const uint8_t child_frame_id, const Eigen::Vector3f &position,
//    const Eigen::Quaternionf &quat, const Eigen::Vector3f &velocity,
//    const Eigen::Vector3f &ang_vel, const uint8_t estimator_type) {
//  mavlink_msg::ODOMETRY odom = {};
//  odom.time_usec = time_us;
//  odom.frame_id = frame_id;
//  odom.child_frame_id = child_frame_id;
//  odom.x = position(0);
//  odom.y = position(1);
//  odom.z = position(2);
//  odom.q[0] = quat.w();
//  odom.q[1] = quat.x();
//  odom.q[2] = quat.y();
//  odom.q[3] = quat.z();
//  odom.vx = velocity(0);
//  odom.vy = velocity(1);
//  odom.vz = velocity(2);
//  odom.rollspeed = ang_vel(0);
//  odom.pitchspeed = ang_vel(1);
//  odom.yawspeed = ang_vel(2);
//  odom.estimator_type = estimator_type;
//  client->send_message(odom);
//}
//
//mavlink_msg::HEARTBEAT
//QrotorMavlink::deserialize_heartbeat(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::HEARTBEAT s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}
//
//mavlink_msg::ATTITUDE_QUATERNION
//QrotorMavlink::deserialize_attitude_quaternion(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::ATTITUDE_QUATERNION s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}
//
//mavlink_msg::ONBOARD_IMU QrotorMavlink::deserialize_imu(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::ONBOARD_IMU s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}
//
//mavlink_msg::POWER_READINGS
//QrotorMavlink::deserialize_battery_status(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::POWER_READINGS s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}
//
//mavlink_msg::OFFBOARD_CONTROL
//QrotorMavlink::deserialize_offboard_control(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::OFFBOARD_CONTROL s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}
//
//mavlink_msg::ODOMETRY
//QrotorMavlink::deserialize_odometry(mavlink_message_t msg) {
//  mavlink::MsgMap map(msg);
//  mavlink_msg::ODOMETRY s;
//  s.deserialize(map);
//  // std::cout << s.to_yaml() << std::endl;
//  return s;
//}

} // namespace qrotor_mavlink
