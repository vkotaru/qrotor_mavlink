#ifndef QROTOR_MAVLINK_H
#define QROTOR_MAVLINK_H

#include <chrono>
#include <condition_variable>
#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>
#include <mutex>
#include <queue>

#include <eigen3/Eigen/Dense>

namespace qrotor_mavlink {
using namespace mavconn;
using namespace mavlink;
namespace mavlink_msg {
using namespace mavlink::minimal::msg;
using namespace mavlink::common::msg;
using namespace mavlink::qrotor::msg;
} // namespace mavlink_msg

class QrotorMavlink {
private:
  std::mutex mutex; // TODO figure this out
  std::mutex message_buffer_mutex_;
  std::condition_variable cond;

  uint8_t system_id = 1;
  uint8_t component_id = 250;

  mavlink_status_t status;
  bool initialized = false;
  bool udp_client_created = false;

  mavconn::MAVConnInterface::Ptr server, client;

  void recv_message(const mavlink_message_t *message, const Framing framing);
  bool wait_one();
  std::queue<mavlink_message_t> msg_buffer_queue_;

public:
  QrotorMavlink();
  QrotorMavlink(uint8_t system_id, uint8_t component_id, std::string bind_host,
                unsigned short bind_port, std::string remote_host,
                unsigned short remote_port);

  ~QrotorMavlink();

  bool init();
  void decode();

  void
  copy_and_flush_buffer_queue(std::queue<mavlink_message_t> &copy_to_buffer) {
    message_buffer_mutex_.lock();
    copy_to_buffer = msg_buffer_queue_;
    msg_buffer_queue_ = {};
    message_buffer_mutex_.unlock();
  }

  template <typename T> void send_message(T msg) { client->send_message(msg); }
  template <typename T> T deserialize_message(mavlink_message_t msg) {
    mavlink::MsgMap map(msg);
    T s;
    s.deserialize(map);
    // std::cout << s.to_yaml() << std::endl;
    return s;
  }

  /* send mavlink messages */
  void send_heartbeat(const uint8_t base_mode, const uint32_t custom_mode,
                      const uint8_t system_status);
  void send_imu(uint64_t timestamp_us, const Eigen::Vector3f &accel,
                const Eigen::Vector3f &gyro, const Eigen::Vector3f &mag);
  void send_attitude_quaternion(uint64_t timestamp_us,
                                const Eigen::Quaternionf &q,
                                const Eigen::Vector3f &angular_velocity);
  void send_battery_status(const uint16_t voltage, const int16_t current);
  void send_offboard_control(const uint8_t mode, const float x, const float y,
                             const float z, const float thrust,
                             const float yaw);
  void
  send_odometry(const uint64_t time_us, const uint8_t frame_id,
                const uint8_t child_frame_id, const Eigen::Vector3f &position,
                const Eigen::Quaternionf &quat, const Eigen::Vector3f &velocity,
                const Eigen::Vector3f &ang_vel, const uint8_t estimator_type);

  /* handle mavlink messages */
  mavlink_msg::HEARTBEAT deserialize_heartbeat(mavlink_message_t msg);
  mavlink_msg::ATTITUDE_QUATERNION
  deserialize_attitude_quaternion(mavlink_message_t msg);
  mavlink_msg::SCALED_IMU deserialize_imu(mavlink_message_t msg);
  mavlink_msg::BATTERY_STATUS deserialize_battery_status(mavlink_message_t msg);
  mavlink_msg::OFFBOARD_CONTROL
  deserialize_offboard_control(mavlink_message_t msg);
  mavlink_msg::ODOMETRY deserialize_odometry(mavlink_message_t msg);
};

} // namespace qrotor_mavlink

#endif