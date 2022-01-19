#ifndef QROTOR_MAVLINK_H
#define QROTOR_MAVLINK_H

#include <chrono>
#include <condition_variable>
#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>
#include <queue>

#include <eigen3/Eigen/Dense>

namespace qrotor_mavlink {
using namespace mavconn;
using namespace mavlink;
namespace mavlink_msg {
using namespace mavlink::minimal::msg;
using namespace mavlink::common::msg;
} // namespace msg

class QrotorMavlink {
private:
  std::mutex mutex;
  std::condition_variable cond;

  uint8_t system_id = 1;
  uint8_t component_id = 250;

  mavlink_status_t status;
  bool initialized = false;

  mavconn::MAVConnInterface::Ptr server, client;

  void recv_message(const mavlink_message_t *message, const Framing framing);
  bool wait_one();

  std::queue<mavlink_message_t> msg_buffer_queue;

public:
  QrotorMavlink();
  QrotorMavlink(uint8_t system_id, uint8_t component_id, std::string bind_host,
                unsigned short bind_port, std::string remote_host,
                unsigned short remote_port);

  ~QrotorMavlink();

  bool init();
  void decode();

  /* send mavlink messages */
  void send_heartbeat(const uint8_t base_mode, const uint32_t custom_mode,
                      const uint8_t system_status);
  void send_imu(uint64_t timestamp_us, const Eigen::Vector3f &accel,
                const Eigen::Vector3f &gyro, const Eigen::Vector3f &mag);
  void send_attitude_quaternion(uint64_t timestamp_us,
                                const Eigen::Quaternionf &q,
                                const Eigen::Vector3f &angular_velocity);
  void send_battery_status(const uint16_t voltage, const int16_t current);

  /* handle mavlink messages */
  mavlink_msg::HEARTBEAT deserialize_heartbeat(mavlink_message_t msg);
  mavlink_msg::ATTITUDE_QUATERNION
  deserialize_attitude_quaternion(mavlink_message_t msg);
  mavlink_msg::SCALED_IMU deserialize_imu(mavlink_message_t msg);
  mavlink_msg::BATTERY_STATUS deserialize_battery_status(mavlink_message_t msg);
};

} // namespace qrotor_mavlink

#endif