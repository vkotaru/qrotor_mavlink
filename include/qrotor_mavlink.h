#ifndef QROTOR_MAVLINK_H
#define QROTOR_MAVLINK_H

#include <chrono>
#include <condition_variable>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

#include <eigen3/Eigen/Dense>

namespace qrotor_mavlink {
using namespace mavconn;
using namespace mavlink;

class QrotorMavlink {
private:
  std::mutex mutex;
  std::condition_variable cond;

  uint8_t system_id = 1;
  uint8_t component_id = 250;

  mavlink_message_t msg;
  mavlink_status_t status;
  bool initialized = false;

  mavconn::MAVConnInterface::Ptr server, client;

  void recv_message(const mavlink_message_t *message, const Framing framing);
  bool wait_one();

public:
  QrotorMavlink();
  QrotorMavlink(uint8_t system_id, uint8_t component_id, std::string bind_host,
                unsigned short bind_port, std::string remote_host,
                unsigned short remote_port);

  ~QrotorMavlink();

  bool init();

  /* mavlink messages */
  void send_heartbeat(const uint8_t base_mode, const uint32_t custom_mode,
                      const uint8_t system_status);
  void send_imu(uint8_t system_id, uint64_t timestamp_us,
                const Eigen::Vector3f &accel, const Eigen::Vector3f &gyro);
};

} // namespace qrotor_mavlink

#endif