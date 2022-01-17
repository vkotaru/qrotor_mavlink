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
  printf("Got message %u, len: %u, framing: %d\n", message->msgid, message->len,
         int(framing));
  // message_id = message->msgid;
  cond.notify_one();
}

bool QrotorMavlink::wait_one() {
  std::unique_lock<std::mutex> lock(mutex);
  return cond.wait_for(lock, std::chrono::seconds(2)) ==
         std::cv_status::no_timeout;
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
  mavlink::minimal::msg::HEARTBEAT hb = {};
  hb.type = int(mavlink::minimal::MAV_TYPE::QUADROTOR);
  hb.autopilot = int(mavlink::minimal::MAV_AUTOPILOT::GENERIC);
  hb.base_mode = base_mode;
  hb.custom_mode = custom_mode;
  hb.system_status = system_status;
  client->send_message(hb);
}

void QrotorMavlink::send_imu(uint8_t system_id, uint64_t timestamp_us,
                             const Eigen::Vector3f &accel,
                             const Eigen::Vector3f &gyro) {

  mavlink::common::msg::SCALED_IMU imu = {};
  imu.time_boot_ms = timestamp_us;
  imu.xacc = accel(0);
  imu.yacc = accel(1);
  imu.zacc = accel(2);
  imu.xgyro = gyro(0);
  imu.ygyro = gyro(1);
  imu.zgyro = gyro(2);
  client->send_message(imu);
}

} // namespace qrotor_mavlink
