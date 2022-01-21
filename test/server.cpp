#include "qrotor_mavlink.h"

int main() {

  qrotor_mavlink::QrotorMavlink server(2, 200, "192.168.0.199", 45002,
                                       "192.168.0.199", 45003);

  if (server.init()) {
    std::cout << "server initialized\n";
  }
  while (true) {
    server.send_offboard_control(
        static_cast<uint8_t>(qrotor_mavlink::qrotor::OFFBOARD_CONTROL_MODE::
                                 MODE_POSITION_TARGET_YAW),
        0, 1, 1, 0, 0);

    qrotor_mavlink::mavlink_msg::ODOMETRY odom = {};
    odom.time_usec = 0;
    odom.frame_id = 1;
    odom.child_frame_id = 0;
    odom.x = 0;
    odom.y = 1;
    odom.z = 2;
    odom.q[0] = 1;
    odom.q[1] = 0;
    odom.q[2] = 0;
    odom.q[3] = 0;
    odom.vx = 0;
    odom.vy = 0;
    odom.vz = 0;
    odom.rollspeed = 0;
    odom.pitchspeed = 0;
    odom.yawspeed = 0;
    server.send_message<qrotor_mavlink::mavlink_msg::ODOMETRY>(odom);
    sleep(0.1);
  }
  return 0;
}
