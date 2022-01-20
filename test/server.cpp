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
    sleep(0.1);
  }
  return 0;
}
