#include "qrotor_mavlink.h"

int main() {

  qrotor_mavlink::QrotorMavlink client(1, 200, "192.168.4.42", 45003,
                                       "192.168.4.42", 45002);

  if (client.init()) {
    std::cout << "client initialized\n";
  }
  client.send_heartbeat(0, 0, 0);

  return 0;
}
