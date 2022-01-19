#include "qrotor_mavlink.h"

int main() {

  qrotor_mavlink::QrotorMavlink server(2, 200, "192.168.4.42", 45002,
                                       "192.168.4.42", 45003);

  if (server.init()) {
    std::cout << "server initialized\n";
  }
  while (true) {
    server.decode();
    sleep(0.1);
  }
  return 0;
}
