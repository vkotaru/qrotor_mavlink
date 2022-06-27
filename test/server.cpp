#include "qrotor_mavlink.h"

int main() {

  qrotor_mavlink::QrotorMavlink receiver_(2, 200, "127.0.0.1", 45002,
                                          "127.0.0.1", 45003);

  if (receiver_.init()) {
    std::cout << "receiver_ initialized\n";
  }
//  while (true) {
////    receiver_.send_message("right back at you");
//    sleep(1);
//  }
  while(1);
  return 0;
}

//#include "cpp_sockets.h"
//
//int main() {
//
//  u_short socket_port{9090};
//  std::string destination_ip{"127.0.0.1"};
//  UDPServer server(socket_port);
//  int bind_status = server.socket_bind();
//
//  // Return if there is an issue binding
//  if (bind_status) {
//    return bind_status;
//  }
//  server.listen();
//}

