#include "qrotor_mavlink.h"

int main() {

  qrotor_mavlink::QrotorMavlink sender_(1, 200, "127.0.0.1", 45003,
                                        "127.0.0.1", 45002);

  if (sender_.init()) {
    std::cout << "sender_ initialized\n";
  }
  int i = 0;

  mavlink_message_t msg;
  uint16_t len;

  uint8_t buf[BUFFER_LENGTH];

  while (true) {

    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, 0, 0,
                               MAV_STATE_ACTIVE);
//    len = mavlink_msg_to_send_buffer(buf, &msg);
    sender_.send_message(msg);
    std::cout << "sending message " << std::endl;

    sleep(1);
  }

  return 0;
}
//
//#include "cpp_sockets.h"
//
//int main() {
//
//  u_short socket_port{9090};
//  std::string destination_ip{"127.0.0.1"};
//
//  UDPClient client(socket_port, destination_ip);
//
//  for(int i = 0; i < 100; i++) {
//    std::string message = "Hello " + std::to_string(8475768437684347684);
//    auto send_status = client.send_message(message);
//    std::cout << "Sent message: " << message << " send status " << send_status << std::endl;
//    sleep(1);
//  }
//}
