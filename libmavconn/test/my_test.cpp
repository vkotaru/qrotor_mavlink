
#include <chrono>
#include <condition_variable>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

using namespace mavconn;
using mavlink::mavlink_message_t;
using mavlink::msgid_t;

msgid_t message_id;

std::mutex mutex;
std::condition_variable cond;
namespace mavlink {
namespace common {
using namespace mavlink::minimal;
namespace msg {
using namespace mavlink::minimal::msg;
}
} // namespace common
} // namespace mavlink

static void send_heartbeat(MAVConnInterface *ip) {
  using mavlink::common::MAV_AUTOPILOT;
  using mavlink::common::MAV_MODE;
  using mavlink::common::MAV_STATE;
  using mavlink::common::MAV_TYPE;

  mavlink::common::msg::HEARTBEAT hb = {};
  hb.type = int(MAV_TYPE::ONBOARD_CONTROLLER);
  hb.autopilot = int(MAV_AUTOPILOT::INVALID);
  hb.base_mode = int(MAV_MODE::MANUAL_ARMED);
  hb.custom_mode = 0;

  hb.system_status = int(MAV_STATE::ACTIVE);

  ip->send_message(hb);
}

void recv_message(const mavlink_message_t *message, const Framing framing) {
  printf("Got message %u, len: %u, framing: %d\n", message->msgid,
  message->len, int(framing));
  message_id = message->msgid;
  cond.notify_one();
}

bool wait_one() {
  std::unique_lock<std::mutex> lock(mutex);
  return cond.wait_for(lock, std::chrono::seconds(2)) ==
         std::cv_status::no_timeout;
}

int main() {

  MAVConnInterface::Ptr echo, client;

  message_id = std::numeric_limits<msgid_t>::max();
  auto msgid = mavlink::common::msg::HEARTBEAT::MSG_ID;

  // create echo server
  echo = std::make_shared<MAVConnUDP>(42, 200, "192.168.4.42", 45002);
  echo->connect([&](const mavlink_message_t *msg, const Framing framing) {
    echo->send_message(msg);
  });

  // create client
  client = std::make_shared<MAVConnUDP>(44, 200, "192.168.4.42", 45003,
                                        "192.168.4.42", 45002);
  client->connect(
      std::bind(&recv_message, std::placeholders::_1, std::placeholders::_2));

  // wait echo
  send_heartbeat(client.get());
  send_heartbeat(client.get());

  if (wait_one()){
  std::cout << message_id << " " << msgid << std::endl;}
  return 0;
}