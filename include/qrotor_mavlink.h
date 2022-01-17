#ifndef QROTOR_MAVLINK_H
#define QROTOR_MAVLINK_H

#include <chrono>
#include <condition_variable>

#include <mavconn/interface.h>
#include <mavconn/serial.h>
#include <mavconn/tcp.h>
#include <mavconn/udp.h>

namespace qrotor_mavlink {

class QrotorMavlink {
public:
  QrotorMavlink();
  ~QrotorMavlink();
};

} // namespace qrotor_mavlink

#endif