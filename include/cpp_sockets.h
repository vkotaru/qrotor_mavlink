/**
 * source: https://github.com/computersarecool/cpp_sockets
 */
#include <iostream>
#include <string>
#include <arpa/inet.h> // This contains inet_addr
#include <sys/socket.h>
#include <unistd.h> // This contains close
#include <thread>
#include <mutex>
#include "sys_log.hpp"

namespace qrotor_mavlink {

#define BUFFER_LENGTH 2041

#define INVALID_SOCKET (SOCKET)(~0)
#define SOCKET_ERROR (-1)
typedef int SOCKET;

// These could also be enums
const int socket_bind_err = 3;
const int socket_accept_err = 4;
const int connection_err = 5;
const int message_send_err = 6;
const int receive_err = 7;

class Socket {
public:
  enum class SocketType { TYPE_STREAM = SOCK_STREAM, TYPE_DGRAM = SOCK_DGRAM };

protected:
  explicit Socket(const SocketType socket_type) : m_socket(), m_addr() {

    // Create the socket handle
    m_socket = socket(AF_INET, static_cast<int>(socket_type), 0);
    if (m_socket == INVALID_SOCKET) {
      throw std::runtime_error("Could not create socket");
    }

    m_addr.sin_family = AF_INET;
  }
  ~Socket() = default;
  SOCKET m_socket;
  sockaddr_in m_addr;
  void set_port(u_short port) { m_addr.sin_port = htons(port); }
  int set_address(const std::string &ip_address) {
    return inet_pton(AF_INET, ip_address.c_str(), &m_addr.sin_addr);
  }

private:
};

class UDPClient : public Socket {
public:
  explicit UDPClient(u_short port = 8000, const std::string &ip_address = "127.0.0.1")
      : Socket(SocketType::TYPE_DGRAM) {
    set_address(ip_address);
    set_port(port);
    LOG_INFO("UDP Client created.");
  }
  ssize_t send_message(const std::string &message) {
    size_t message_length = message.length();
    return sendto(m_socket, message.c_str(), message_length, 0,
                  reinterpret_cast<sockaddr *>(&m_addr), sizeof(m_addr));
  }
  ssize_t send_message(uint8_t *buff, uint16_t len) {
    return sendto(m_socket, buff, len, 0,
                  reinterpret_cast<sockaddr *>(&m_addr), sizeof(m_addr));
  }
};

class UDPServer : public Socket {
protected:
  bool run_server{true};
  sockaddr_in client{};
  char client_ip[INET_ADDRSTRLEN]{};
  socklen_t slen = sizeof(client);
  char message_buffer[BUFFER_LENGTH]{};
  std::thread listen_thread_;

public:
  explicit UDPServer(u_short port = 8000, const std::string &ip_address = "0.0.0.0")
      : Socket(SocketType::TYPE_DGRAM) {
    set_port(port);
    set_address(ip_address);
    LOG_INFO("UDP Server created.");
  }
  int socket_bind() {
    if (bind(m_socket, reinterpret_cast<sockaddr *>(&m_addr), sizeof(m_addr)) ==
        SOCKET_ERROR) {
      LOG_ERROR("UDP Bind error.");
      return socket_bind_err;
    }
    LOG_INFO("UDP Socket Bound.");
    return 0;
  }
  ssize_t receive(uint8_t *buffer) {
    return recvfrom(m_socket, buffer, BUFFER_LENGTH, 0,
                    reinterpret_cast<sockaddr *>(&client), &slen);

  }

//  void listen() {
//    LOG_INFO("Waiting for data...");
//
//    while (run_server) {
//      // This is a blocking call
//      memset(message_buffer, 0, BUFFER_LENGTH);
//      ssize_t recv_len =
//          recvfrom(m_socket, message_buffer, sizeof(message_buffer), 0,
//                   reinterpret_cast<sockaddr *>(&client), &slen);
//      if (recv_len == SOCKET_ERROR) {
//        LOG_ERROR("Receive Data error.");
//      }
//      std::cout << "Received packet from "
//                << inet_ntop(AF_INET, &client.sin_addr, client_ip,
//                             INET_ADDRSTRLEN)
//                << ':' << ntohs(client.sin_port) << std::endl;
//      std::cout << message_buffer << std::endl;
//    }
//  }S

  void stop_server() {
    run_server = false;
  }
};

class TCPClient : public Socket {
public:
  explicit TCPClient(u_short port = 8000, const std::string &ip_address = "127.0.0.1")
      : Socket(SocketType::TYPE_STREAM) {
    set_address(ip_address);
    set_port(port);
    LOG_INFO("TCP client created.");
  }
  int make_connection() {
    LOG_INFO("Connecting...");
    if (connect(m_socket, reinterpret_cast<sockaddr *>(&m_addr),
                sizeof(m_addr)) < 0) {
      std::cout << "Connection error" << std::endl;
      return connection_err;
    }
    LOG_INFO("connected!");

    return 0;
  }
  int send_message(const std::string &message) {
    char server_reply[2000];
    size_t length = message.length();

    if (send(m_socket, message.c_str(), length, 0) < 0) {
      LOG_ERROR("Send failed");
      return message_send_err;
    } else {
      LOG_INFO("Data sent");
    }

    if (recv(m_socket, server_reply, 2000, 0) == SOCKET_ERROR) {
      LOG_ERROR("Receive Failed");
      return receive_err;
    } else {
      LOG_INFO(server_reply);
    }

    return 0;
  }
};

class TCPServer : public Socket {
public:
  explicit TCPServer(u_short port, const std::string &ip_address = "0.0.0.0")
      : Socket(SocketType::TYPE_STREAM) {
    set_port(port);
    set_address(ip_address);
    LOG_INFO("TCP Server created.");
  }
  int socket_bind() {
    if (bind(m_socket, reinterpret_cast<sockaddr *>(&m_addr), sizeof(m_addr)) ==
        SOCKET_ERROR) {
      LOG_ERROR("TCP Socket Bind error.");
      return socket_bind_err;
    }

    LOG_INFO("TCP Socket Bound.");
    listen(m_socket, 3);
    LOG_INFO("TCP Socket waiting for incoming connections...");

    socklen_t client_size = sizeof(sockaddr_in);
    sockaddr_in client{};
    SOCKET new_socket;
    char message_buffer[512];

    new_socket =
        accept(m_socket, reinterpret_cast<sockaddr *>(&client), &client_size);
    if (new_socket == INVALID_SOCKET) {
      LOG_ERROR("TCP Socket accept error");
      return socket_accept_err;
    } else {
      std::cout << "Connection accepted from IP address "
                << inet_ntoa(client.sin_addr) << " on port "
                << ntohs(client.sin_port) << std::endl;
      ssize_t recv_len =
          recv(new_socket, message_buffer, sizeof(message_buffer), 0);
      std::cout << "Incoming message is:\n" << message_buffer << std::endl;
      std::cout << "Message length was: " << recv_len << std::endl;
      std::string message = "Your message has been received client\n";
      size_t message_length = message.length();
      send(new_socket, message.c_str(), message_length, 0);
    }
    close(m_socket);
    return 0;
  }
};

}