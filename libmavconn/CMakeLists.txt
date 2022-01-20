cmake_minimum_required(VERSION 2.8.3)
project(libmavconn)

find_package(catkin REQUIRED)
find_package(console_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
include(EnableCXX11)
include(MavrosMavlink)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include
  ${Boost_INCLUDE_DIRS}
  ${mavlink_INCLUDE_DIRS}
  ${console_bridge_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mavconn
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp
  src/interface.cpp
  src/serial.cpp
  src/tcp.cpp
  src/udp.cpp
)
target_link_libraries(mavconn
  ${Boost_LIBRARIES}
  ${console_bridge_LIBRARIES}
)
target_include_directories(mavconn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Use catkin-supplied em_expand macros to generate source files
em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/mavlink_dialect.h.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h)

em_expand(${CMAKE_CURRENT_SOURCE_DIR}/mavlink.context.py.in
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/mavlink.context.py
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mavlink_helpers.cpp.em
  ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/src/mavlink_helpers.cpp)

# message(STATUS "Copy mavlink_dialect.h to source tree")
# file(
#   COPY ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/include/mavconn/mavlink_dialect.h
#   DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/include/mavconn/
#   )
