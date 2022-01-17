cmake_minimum_required(VERSION 2.8.3)
project(qrotor_mavlink)

set(PYTHON_EXECUTABLE /usr/bin/python3)
add_subdirectory(libmavconn)

include_directories(include)

add_library(qrotor_mavlink src/qrotor_mavlink.cpp)
target_link_libraries(qrotor_mavlink mavconn)

