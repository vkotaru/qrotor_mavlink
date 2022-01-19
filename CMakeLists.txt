cmake_minimum_required(VERSION 2.8.3)
project(qrotor_mavlink)

set(PYTHON_EXECUTABLE /usr/bin/python3)
add_subdirectory(libmavconn)

include_directories(include)

add_library(qrotor_mavlink src/qrotor_mavlink.cpp)
target_link_libraries(qrotor_mavlink mavconn pthread)
target_include_directories(qrotor_mavlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(client test/client.cpp)
target_link_libraries(client qrotor_mavlink pthread)

add_executable(server test/server.cpp)
target_link_libraries(server qrotor_mavlink pthread)

