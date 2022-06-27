cmake_minimum_required(VERSION 2.8.3)
project(qrotor_mavlink)

set(PYTHON_EXECUTABLE /usr/bin/python3)


if (QROTOR_MAVLINK_BUILD_STATIC_LIBS)
    set(qrotor-mavlink-type STATIC)
else ()
    set(qrotor-mavlink-type SHARED)
endif ()


include_directories(include
        include/mavlink/v2.0/qrotor)

add_library(qrotor_mavlink ${qrotor-mavlink-type} src/qrotor_mavlink.cpp)
target_link_libraries(qrotor_mavlink pthread)
target_include_directories(qrotor_mavlink PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(client test/client.cpp)
target_link_libraries(client qrotor_mavlink pthread)

add_executable(server test/server.cpp)
target_link_libraries(server qrotor_mavlink pthread)

