project(nano_ros_common)
cmake_minimum_required(VERSION 3.13)
add_library(nano_ros_common INTERFACE)

target_sources(nano_ros_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/nros_stream_parser.c
)

target_include_directories(nano_ros_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/include
)