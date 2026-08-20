project(libnanoros)
cmake_minimum_required(VERSION 3.13)
add_library(libnanoros INTERFACE)

set(UCDR_SUPERBUILD OFF CACHE BOOL "" FORCE)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../external/micro_cdr/Micro-CDR-2.0.2
                  ${CMAKE_CURRENT_BINARY_DIR}/microcdr)

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_BINARY_DIR}/common
)

target_sources(libnanoros INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/nano_ros.c
)

target_link_libraries(libnanoros INTERFACE
    nano_ros_common
    microcdr
)

target_include_directories(libnanoros INTERFACE
    #${CMAKE_CURRENT_LIST_DIR}/../common/include
    ${CMAKE_CURRENT_LIST_DIR}/include
)
