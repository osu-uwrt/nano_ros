cmake_minimum_required(VERSION 3.8)
project(nano_ros_agent)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../common
    ${CMAKE_CURRENT_BINARY_DIR}/common
)

set(CANMORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../external/canmore)
set(NANO_ROS_SOCKETCAN OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(NANO_ROS_SOCKETCAN ON)
  add_subdirectory(${CANMORE_DIR} canmore)
else()
  # canmore_cpp is Linux only (SocketCAN), and PICO_ON_DEVICE is titan_canmore's switch for leaving it out
  set(PICO_ON_DEVICE ON)
  add_subdirectory(${CANMORE_DIR} canmore)
  unset(PICO_ON_DEVICE)
endif()

# gs_usb adapters through libusb: CAN on hosts without SocketCAN (macOS, WSL)
option(NANO_ROS_GS_USB "Build gs_usb (candleLight) CAN adapter support with libusb" ON)
if(NANO_ROS_GS_USB)
  find_package(PkgConfig)
  if(PkgConfig_FOUND)
    pkg_check_modules(LIBUSB IMPORTED_TARGET libusb-1.0)
  endif()
  if(NOT LIBUSB_FOUND)
    if(NANO_ROS_SOCKETCAN)
      message(WARNING "libusb-1.0 not found, building without gs_usb support (SocketCAN only)")
      set(NANO_ROS_GS_USB OFF)
    else()
      message(FATAL_ERROR "libusb-1.0 not found: gs_usb is the only CAN backend on ${CMAKE_SYSTEM_NAME}")
    endif()
  endif()
endif()

add_executable(nano_ros_agent 
    src/server.cpp
    src/agent.cpp
    src/client_handler.cpp
    src/transport/canmore.cpp
    src/transport/canmore_link.cpp
    src/transport/serial.cpp
)

if(NANO_ROS_SOCKETCAN)
  target_sources(nano_ros_agent PRIVATE src/transport/canmore_socketcan.cpp)
  target_compile_definitions(nano_ros_agent PRIVATE NROS_HAVE_SOCKETCAN)
  target_link_libraries(nano_ros_agent PRIVATE canmore_cpp)
endif()

if(NANO_ROS_GS_USB)
  target_sources(nano_ros_agent PRIVATE
    src/transport/gs_usb.cpp
    src/transport/canmore_gs_usb.cpp
  )
  target_compile_definitions(nano_ros_agent PRIVATE NROS_HAVE_GS_USB)
  find_package(Threads REQUIRED)
  target_link_libraries(nano_ros_agent PRIVATE PkgConfig::LIBUSB Threads::Threads)
endif()

# ament_target_dependencies(nano_ros_agent rclcpp)

target_link_libraries(nano_ros_agent PRIVATE
    nano_ros_common
    rclcpp::rclcpp
    canmore
)


target_include_directories(nano_ros_agent PUBLIC 
    #${CMAKE_CURRENT_SOURCE_DIR}/../common/include
    include/nano_ros_agent/transport
    include/nano_ros_agent
    # canmore_cpp's portable headers (PollFD.hpp, span_compat.hpp), used by the gs_usb backend everywhere
    ${CANMORE_DIR}/canmore_cpp/include
)


ament_package()
