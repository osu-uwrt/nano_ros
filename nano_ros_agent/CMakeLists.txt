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


add_executable(nano_ros_agent 
    src/server.cpp
    src/client_handler.cpp
    src/transport/can.cpp
    src/transport/serial.cpp
)

# ament_target_dependencies(nano_ros_agent rclcpp)

target_link_libraries(nano_ros_agent PRIVATE
    nano_ros_common
    rclcpp::rclcpp
)


target_include_directories(nano_ros_agent PUBLIC 
    #${CMAKE_CURRENT_SOURCE_DIR}/../common/include
    include/nano_ros_agent/transport
    include/nano_ros_agent
)


ament_package()
