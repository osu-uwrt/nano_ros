# nano ros

a lightweight firmware library, codegen tool, and host side agent

## usage

### ros.toml
ros.toml is the file where topics are defined that will be used on a board

an example of a ros.toml file is as follows. (naming conventions will change)
```toml
[board]
name = "foo_board"
transport = "can"

[[topics]]
name = "imu_data"
direction = "publisher"
ros_type = "sensor_msgs/Imu"
qos = "default"

[[topics]]
name = "bit_sub"
direction = "subscriber"
ros_type = "std_msgs/UInt8"
qos = "default"

[[topics]]
name = "firmware_status"
direction = "publisher"
ros_type = "riptide_msgs/FirmwareStatus"
qos = "default"
```

nano_ros_codegen reads this file and supplies the user with the code they'll need to publish/subscribe and gives the firmware library code an array of structures to use.

this should be ran first. 

### codegen

codegen will accept a path to a ros.toml file, a path to the users ros installation prefix, and a path to a users custom .msg types

for example,
```bash 
$ ./nano_ros_codegen <path to ros.toml> --user-msgs <path to user msgs, i.e. riptide_core> --ros-prefix <path to ros prefix, i.e. /opt/ros/humble>
```

as of now (will change), generated files get spit out next to the ros.toml file supplied

### libnanoros

once code has been generated, you can use it in your project with libnanoros

note: you should #include both "nano_ros.h" and "nano_ros_codegen.h"

### nano_ros_agent

spawn the agent on a port and go brr

## building

### nano_ros_codegen
ensure rustc / cargo is installed on your machine and navigate to nano_ros_codegen

```bash
$ cd nano_ros_codegen
```
compile
```bash
$ cargo build
```
add the --release flag for a prod build

the binary resides in target/debug/nano_ros_codegen if you didn't compile with the --release flag

### libnanoros 
note: for the time being, this is meant to be linked into a project inside of titan firmware via cmake

add libnanoros as a subdir and link into project. For example
```bash
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/<path to libnanoros root>
    ${CMAKE_CURRENT_BINARY_DIR}/libnanoros # only needed if outside current cmake tree
)

target_link_libraries(<target name> PUBLIC
    libnanoros
)
```
if the generated code is not being compiled in, evil linky errors ensue

### nano_ros_agent

ensure you got your ros business in order and navigate to nano_ros_agent
```bash
$ cd nano_ros_agent
```
compile
```bash
colcon build
```

for now, you're on your own soldier



