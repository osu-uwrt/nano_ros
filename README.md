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

`qos` is optional and defaults to `"default"`. It picks the QoS profile the agent uses for the topic's ROS publisher/subscription:

| `qos` | profile |
|---|---|
| `default` | rclcpp default: reliable, volatile, keep last 10 |
| `sensor_data` | `rclcpp::SensorDataQoS`: best effort, volatile, keep last 5 |
| `reliable` | reliable, volatile, keep last 10 |
| `best_effort` | best effort, volatile, keep last 10 |
| `transient_local` | reliable, transient local (latched), keep last 1 |
| `system_default` | `rclcpp::SystemDefaultsQoS` |

the agent rejects a registration with an unknown QoS value by ACKing it with a nonzero status (see `nros_ack_status_t` in `common/include/protocol.h`).

nano_ros_codegen reads this file and supplies the user with the code they'll need to publish/subscribe and gives the firmware library code an array of structures to use.

### libnanoros

link libnanoros into your firmware and point `nano_ros_generate()` at your ros.toml (see [building libnanoros](#libnanoros-1)). the codegen runs as part of your build, so there's no separate step

then #include both "nano_ros.h" and "nano_ros_codegen.h". for each topic you get a C struct for its message type, plus `nros_publish_<name>()` for publishers and `nros_set_<name>_subscriber_cb()` for subscribers

#### board status
call `nros_enable_status()` once and the board reports its health about once a second while connected. the agent publishes it on `/diagnostics`, so it shows up in `rqt_robot_monitor` and `ros2 topic echo /diagnostics`

```c
static uint32_t millis(void) { return to_ms_since_boot(get_absolute_time()); }

static void fill_status(nros_status_t *s) {
    s->version = FULL_BUILD_TAG;                  // anything that identifies the build
    s->reset_reason = watchdog_caused_reboot() ? NROS_RESET_WATCHDOG : NROS_RESET_POWER_ON;
    s->faults = *fault_list_reg;                  // board-defined bitmask, nonzero shows as an error
}

nros_enable_status(millis, fill_status); // fill can be NULL
```

each report has the board name (from ros.toml), uptime, version, reset reason, faults and how many `nros_publish` calls failed. it shows as an error when faults are set, and a warning when the last reset was a watchdog or brownout. if a board that was reporting goes quiet for 3 s, the agent marks it stale and keeps saying how long it's been

### codegen

you only need to run codegen by hand to look at its output. it accepts a path to a ros.toml file, the ros installation prefix, and a directory of custom .msg packages

for example,
```bash
$ ./nano_ros_codegen <path to ros.toml> --user-msgs <path to user msgs, i.e. riptide_core> --ros-prefix <path to ros prefix, i.e. /opt/ros/humble>
```

`--ros-prefix` defaults to the first entry of `AMENT_PREFIX_PATH`. generated files go in `generated/` next to the ros.toml unless you pass `--out-dir`. `--depfile <path>` also writes a Makefile-style list of every file codegen read, which is what the CMake integration uses

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

add libnanoros as a subdir, link it into your target, and call `nano_ros_generate()` on the same target. For example
```cmake
add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/<path to libnanoros root>
    ${CMAKE_CURRENT_BINARY_DIR}/libnanoros # only needed if outside current cmake tree
)

target_link_libraries(<target name> PUBLIC
    libnanoros
)

nano_ros_generate(<target name> ros.toml
    USER_MSGS <path to user msgs, i.e. riptide_core> # optional
    ROS_PREFIX /opt/ros/humble                        # optional, defaults to AMENT_PREFIX_PATH
)
```

this builds nano_ros_codegen from this repo with cargo (so it always matches libnanoros), runs it into the build directory, and compiles the generated code into the target. it reruns when ros.toml or any .msg file it uses changes. relative paths are relative to the CMakeLists.txt calling it

cargo needs to be on your PATH (it is in the nix dev shell). to use a prebuilt codegen instead, configure with `-DNANO_ROS_CODEGEN_EXECUTABLE=<path>`

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



