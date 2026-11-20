# nano_ros_generate(<target> <ros.toml> [ROS_PREFIX <dir>] [USER_MSGS <dir>])
#
# Runs nano_ros_codegen on <ros.toml> at build time and compiles the generated code into <target>,
# which must also link libnanoros. Codegen reruns when ros.toml or any .msg file it read changes.
#
#   ROS_PREFIX  ROS install to resolve standard messages from. Defaults to the first entry of
#               AMENT_PREFIX_PATH at configure time.
#   USER_MSGS   Directory searched for custom message packages (e.g. riptide_msgs2).
#
# Relative paths are relative to the calling CMakeLists.txt. The codegen is built from this repo
# with cargo, so it always matches libnanoros; set NANO_ROS_CODEGEN_EXECUTABLE to use a prebuilt one.

set(NANO_ROS_CODEGEN_DIR "${CMAKE_CURRENT_LIST_DIR}/../../nano_ros_codegen" CACHE INTERNAL "")
set(NANO_ROS_CODEGEN_EXECUTABLE "" CACHE FILEPATH
    "Prebuilt nano_ros_codegen to use instead of building it from source with cargo")

# Defines the nano_ros_codegen_build target the first time it's needed, so projects that never
# call nano_ros_generate() don't need cargo
function(_nano_ros_codegen_target out_exe)
    if(NANO_ROS_CODEGEN_EXECUTABLE)
        set(${out_exe} "${NANO_ROS_CODEGEN_EXECUTABLE}" PARENT_SCOPE)
        return()
    endif()

    set(target_dir "${CMAKE_BINARY_DIR}/nano_ros_codegen")
    set(exe "${target_dir}/release/nano_ros_codegen${CMAKE_HOST_EXECUTABLE_SUFFIX}")

    if(NOT TARGET nano_ros_codegen_build)
        find_program(NANO_ROS_CARGO cargo)
        if(NOT NANO_ROS_CARGO)
            message(FATAL_ERROR "nano_ros_generate: cargo not found. Install Rust (or use the repo's "
                    "nix dev shell), or set NANO_ROS_CODEGEN_EXECUTABLE to a prebuilt nano_ros_codegen.")
        endif()

        # Always runs; cargo decides whether anything needs rebuilding
        add_custom_target(nano_ros_codegen_build
            COMMAND ${NANO_ROS_CARGO} build --release --quiet
                    --manifest-path "${NANO_ROS_CODEGEN_DIR}/Cargo.toml"
                    --target-dir "${target_dir}"
            BYPRODUCTS "${exe}"
            COMMENT "Building nano_ros_codegen"
            VERBATIM
        )
    endif()

    set(${out_exe} "${exe}" PARENT_SCOPE)
endfunction()

function(nano_ros_generate target toml)
    cmake_parse_arguments(PARSE_ARGV 2 arg "" "ROS_PREFIX;USER_MSGS" "")
    if(arg_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR "nano_ros_generate: unknown arguments: ${arg_UNPARSED_ARGUMENTS}")
    endif()
    if(NOT TARGET ${target})
        message(FATAL_ERROR "nano_ros_generate: '${target}' is not a target. Call it after "
                "add_executable() / add_library().")
    endif()

    get_filename_component(toml "${toml}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT EXISTS "${toml}")
        message(FATAL_ERROR "nano_ros_generate: ${toml} does not exist")
    endif()

    set(codegen_args "${toml}")

    if(NOT arg_ROS_PREFIX AND DEFINED ENV{AMENT_PREFIX_PATH})
        string(REPLACE ":" ";" ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
        list(GET ament_prefixes 0 arg_ROS_PREFIX)
    endif()
    if(arg_ROS_PREFIX)
        get_filename_component(ros_prefix "${arg_ROS_PREFIX}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        list(APPEND codegen_args --ros-prefix "${ros_prefix}")
    else()
        message(WARNING "nano_ros_generate: no ROS_PREFIX given and AMENT_PREFIX_PATH is not set; "
                "codegen will fall back to /opt/ros/humble")
    endif()

    if(arg_USER_MSGS)
        get_filename_component(user_msgs "${arg_USER_MSGS}" ABSOLUTE BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
        list(APPEND codegen_args --user-msgs "${user_msgs}")
    endif()

    set(out_dir "${CMAKE_CURRENT_BINARY_DIR}/nano_ros_generated/${target}")
    set(outputs
        "${out_dir}/nano_ros_codegen.h"
        "${out_dir}/nano_ros_serde_codegen.c"
        "${out_dir}/nano_ros_impl_codegen.c"
    )
    list(APPEND codegen_args --out-dir "${out_dir}")

    # The depfile lists ros.toml and every .msg file read, so editing a message regenerates.
    # Makefile generators only support DEPFILE from CMake 3.20; without it only ros.toml is tracked.
    set(depfile_args "")
    if(CMAKE_GENERATOR MATCHES "Ninja" OR NOT CMAKE_VERSION VERSION_LESS 3.20)
        set(depfile "${out_dir}/nano_ros_codegen.d")
        list(APPEND codegen_args --depfile "${depfile}")
        set(depfile_args DEPFILE "${depfile}")
    endif()

    _nano_ros_codegen_target(codegen_exe)
    set(codegen_deps "${codegen_exe}")
    if(TARGET nano_ros_codegen_build)
        list(APPEND codegen_deps nano_ros_codegen_build)
    endif()

    add_custom_command(
        OUTPUT ${outputs}
        COMMAND "${codegen_exe}" ${codegen_args}
        DEPENDS "${toml}" ${codegen_deps}
        ${depfile_args}
        COMMENT "Generating nano_ros code for ${target} from ${toml}"
        VERBATIM
    )

    target_sources(${target} PRIVATE ${outputs})
    target_include_directories(${target} PUBLIC "${out_dir}")
endfunction()
