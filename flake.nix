{
  description = "nano_ros development environment (from the shared UWRT Nix setup)";

  # TODO: switch to the team's repo when uwrt_nix moves there
  inputs.uwrt.url = "github:atk422/uwrt_nix";

  outputs =
    { uwrt, ... }:
    {
      devShells = uwrt.lib.forAllSystems (
        pkgs:
        let
          system = pkgs.stdenv.hostPlatform.system;
        in
        {
          # Firmware tools for libnanoros and the codegen (Rust), plus ROS 2 with the riptide messages
          # and colcon for the agent
          default = pkgs.mkShell {
            inputsFrom = [ uwrt.devShells.${system}.firmware ];

            # Not inputsFrom uwrt's `ros` shell: its buildEnv leaves out ament_cmake_core, which
            # ament_cmake needs at configure time, so `colcon build` fails in find_package(). List
            # it explicitly, like uwrt's `autonomy` shell does.
            packages = [
              (pkgs.rosPackages.humble.buildEnv {
                paths = with pkgs.rosPackages.humble; [
                  ros-core
                  ament-cmake-core
                  pkgs.uwrt.riptide-msgs
                ];
              })
              pkgs.colcon
              # gs_usb CAN adapter support in the agent
              pkgs.libusb1
              pkgs.pkg-config
            ];

            # inputsFrom brings the packages but not the environment variables
            inherit (uwrt.devShells.${system}.firmware)
              PICO_SDK_PATH
              PICO_TOOLCHAIN_PATH
              CMAKE_POLICY_VERSION_MINIMUM
              MICRO_ROS_LIB_DIR
              ;
          };
        }
      );
    };
}
