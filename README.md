# duatic_ros2control

[![Jazzy Build Main](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-jazzy.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-jazzy.yml)  [![Kilted Build Main](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-kilted.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-kilted.yml)  [![Rolling Build Main](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-rolling.yml/badge.svg?branch=main)](https://github.com/Duatic/duatic_ros2control/actions/workflows/build-rolling.yml)

This repository contains implementations that helps to integrate any Duatic DuaDrive setup into [ros2_control](https://control.ros.org/).

# License

The contents are licensed under the BSD-3-Clause  [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository. Please open an issue in order to get in touch with us.

# Dependencies

All dependencies with their corresponding version are listed in the [repos.list](./repos.list).

| Name | Description | License
| ---  | --- | --- |
| [ethercat_sdk_master](https://github.com/Duatic/ethercat_sdk_master) | Object oriented wrapper around the soem_interface | BSD-3-Clause |
| [rsl_drive_sdk](https://github.com/leggedrobotics/rsl_drive_sdk) | Basic drive sdk for the DynaDrives | BSD-3-Clause |
| [soem_interface](https://github.com/Duatic/soem_interface) | Ethercat wrapper library around SOME | GPL v3 |
| [message_logger](https://github.com/leggedrobotics/message_logger) | Logging library which allows logging with and without ROS | BSD-3-Clause |

# Usage

For more detailed information please refer to the DynaArm [documentation](https://docs.duatic.com)

# Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)
