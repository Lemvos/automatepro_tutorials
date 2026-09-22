# AutomatePro Tutorials

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)

ROS 2 Humble examples in Python and C++ for the AutomatePro sensors and IO controller.
The [AutomatePro documentation](https://docs.lemvos.com/automatepro/system-overview) describes the topics and messages they use.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Examples](#examples)
- [Project Structure](#project-structure)
- [Testing](#testing)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Overview

The repository holds two ROS 2 packages that provide the same nine examples, and a folder of CAN and RS485 examples that run outside ROS.

| Folder | Contents |
|---|---|
| `automatepro_python_tutorials` | Python package with the examples listed under [Examples](#examples) |
| `automatepro_cpp_tutorials` | C++ package with the same examples |
| `misc` | CAN and RS485 examples outside ROS; see [misc/README.md](misc/README.md) |

Six examples print sensor, input, or diagnostic data, and three switch an IO controller output.

## Prerequisites

- An AutomatePro unit with the AutomatePro software installed, which provides ROS 2 Humble packages including `automatepro_interfaces`.
- `automatepro-io-agent.service` running for the IO examples, and `automatepro-core-driver.service` running for the sensor examples.
- `colcon` and `rosdep`: `sudo apt install python3-colcon-common-extensions python3-rosdep`.
  If rosdep was never initialized on the unit, run `sudo rosdep init` and `rosdep update` once.
- A terminal whose `ROS_DOMAIN_ID` matches the AutomatePro services, which read it from `/opt/automatepro/.env`; see [Installation Prompts](https://docs.lemvos.com/automatepro/manual/misc/software-services#installation-prompts).

## Installation

```bash
mkdir -p ~/tutorials_ws/src
cd ~/tutorials_ws/src
git clone https://github.com/Lemvos/automatepro_tutorials.git
cd ~/tutorials_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage

In each new terminal, source the workspace first:

```bash
source ~/tutorials_ws/install/setup.bash
```

Then run an example from either package, for example:

```bash
ros2 run automatepro_python_tutorials imu_node
ros2 run automatepro_cpp_tutorials imu_node
```

Stop an example with Ctrl+C.

### Examples

Both packages provide the same executables.

| Executable | Topic | Behavior |
|---|---|---|
| `imu_node` | `/sensor/imu/data`, `/sensor/imu/magnetic_field` | Prints IMU and magnetic field data, at most once per second; the magnetic field is published only when the IMU driver's `publish.magnetic_field.enabled` is `true`, which the AutomatePro default configuration turns off |
| `gnss_position_node` | `/sensor/gnss/position/fix` | Prints latitude, longitude, and altitude |
| `gnss_heading_node` | `/sensor/gnss/heading/true_heading` | Prints the heading, clockwise from north, and its accuracy |
| `analog_in_node` | `/io/ain` | Prints the 14 analog inputs, at most once per second |
| `digital_in_node` | `/io/din`, `/io/din/request` | Requests the current state, then prints the 10 digital inputs on each change |
| `digital_out_node` | `/io/digital_out` | Steps `DIGITAL_OUT_H_01` through 0%, 50%, 100%, and 50% duty cycle, one step per second |
| `digital_drive_out_node` | `/io/digital_drive_out` | Switches `HALF_BRIDGE_DRIVE_01` between 0% and 100% duty cycle every second |
| `warning_system_out_node` | `/io/warning_system_out` | Switches the buzzer on and off every second |
| `io_controller_diagnostic_node` | `/diagnostic/io_controller_hw` | Prints the IO controller supply voltages and currents, power-good states, drive faults, and board temperature |

> [!WARNING]
> `digital_out_node`, `digital_drive_out_node`, and `warning_system_out_node` switch real outputs.
> Disconnect or secure any load on `DIGITAL_OUT_H_01` or `HALF_BRIDGE_DRIVE_01` before you run them.
> The IO controller keeps the last command it receives, so stop these nodes with Ctrl+C: they switch their output off before they exit, which can take up to 5 seconds.
> A node that is killed with `kill -9` or crashes leaves its output in the last state.

## Project Structure

```text
automatepro_tutorials/
├── automatepro_cpp_tutorials/
│   ├── src/
│   │   ├── diagnostics/
│   │   │   └── io_controller.cpp
│   │   ├── io/
│   │   │   ├── analog_in.cpp
│   │   │   ├── digital_drive_out.cpp
│   │   │   ├── digital_in.cpp
│   │   │   ├── digital_out.cpp
│   │   │   └── warning_system_out.cpp
│   │   └── sensors/
│   │       ├── gnss_heading.cpp
│   │       ├── gnss_position.cpp
│   │       └── imu.cpp
│   ├── CMakeLists.txt
│   ├── LICENSE
│   └── package.xml
├── automatepro_python_tutorials/
│   ├── automatepro_python_tutorials/
│   │   ├── diagnostics/
│   │   │   └── io_controller.py
│   │   ├── io/
│   │   │   ├── analog_in.py
│   │   │   ├── digital_drive_out.py
│   │   │   ├── digital_in.py
│   │   │   ├── digital_out.py
│   │   │   └── warning_system_out.py
│   │   └── sensors/
│   │       ├── gnss_heading.py
│   │       ├── gnss_position.py
│   │       └── imu.py
│   ├── resource/
│   ├── test/
│   ├── LICENSE
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── misc/
└── README.md
```

## Testing

The tests are the ament linters: cppcheck, cpplint, uncrustify, lint_cmake, xmllint, and copyright for the C++ package, and flake8, pep257, and copyright for the Python package.

```bash
cd ~/tutorials_ws
colcon test --return-code-on-test-failure
colcon test-result --verbose
```

## Troubleshooting

### An example prints nothing

Check that the terminal and the AutomatePro services use the same ROS domain, and that the service behind the example is running:

```bash
grep ROS_DOMAIN_ID /opt/automatepro/.env
echo $ROS_DOMAIN_ID
systemctl status automatepro-io-agent.service automatepro-core-driver.service
ros2 topic list
```

`gnss_position_node` prints only when a GNSS position receiver is connected, and `imu_node` prints magnetic field data only when the IMU driver publishes it.

### An output example logs "off command not delivered"

The node found no subscriber on its command topic when it stopped, so the IO controller did not receive the off command and the output state is unknown.
Fix the `ROS_DOMAIN_ID` or start `automatepro-io-agent.service`, then run the example again and stop it with Ctrl+C.

### `digital_in_node` prints only once

The IO controller publishes `/io/din` only when an input changes.
Request the current state again from another terminal:

```bash
ros2 service call /io/din/request automatepro_interfaces/srv/ReqDigitalIn
```

## License

Both packages are licensed under the Apache License 2.0; see [automatepro_cpp_tutorials/LICENSE](automatepro_cpp_tutorials/LICENSE) and [automatepro_python_tutorials/LICENSE](automatepro_python_tutorials/LICENSE).
