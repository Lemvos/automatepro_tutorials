# AutomatePro Tutorials

ROS 2 Humble examples in Python and C++ for the AutomatePro sensors and IO controller.
The [AutomatePro documentation](https://docs.lemvos.com/automatepro/system-overview) describes the topics and messages they use.

## Contents

- [Packages](#packages)
- [Examples](#examples)
- [Requirements](#requirements)
- [Build](#build)
- [Run](#run)

## Packages

| Folder | Contents |
|---|---|
| `automatepro_python_tutorials` | Python package with the examples below |
| `automatepro_cpp_tutorials` | C++ package with the same examples |
| `misc` | CAN and RS485 examples outside ROS; see [misc/README.md](./misc/README.md) |

## Examples

Both packages provide the same executables.

| Executable | Topic | Behavior |
|---|---|---|
| `imu_node` | `/sensor/imu/data`, `/sensor/imu/magnetic_field` | Prints IMU and magnetic field data, at most once per second |
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
> The IO controller keeps the last command it receives, so stop these nodes with Ctrl+C: they switch their output off before they exit, which can take up to 4 seconds.
> A node that is killed with `kill -9` or crashes leaves its output in the last state.

## Requirements

- An AutomatePro unit with the AutomatePro software installed, which provides ROS 2 Humble packages including `automatepro_interfaces`.
- `automatepro-io-agent.service` running for the IO examples, and `automatepro-core-driver.service` running for the sensor examples.
- `colcon` and `rosdep`: `sudo apt install python3-colcon-common-extensions python3-rosdep`.
  If rosdep was never initialized on the unit, run `sudo rosdep init` and `rosdep update` once.
- A terminal whose `ROS_DOMAIN_ID` matches the AutomatePro services; see [Change the ROS Domain ID](https://docs.lemvos.com/automatepro/manual/misc/software-services#change-the-ros-domain-id).

## Build

```bash
mkdir -p ~/tutorials_ws/src
cd ~/tutorials_ws/src
git clone https://github.com/Lemvos/automatepro_tutorials.git
cd ~/tutorials_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Run

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
