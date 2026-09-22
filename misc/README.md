# CAN and RS485 Examples

C++ and Python programs that send and receive data on a CAN bus through SocketCAN and on an RS485 link through a serial port.
They run outside ROS and need no workspace.

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [CAN](#can)
  - [RS485](#rs485)
- [Project Structure](#project-structure)

## Overview

Each bus has a sender and a receiver, in C++ and in Python.
The senders transmit a 16-bit counter once per second, low byte first: in CAN frames with ID `0x123`, and as two raw bytes on RS485.
The receivers print what they receive.
All programs stop on Ctrl+C.

## Prerequisites

- `g++` for the C++ programs.
- Python 3 with `python-can` and `pyserial` for the Python programs:

  ```bash
  pip install python-can pyserial
  ```

- For CAN, a SocketCAN interface: `can0` on AutomatePro, or a virtual `vcan0` for testing, as shown under [CAN](#can).
- For RS485, membership of the `dialout` group.

## Installation

Build the C++ programs from this folder:

```bash
g++ -o can_sender can_sender.cpp
g++ -o can_receiver can_receiver.cpp
g++ -o rs485_sender rs485_sender.cpp
g++ -o rs485_receiver rs485_receiver.cpp
```

The Python programs need no build.

## Usage

Run a receiver and a sender in separate terminals, from this folder.

### CAN

Each CAN program takes the interface name as its only argument.
On AutomatePro, `automatepro-start-can0.service` brings `can0` up at 500 kbit/s; pass `can0` instead of `vcan0` to use the CAN bus.
To test without a bus, create a virtual interface:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

Run a receiver:

```bash
./can_receiver vcan0
python3 can_receiver.py vcan0
```

Run a sender:

```bash
./can_sender vcan0
python3 can_sender.py vcan0
```

Remove the virtual interface when you are done:

```bash
sudo ip link delete vcan0
```

### RS485

Each RS485 program opens `/dev/ttyTHS1`, the AutomatePro RS485 port, unless you pass another serial device as its only argument, for example `./rs485_sender /dev/ttyUSB0` for a USB-RS485 adapter on a PC.
On AutomatePro the 5G modem provides the `/dev/ttyUSB*` ports, so do not pass one of those.
The link runs at 9600 baud, 8 data bits, no parity, and 1 stop bit.

Run a receiver:

```bash
./rs485_receiver
python3 rs485_receiver.py
```

Run a sender:

```bash
./rs485_sender
python3 rs485_sender.py
```

## Project Structure

```text
misc/
├── can_receiver.cpp
├── can_receiver.py
├── can_sender.cpp
├── can_sender.py
├── rs485_receiver.cpp
├── rs485_receiver.py
├── rs485_sender.cpp
├── rs485_sender.py
└── README.md
```
