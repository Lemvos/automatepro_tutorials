# CAN and RS485 Communication Examples

This folder contains C++ and Python implementations of sender and receiver programs using:

- **CAN Bus** (via `socketcan`)
- **RS485 Serial Communication** (via `/dev/ttyTHS1` by default)

Each mode has:
- A sender and a receiver implementation
- Implementations in both **C++** and **Python**

---

## 📁 File Structure

| Type          | Language | Sender                | Receiver                |
|---------------|----------|-----------------------|-------------------------|
| CAN           | C++      | `can_sender.cpp`      | `can_receiver.cpp`      |
| CAN           | Python   | `can_sender.py`       | `can_receiver.py`       |
| RS485 Serial  | C++      | `rs485_sender.cpp`    | `rs485_receiver.cpp`    |
| RS485 Serial  | Python   | `rs485_sender.py`     | `rs485_receiver.py`     |

---

## ⚙️ Prerequisites

### For C++ programs
- `g++` compiler
- SocketCAN support (`vcan` or `can0` for testing)
- Serial port access: membership of the `dialout` group

### For Python programs
- Python 3
- Install dependencies:

```bash
pip install python-can pyserial
```

## CAN Bus Examples
### Setup Virtual CAN (vcan) for Testing or can0
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```
On AutomatePro, `automatepro-start-can0.service` brings `can0` up at 500 kbit/s; pass `can0` instead of `vcan0` to use the CAN bus.
### Build C++ CAN Programs
```bash
g++ -o can_sender can_sender.cpp
g++ -o can_receiver can_receiver.cpp
```
### Run CAN Receivers
```bash
./can_receiver vcan0
python3 can_receiver.py vcan0
```
### Run CAN Senders
```bash
./can_sender vcan0
python3 can_sender.py vcan0
```

## RS485 Serial Examples
Each RS485 program opens `/dev/ttyTHS1`, the AutomatePro RS485 port, unless you pass another serial device as its only argument, for example `./rs485_sender /dev/ttyUSB0` for a USB-RS485 adapter on a PC.
On AutomatePro the 5G modem provides the `/dev/ttyUSB*` ports, so do not pass one of those.
### Build C++ RS485 Programs
```bash
g++ -o rs485_sender rs485_sender.cpp
g++ -o rs485_receiver rs485_receiver.cpp
```
###  Run RS485 Receivers
```bash
./rs485_receiver
python3 rs485_receiver.py
```
###  Run RS485 Senders
```bash
./rs485_sender
python3 rs485_sender.py
```


