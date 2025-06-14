import serial
import time
import signal
import sys

running = True

def signal_handler(sig, frame):
    global running
    print("\n[Sender] Stopping...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

def main():
    port = '/dev/ttyUSB0'
    baudrate = 9600

    print(f"[Sender] Opening serial port: {port}")
    try:
        ser = serial.Serial(port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                            timeout=1)
    except serial.SerialException as e:
        print(f"[Sender] Failed to open serial port: {e}")
        sys.exit(1)

    print("[Sender] Serial port ready. Press Ctrl+C to stop.")

    counter = 0
    while running:
        data = bytearray([counter & 0xFF, (counter >> 8) & 0xFF])
        ser.write(data)
        print(f"[Sender] Sent Data: {data[0]} {data[1]}")
        counter += 1
        time.sleep(1)

    ser.close()
    print("[Sender] Closed.")

if __name__ == "__main__":
    main()
