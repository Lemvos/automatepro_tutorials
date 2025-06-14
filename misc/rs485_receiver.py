import serial
import time
import signal
import sys

running = True

def signal_handler(sig, frame):
    global running
    print("\n[Receiver] Stopping...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

def main():
    port = '/dev/ttyUSB0'
    baudrate = 9600

    print(f"[Receiver] Opening serial port: {port}")
    try:
        ser = serial.Serial(port, baudrate=baudrate, bytesize=serial.EIGHTBITS,
                            parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                            timeout=2)
    except serial.SerialException as e:
        print(f"[Receiver] Failed to open serial port: {e}")
        sys.exit(1)

    print("[Receiver] Serial port ready. Press Ctrl+C to stop.")

    while running:
        data = ser.read(2)
        if len(data) == 2:
            print(f"[Receiver] Received Data: {data[0]} {data[1]}")
        elif len(data) > 0:
            print(f"[Receiver] Partial Data: {list(data)}")
        else:
            print("[Receiver] Timeout - no data")
        time.sleep(0.5)

    ser.close()
    print("[Receiver] Closed.")

if __name__ == "__main__":
    main()
