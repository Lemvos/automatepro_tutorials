import can
import sys
import signal

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n[Receiver] Stopping...")

signal.signal(signal.SIGINT, signal_handler)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 can_receiver.py <interface>\nExample: python3 can_receiver.py vcan0")
        sys.exit(1)

    interface = sys.argv[1]
    print(f"[Receiver] Listening on interface: {interface}")

    try:
        bus = can.interface.Bus(channel=interface, interface='socketcan')
    except OSError as e:
        print(f"[Receiver] Failed to connect to interface {interface}: {e}")
        sys.exit(1)

    while running:
        try:
            msg = bus.recv(timeout=1.0)
            if msg:
                print(f"[Receiver] Received ID 0x{msg.arbitration_id:X} Data: {' '.join(str(b) for b in msg.data)}")
        except can.CanError as e:
            print(f"[Receiver] Receive failed: {e}")

    bus.shutdown()
    print("[Receiver] Closed.")

if __name__ == "__main__":
    main()
