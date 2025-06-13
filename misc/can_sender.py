import can
import sys
import time
import signal

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n[Sender] Stopping...")

signal.signal(signal.SIGINT, signal_handler)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 can_sender.py <interface>\nExample: python3 can_sender.py vcan0")
        sys.exit(1)

    interface = sys.argv[1]
    print(f"[Sender] Using interface: {interface}")

    try:
        bus = can.interface.Bus(channel=interface, bustype='socketcan')
    except OSError as e:
        print(f"[Sender] Failed to connect to interface {interface}: {e}")
        sys.exit(1)

    counter = 0

    while running:
        data = [counter & 0xFF, (counter >> 8) & 0xFF]
        msg = can.Message(arbitration_id=0x123, data=data, is_extended_id=False)

        try:
            bus.send(msg)
            print(f"[Sender] Sent ID 0x{msg.arbitration_id:X} Data: {' '.join(str(b) for b in msg.data)}")
        except can.CanError as e:
            print(f"[Sender] Send failed: {e}")

        counter += 1
        time.sleep(1)

    bus.shutdown()
    print("[Sender] Closed.")

if __name__ == "__main__":
    main()
