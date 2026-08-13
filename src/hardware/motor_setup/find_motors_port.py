# find_motors_bus_port.py
# Minimal version of LeRobot's port-finder utility
import serial
import serial.tools.list_ports
import time

def checksum(data):
    return (~sum(data)) & 0xFF

def ping_packet(servo_id):
    pkt = [0xFF, 0xFF, servo_id, 0x02, 0x01]
    pkt.append(checksum(pkt[2:]))
    return bytearray(pkt)

def try_port(port_name, baud=1000000):
    try:
        ser = serial.Serial(port_name, baud, timeout=0.1)
        for test_id in range(1, 10):
            ser.write(ping_packet(test_id))
            time.sleep(0.05)
            if ser.in_waiting:
                print(f"✅ Motor with ID {test_id} responded on {port_name}")
                ser.close()
                return True
        ser.close()
    except Exception as e:
        pass
    return False

def main():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports detected.")
        return

    print("Scanning available ports...\n")
    found = False
    for p in ports:
        print(f"Trying {p.device} ...")
        if try_port(p.device):
            print(f"\n🟢 Found active motor bus on {p.device}\n")
            found = True
            break

    if not found:
        print("\nNo active motor bus detected. "
              "Make sure power is on and the adapter is connected.")

if __name__ == "__main__":
    main()
