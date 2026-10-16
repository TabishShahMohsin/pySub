"""Find the serial port of the HFI-A9 IMU."""

import serial.tools.list_ports


def get_imu_port():
    for port in serial.tools.list_ports.comports():
        # The HFI-A9 usually shows up as 'CH340', 'CP210' or 'USB Serial'
        desc = port.description.lower()
        if "usb" in desc or "serial" in desc or "ch340" in desc or "cp210" in desc:
            print(f"Found IMU on port: {port.device} ({port.description})")
            return port.device
    return None


def main():
    port_name = get_imu_port()
    if port_name:
        print(f"Connecting to {port_name}...")
        # ser = serial.Serial(port_name, 921600)
    else:
        print("No IMU detected. Check your USB connection.")


if __name__ == "__main__":
    main()
