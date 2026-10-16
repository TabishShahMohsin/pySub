"""Read the HFI-A9 serial IMU (Linux / macOS).

Requires: pip install pyserial
Windows demo: https://github.com/belovictor/handsfree_ros_imu/tree/master/demo/linux
"""

import math
import struct
import sys

import serial
import serial.tools.list_ports

PORT = "/dev/ttyUSB0"
BAUDRATE = 921600

HEADER_1 = 0xAA
HEADER_2 = 0x55
IMU_PACKET_LEN = 0x2C  # Gyro, accel, magnetometer
EULER_PACKET_LEN = 0x14  # Euler angles


def find_tty_usb():
    """List the connected USB serial devices."""
    print(
        f"The default IMU serial port is {PORT}. "
        "If multiple serial devices are detected, change PORT in this script."
    )
    ports = [port.device for port in serial.tools.list_ports.comports() if "USB" in port.device]
    print(f"Currently connected USB serial devices ({len(ports)}): {ports}")


def check_sum(data, check_data):
    """CRC-16 (Modbus) check of a packet."""
    crc = 0xFFFF
    for byte in bytearray(data):
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    swapped = ((crc & 0xFF) << 8) | (crc >> 8)
    return swapped == ((check_data[0] << 8) | check_data[1])


def bytes_to_floats(raw):
    """Decode little-endian IEEE 754 float32 values."""
    return list(struct.unpack(f"<{len(raw) // 4}f", bytes(raw)))


class ImuParser:
    """Byte-by-byte parser for the HFI-A9 serial protocol."""

    def __init__(self):
        self.buff = []
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.magnetometer = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.pending = [True, True]  # [IMU packet, Euler packet] still awaited
        self.screen_cleared = False

    def feed(self, byte):
        self.buff.append(byte)

        if self.buff[0] != HEADER_1:
            self.buff = []
            return
        if len(self.buff) < 3:
            return
        if self.buff[1] != HEADER_2:
            self.buff = []
            return
        # Wait until the full packet is received
        if len(self.buff) < self.buff[2] + 5:
            return

        data = self.buff
        self.buff = []
        length = data[2]

        if length == IMU_PACKET_LEN and self.pending[0]:
            if check_sum(data[2:47], data[47:49]):
                values = bytes_to_floats(data[7:47])
                self.angular_velocity = values[1:4]
                self.acceleration = values[4:7]
                self.magnetometer = values[7:10]
            else:
                print("Checksum failed")
            self.pending[0] = False
        elif length == EULER_PACKET_LEN and self.pending[1]:
            if check_sum(data[2:23], data[23:25]):
                values = bytes_to_floats(data[7:23])
                self.angle_degree = values[1:4]
            else:
                print("Checksum failed")
            self.pending[1] = False
        elif length not in (IMU_PACKET_LEN, EULER_PACKET_LEN):
            print(f"No parser implemented for packet length: {length}, or invalid data")
            return

        # Print once both packets have arrived
        if any(self.pending):
            return
        self.pending = [True, True]
        self.display()

    def display(self):
        acc = self.acceleration
        acc_norm = math.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2) or 1.0
        gyro = self.angular_velocity
        ang = self.angle_degree
        mag = self.magnetometer

        if not self.screen_cleared:
            print("\033[2J", end="")  # Clear screen once
            self.screen_cleared = True
        print("\033[H", end="")

        print(
            f"""
Acceleration (m/s²):
    x-axis: {acc[0] * -9.8 / acc_norm:.2f}
    y-axis: {acc[1] * -9.8 / acc_norm:.2f}
    z-axis: {acc[2] * -9.8 / acc_norm:.2f}

Angular Velocity (rad/s):
    x-axis: {gyro[0]:.2f}
    y-axis: {gyro[1]:.2f}
    z-axis: {gyro[2]:.2f}

Euler Angles (degrees):
    roll : {ang[0]:.2f}
    pitch: {ang[1]:.2f}
    yaw  : {ang[2]:.2f}

Magnetic Field:
    x-axis: {mag[0]:.2f}
    y-axis: {mag[1]:.2f}
    z-axis: {mag[2]:.2f}
""",
            end="",
            flush=True,
        )


def main():
    find_tty_usb()

    try:
        hf_imu = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=0.5)
        if not hf_imu.is_open:
            hf_imu.open()
        print("Serial port opened successfully...")
    except serial.SerialException as e:
        print(e)
        print("Failed to open serial port")
        sys.exit(1)

    parser = ImuParser()
    try:
        while True:
            # read() blocks for up to `timeout` waiting for at least one byte,
            # so this loop doesn't busy-spin when no data is available.
            data = hf_imu.read(max(1, hf_imu.in_waiting))
            for byte in data:
                parser.feed(byte)
    except serial.SerialException as e:
        print(f"Exception: {e}")
        print("IMU disconnected or communication error")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        hf_imu.close()


if __name__ == "__main__":
    main()
