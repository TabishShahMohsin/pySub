"""Print the external Bar30 pressure sensor (SCALED_PRESSURE2) from the Pixhawk."""

import os

# Must be set before pymavlink is imported, otherwise MAVLink 1 is selected.
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil  # noqa: E402

DEVICE = "/dev/ttyACM0"
BAUD = 115200
SURFACE_PRESSURE_HPA = 1013.25
# 1 hPa = 100 Pa; depth = dP / (rho * g) with fresh water rho = 1000 kg/m^3
METRES_PER_HPA = 100 / (1000 * 9.80665)


def main():
    master = mavutil.mavlink_connection(DEVICE, baud=BAUD)
    master.wait_heartbeat()

    try:
        while True:
            msg = master.recv_match(type="SCALED_PRESSURE2", blocking=True, timeout=1.0)
            if msg is None:
                print("No SCALED_PRESSURE2 message received... check stream rates.")
                continue

            pressure = msg.press_abs  # hPa (mbar)
            temp = msg.temperature / 100.0  # centi-degrees -> °C
            depth = (pressure - SURFACE_PRESSURE_HPA) * METRES_PER_HPA
            print(f"Pressure: {pressure:.2f} hPa | Temp: {temp:.2f} C | Approx depth: {depth:.2f} m")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        master.close()


if __name__ == "__main__":
    main()
