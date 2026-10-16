"""Read the Bar30 (MS5837-30BA) pressure sensor directly over I2C.

Wiring (Raspberry Pi):
    Black: GND
    White: SDA - pin 3
    Green: SCL - pin 5
    Red:   3.3 V - pin 1 (do NOT connect to 5 V, it will break)
"""

import sys
import time

import ms5837

READ_PERIOD_S = 0.1  # 10 Hz is plenty for an ROV


def initialize_sensor():
    print("--- Initializing MS5837 pressure sensor ---")
    sensor = ms5837.MS5837_30BA()  # Default I2C bus 1 (standard on the Pi)
    if not sensor.init():
        print("Error: sensor could not be initialized. Check I2C wiring.")
        sys.exit(1)

    # Fresh water = 997 kg/m^3 (default), salt water = 1029 kg/m^3
    sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
    print("Sensor initialized successfully.\n")
    return sensor


def main():
    sensor = initialize_sensor()

    try:
        while True:
            # read() updates all internal values from the hardware
            if sensor.read():
                depth = sensor.depth()  # m
                temp = sensor.temperature()  # °C
                press = sensor.pressure(ms5837.UNITS_mbar)  # mbar
                output = f"Depth: {depth:6.2f} m | Temp: {temp:5.2f} C | Pressure: {press:8.2f} mbar"
                print(output, end="\r", flush=True)
            else:
                print("\nWarning: sensor read failed!")
            time.sleep(READ_PERIOD_S)
    except KeyboardInterrupt:
        print("\nStopping sensor telemetry...")


if __name__ == "__main__":
    main()
