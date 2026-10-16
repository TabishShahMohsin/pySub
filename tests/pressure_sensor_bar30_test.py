import sys
import time

import ms5837

# Initialize sensor
# Default I2C bus is 1 (Standard for Raspberry Pi)
sensor = ms5837.MS5837_30BA()

"""
Black: GND
White: SDA: Pin 3
Green: SCL: Pin 5
Red: 3.3 v: Pin 1, Don't get this to 5v, it will break
"""


def initialize_sensor():
    print("--- Initializing MS5837 Pressure Sensor ---")
    if not sensor.init():
        print("Error: Sensor could not be initialized. Check I2C wiring.")
        sys.exit(1)

    # Set Fluid Density:
    # Freshwater = 997 kg/m^3 (Default)
    # Saltwater = 1029 kg/m^3
    sensor.setFluidDensity(ms5837.DENSITY_FRESHWATER)
    print("Sensor initialized successfully.\n")


def main():
    initialize_sensor()

    try:
        while True:
            # .read() updates all internal values from the hardware
            if sensor.read():
                depth = sensor.depth()  # Meters
                temp = sensor.temperature()  # Celsius
                press = sensor.pressure(ms5837.UNITS_mbar)  # Millibar

                # Format strings for a cleaner dashboard look
                output = f"Depth: {depth:6.2f} m | Temp: {temp:5.2f} C | Pressure: {press:8.2f} mbar"
                print(output, end="\r", flush=True)
            else:
                print("\nWarning: Sensor read failed!")

            time.sleep(0.1)  # 10Hz update rate is usually plenty for ROVs

    except KeyboardInterrupt:
        print("\nStopping sensor telemetry...")


if __name__ == "__main__":
    main()
