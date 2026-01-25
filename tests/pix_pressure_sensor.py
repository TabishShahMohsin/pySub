import os
os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()

try:
    while True:
        # We look for SCALED_PRESSURE2 for the external Bar30 sensor
        msg = master.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=1.0)
        
        if msg:
            pressure = msg.press_abs  # Pressure in hPa (mbar)
            temp = msg.temperature / 100.0  # Convert centi-degrees to Celsius
            
            # Simple depth calculation: (Pressure - Surface Pressure) / (density * gravity)
            # Assuming surface pressure is ~1013 hPa
            depth = (pressure - 1013.25) * 0.010197 
            
            print(f"Pressure: {pressure:.2f} hPa | Temp: {temp:.2f} C | Approx Depth: {depth:.2f} m")
            
except KeyboardInterrupt:
    print("\nStopped.")