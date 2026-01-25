from pymavlink import mavutil
import time

# Replace '/dev/ttyACM0' with your actual port if it differs
# The baud rate for USB is usually ignored, but 115200 is standard
device = '/dev/ttyACM0'
baud = 115200

print(f"Connecting to Pixhawk on {device}...")
master = mavutil.mavlink_connection(device, baud=baud)

# Wait for the first heartbeat 
# This confirms the connection is working
master.wait_heartbeat()
print("Heartbeat received! System (ID %u component %u)" % (master.target_system, master.target_component))

try:
    while True:
        # Request IMU data (RAW_IMU contains accel, gyro, and mag)
        # You can also look for 'SCALED_IMU' or 'HIGHRES_IMU'
        msg = master.recv_match(type='RAW_IMU', blocking=True)
        
        if msg:
            print("\n--- IMU DATA ---")
            print(f"Accel: x={msg.xacc}, y={msg.yacc}, z={msg.zacc}")
            print(f"Gyro:  x={msg.xgyro}, y={msg.ygyro}, z={msg.zgyro}")
            print(f"Mag:   x={msg.xmag}, y={msg.ymag}, z={msg.zmag}")
            
        time.sleep(0.1) # Adjust rate as needed

except KeyboardInterrupt:
    print("\nClosing connection.")

    
