import os
# Force MAVLink 2.0 before importing pymavlink
os.environ['MAVLINK20'] = '1'

from pymavlink import mavutil
import math
import time

device = '/dev/ttyACM0'
master = mavutil.mavlink_connection(device, baud=115200)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!")

# IMPORTANT: Tell Pixhawk to start sending ATTITUDE data (Stream ID 0)
# master.mav.request_data_stream_send(master.target_system, master.target_component, 
#                                     mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10, 1)

try:
    while True:
        # Use a timeout so the script doesn't hang forever if a packet is lost
        msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        
        if msg:
            roll = math.degrees(msg.roll)
            pitch = math.degrees(msg.pitch)
            yaw = math.degrees(msg.yaw)
            print(f"Roll: {roll:6.2f} | Pitch: {pitch:6.2f} | Yaw: {yaw:6.2f}")
        else:
            print("No ATTITUDE message received... check stream rates.")
            
except KeyboardInterrupt:
    print("\nExiting.")