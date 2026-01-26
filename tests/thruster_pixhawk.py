from pymavlink import mavutil
import time

# Connect via USB
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Handshake successful!")

# 1. Force Manual Mode
master.set_mode('MANUAL')

# 2. Arm the vehicle
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)
print("Armed! Starting sequential spin...")

# 3. Test Channels 1-8
for i in range(1, 9):
    print(f"Testing Motor {i}...")
    pwms = [1500] * 8
    pwms[i-1] = 1560 # Low thrust to be safe
    
    # Send to Pixhawk
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        *pwms, 0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1.5)
    
    # Stop this motor before moving to next
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(0.5)

print("Test complete. Disarming.")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
)