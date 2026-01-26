from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Connected!")

def set_parameter(name, value):
    """Sets a parameter on the Pixhawk."""
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

# 1. SET PINS TO PASSTHROUGH 
# This tells Pin 1 to just follow Channel 1, Pin 2 to follow Channel 2, etc.
# 51 = RCIN1, 52 = RCIN2, etc.
print("Configuring pins for direct passthrough (no mixing)...")
for i in range(1, 9):
    param_name = f"SERVO{i}_FUNCTION"
    set_parameter(param_name, 50 + i) # 51 through 58
    time.sleep(0.1)

# 2. ARM
master.set_mode('MANUAL')
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)
print("Armed! Testing pins 1-8 individually...")

# 3. TEST LOOP
try:
    for i in range(1, 9):
        print(f"Spinning physical Pin {i} ONLY")
        pwms = [1500] * 8
        pwms[i-1] = 1600 # Set only the target pin to 1600
        
        # Send for 2 seconds
        t_end = time.time() + 2
        while time.time() < t_end:
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component,
                *pwms, 0, 0, 0, 0, 0, 0, 0, 0
            )
            time.sleep(0.1)
            
        # Stop
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 0, 0, 0, 0, 0, 0, 0, 0
        )
        time.sleep(1)

finally:
    # Cleanup: It is recommended to set SERVOx_FUNCTION back to 
    # Motor1...8 (values 33-40) if you want to use normal ROV modes later.
    print("Disarming.")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
