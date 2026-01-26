from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Connected!")

# 1. PIN ORDER MAPPING (Software T# -> Physical Pin #)
LOGICAL_TO_PHYSICAL = {
    1: 2, 2: 1, 3: 4, 4: 3, 
    5: 6, 6: 7, 7: 5, 8: 8
}

# 2. INVERSION MAPPING (Which logical thrusters need to be reversed?)
# You specified Thrusters 2, 3, 4, and 6
INVERT_LIST = [2, 3, 4, 6]

def set_parameter(name, value):
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

def get_mapped_pwm(t_num, base_speed):
    """
    Calculates the correct PWM based on inversion needs.
    If neutral is 1500 and speed is 1600 (+100):
    Inverted becomes 1500 - 100 = 1400.
    """
    if t_num in INVERT_LIST:
        diff = base_speed - 1500
        return 1500 - diff
    return base_speed

# Configure Passthrough
print("Setting pins to Passthrough...")
for i in range(1, 9):
    set_parameter(f"SERVO{i}_FUNCTION", 50 + i) 
    time.sleep(0.05)

# ARM
master.set_mode('MANUAL')
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
)
print("Armed! Sequence: Corrected Order + Inversions Active.")

try:
    for t_num in range(1, 9):
        physical_pin = LOGICAL_TO_PHYSICAL[t_num]
        
        # Calculate speed (1600 normally, 1400 if inverted)
        test_speed = get_mapped_pwm(t_num, 1600)
        
        status = "INVERTED" if t_num in INVERT_LIST else "NORMAL"
        print(f"Testing T{t_num} ({status}) -> Physical Pin {physical_pin} @ PWM {test_speed}")
        
        pwms = [1500] * 8
        pwms[physical_pin - 1] = test_speed 
        
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
        time.sleep(0.5)

finally:
    print("Disarming.")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
