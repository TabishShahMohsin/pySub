import math
import socket
import numpy as np

'''
        ---------------------------width----------------------------
    |    T1 (CW)                                         T2 (CW)
    |
    |                      
    |           T5 (CCW)                          T6 (CW)
    |                               x
    |                               |  
  length                            o ----- y 
    |                                \
    |                                 z (inwards)
    |
    |           T7 (CW)                            T8 (CCW)
    |
    |
    |   T3 (CCW)                                           T4 (CCW)

    Force exerted by a thruster is taken +ve in the sense of pointing to x-axis for the lateral thrusters, and downwards for the vertical thrusters
    Irrespective of CW/CCW propeller
    SI units are used: m, s

'''

# PI_IP = "192.168.137.2"
PI_IP = socket.gethostbyname("auv.local")
UDP_PORT_DATA = 5005
UDP_PORT_CMD = 5006

ROV_WIDTH_MM = 262.629
ROV_LENGTH_MM = 195.311

# "V" Configuration: T1(45), T2(-45), T3(135), T4(-135)
THRUSTER_ANGLES_DEG = [45, -45, 135, -135] # From +ve x-axis (Force exerted to the vehicle)

SIN_45 = math.sin(math.radians(45))

# MAX thrust offered by an individual thruster
# This change was made due to problems in cancelling moments: from fixing PWM ranges to thrust ranges
MAX_THRUST = 2.35 
MAX_THRUST = 0.55 

PWM_NEUTRAL = 1500

XBOX = "XBOX"
PS = "PS"
KEYBAORD = "KEYBOARD"
CONTROLLER_TYPE = KEYBAORD

# Found mechanical team messing up the connections, as exchanging 2 would invert the direction of thrust
# Also 4 propellers must in be in one sense and the other 4 in opp sense for cancelling counter rotor torque
I1 = False
I2 = True
I3 = True
I4 = True 
I5 = True
I6 = True
I7 = False
I8 = False

def invert_pwm(PWM, invert=True):
    if invert == True:
        return PWM_NEUTRAL - (PWM - PWM_NEUTRAL)
    return PWM


# Due to unreliable electronics some thrusters tend to fail, only 6 thrusters with 3 lateral and 3 vertical are reuqired to attain 6 DOFs
# However cacelling the counter rotor torque with one less thruster is impossible and hence unaccounted in such a failure
W1 = True
W2 = True
W3 = True
W4 = True
W5 = True
W6 = True
W7 = True
W8 = True

WORKING_THRUSTERS = np.array([W1, W2, W3, W4, W5, W6, W7, W8])

# Need to find these again for the working thrusters otherwise they will max out before the joystick reaches extrema
MAX_AXIAL_FORCE = 4 * SIN_45 * MAX_THRUST
MAX_YAW_TORQUE = 2 * SIN_45 * (ROV_LENGTH_MM + ROV_WIDTH_MM) * MAX_THRUST
MAX_HEAVE_FORCE = 4 * MAX_THRUST
MAX_ROLL_TORQUE = 4 * (ROV_WIDTH_MM / 2) * MAX_THRUST
MAX_PITCH_TORQUE = 4 * (ROV_LENGTH_MM / 2) * MAX_THRUST

# To be implemented
RECORD_SENSORS = False
RECORD_FOOTAGE = False

# Things to Calibrate:

DEPTH_KF = False

PRESSURE_OFFSET = 988 - 1013.25 # mbar # Diff for location and whether
ROLL_OFFSET = 0
PITCH_OFFSET = 0
YAW_OFFSET = 50 # Make this in the direction of the gate

# meter to -1 to 1
DEPTH_PID = False
DEPTH_KP = 1.2
DEPTH_KI = 0.1
DEPTH_KD = 0.4

# Angle in degrees to -1 to 1
PITCH_PID = False
PITCH_KP = 0.04
PITCH_KI = 0.005
PITCH_KD = 0.03

# Angle in degrees to -1 to 1
ROLL_PID = False
ROLL_KP = 0.04
ROLL_KI = 0.005
ROLL_KD = 0.03

# Angle in degrees to -1 to 1
YAW_PID = False
YAW_KP = 0.04
YAW_KI = 0.0005
YAW_KD = 0.03
