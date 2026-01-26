import socket
import threading
import time
import json
import cv2
import imagezmq
import pyrealsense2 as rs
import numpy as np
from pymavlink import mavutil
import os

# Initializing connection to PIXHAWK
os.environ['MAVLINK20'] = '1' # Force MAVLink 2.0
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()

THRUSTER_MAP = {
    1: 2, 2: 1, 3: 4, 4: 3,
    5: 6, 6: 7, 7: 5, 8: 8
}

def set_pixhawk_passthrough():
    """Sets all 8 pins to RCPassThrough mode (51-58)."""
    print("Configuring Pixhawk Passthrough...")
    for i in range(1, 9):
        param_name = f"SERVO{i}_FUNCTION"
        master.mav.param_set_send(
            master.target_system, master.target_component,
            param_name.encode('utf-8'), 50 + i, mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
    # Request telemetry stream
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 50, 1
    )

def arm_vehicle():
    master.set_mode('MANUAL')
    time.sleep(0.5)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Vehicle ARMED")

# Initializing socket
PC_IP = socket.gethostbyname('mba.local')
PI_IP = "0.0.0.0"        
UDP_PORT_DATA = 5005    
UDP_PORT_CMD = 5006     

# Initializing imagezmq
SENDER = imagezmq.ImageSender(connect_to=f'tcp://{PC_IP}:5555')
HOSTNAME = socket.gethostname()

# Global PWM States
pwms = {f"t{i}": 1500 for i in range(1, 9)}

telemetry = {
    "pressure": 1013.25,
    "timestamp":time.time(),
    "roll": 0,
    "pitch": 0,   
    "yaw": 0     
}

last_command_time = time.time()
is_running = True

def get_mapped_pwm_list():
    """Translates logical 't1-t8' values into the 18-channel MAVLink list."""
    output = [1500] * 18
    for t_num, physical_pin in THRUSTER_MAP.items():
        val = pwms[f"t{t_num}"]
        output[physical_pin - 1] = val
    return output[:8] # Returns first 8 for rc_override

def video_stream_loop():
    """Handles camera startup and automatic reconnection."""
    global is_running
    HOSTNAME = socket.gethostname()
    
    while is_running:
        pipeline = None
        sender = None
        
        try:
            print(f"[Video] Attempting to connect to Base Station at {PC_IP}...")
            sender = imagezmq.ImageSender(connect_to=f'tcp://{PC_IP}:5555')
            
            # 1. Start RealSense
            pipeline = rs.pipeline()
            rs_config = rs.config()
            rs_config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(rs_config)
            
            # Starting the logitech camera
            usb_cam = cv2.VideoCapture(2)
            
            print("[Video] Both streams started successfully.")

            while is_running:
                # Capture RealSense
                frames = pipeline.wait_for_frames(timeout_ms=100)
                color_frame = frames.get_color_frame()
                if color_frame:
                    rs_img = np.asanyarray(color_frame.get_data())
                    sender.send_image(f"{HOSTNAME}_realsense", rs_img)

            # USB Cam Frame
            ret, usb_frame = usb_cam.read()
            if ret:
                sender.send_image(f"usb_cam", usb_frame)

        except Exception as e:
            print(f"[Video] Stream error: {e}. Retrying in 3s...")
            # Cleanup before retry
            if pipeline: 
                try: pipeline.stop()
                except: pass
            if usb_cam: 
                try: usb_cam.release()
                except: pass
            if sender:
                try: sender.close()
                except: pass
            time.sleep(3)

def stop_all_thrusters():
    """Sets targets and current values back to neutral immediately."""
    print("!!! STOPPING ALL THRUSTERS (NEUTRAL) !!!")
    for key in pwms:
        pwms[key] = 1500
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500
    )

def sensor_sender():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while is_running:
        try:
            attn = master.recv_match(type='ATTITUDE', blocking=False)
            if attn:
                telemetry.update({"roll": np.degrees(attn.roll), "pitch": np.degrees(attn.pitch), "yaw": np.degrees(attn.yaw)})
            pres = master.recv_match(type='SCALED_PRESSURE2', blocking=False)
            if pres: telemetry["pressure"] = pres.press_abs
            
            telemetry["timestamp"] = time.time()
            sock.sendto(json.dumps(telemetry).encode(), (PC_IP, UDP_PORT_DATA))
        except: pass
        time.sleep(0.05)

def command_receiver():
    global last_command_time
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((PI_IP, UDP_PORT_CMD))
    sock.settimeout(0.5)
    
    while is_running:
        try:
            data, _ = sock.recvfrom(1024)
            new_cmds = json.loads(data.decode())
            for key, val in new_cmds.items():
                if key in pwms: pwms[key] = max(1150, min(1850, val))
            
            # Map logical commands to physical pins and send
            final_pwms = get_mapped_pwm_list()
            master.mav.rc_channels_override_send(
                master.target_system, master.target_component, *final_pwms
            )
            last_command_time = time.time()
        except: continue

try:
    set_pixhawk_passthrough()
    arm_vehicle()
    stop_all_thrusters()

    threading.Thread(target=sensor_sender, daemon=True).start()
    threading.Thread(target=command_receiver, daemon=True).start()
    threading.Thread(target=video_stream_loop, daemon=True).start() # Uncomment when ready

    while True:
        if time.time() - last_command_time > 1.0:
            stop_all_thrusters()
            print("Warning: Connection lost. Idling...", end='\r')
        else:
            p = pwms
            dashboard = f"T1:{p['t1']} T2:{p['t2']} T3:{p['t3']} T4:{p['t4']} | P:{telemetry['pressure']:.2f}"
            print(f"{dashboard:<100}", end='\r', flush=True)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nShutting down.")
finally:
    is_running = False
    stop_all_thrusters()
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0
    )
    master.close()
