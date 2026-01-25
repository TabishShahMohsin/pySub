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
# Request data at 20Hz (Stream ID 0, Rate 20, Start/Stop 1)
master.mav.request_data_stream_send(
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 
    50, # 50 Hz
    1   # Start sending
)

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

THRUSTER_PINS = {
    "t1": 1, "t2": 2, "t3": 3, "t4": 4, 
    "t5": 8, "t6": 7, "t7": 6, "t8": 5  
}

last_command_time = time.time()
is_running = True

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
    sock = None
    while is_running:
        try:
            if sock is None:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Get Attitude (Roll, Pitch, Yaw)
            attn = master.recv_match(type='ATTITUDE', blocking=False)
            if attn is not None:
                telemetry["roll"] = np.degrees(attn.roll)
                telemetry["pitch"] = np.degrees(attn.pitch)
                telemetry["yaw"] = np.degrees(attn.yaw)
            # Get Pressure (Bar30 on Pixhawk I2C)
            pres = master.recv_match(type='SCALED_PRESSURE2', blocking=False)
            if pres is not None:
                telemetry["pressure"] = pres.press_abs
            telemetry["timestamp"] = time.time()
            message = json.dumps(telemetry).encode()
            sock.sendto(message, (PC_IP, UDP_PORT_DATA))
        except Exception as e:
            print(f"Sensor Socket Error: {e}. Retrying...")
            if sock: sock.close()
            sock = None # Force recreation
        time.sleep(0.05)


def command_receiver():
    global last_command_time, pwms

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((PI_IP, UDP_PORT_CMD))
    sock.settimeout(0.5)

    while is_running:
        try:
            data, addr = sock.recvfrom(1024)

            try:
                new_cmds = json.loads(data.decode())
            except json.JSONDecodeError:
                print("Invalid JSON received")
                continue

            updated = False
            for key, val in new_cmds.items():
                if key in pwms and isinstance(val, int):
                    pwms[key] = max(1150, min(1850, val))
                    updated = True

            if updated:
                master.mav.rc_channels_override_send(
                    master.target_system, master.target_component,
                    pwms['t1'], pwms['t2'], pwms['t3'], pwms['t4'],
                    pwms['t5'], pwms['t6'], pwms['t7'], pwms['t8']
                )
                last_command_time = time.time()

        except socket.timeout:
            continue

        except OSError as e:
            print(f"Socket error: {e}, rebinding...")
            sock.close()
            time.sleep(1)
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((PI_IP, UDP_PORT_CMD))
            sock.settimeout(0.5)

    sock.close()


# --- Main Logic ---
try:
    stop_all_thrusters()
    
    t_sender = threading.Thread(target=sensor_sender, daemon=True)
    t_receiver = threading.Thread(target=command_receiver, daemon=True)
    t_video = threading.Thread(target=video_stream_loop, daemon=True)

    t_sender.start()
    t_receiver.start()
    t_video.start()

    while True:

        # Ensuring NEUTRAL PWM when disconnected then erratic behaviour
        if time.time() - last_command_time > 1.0:
            stop_all_thrusters()
            print("Warning: Connection lost. Idling thrusters...", end='\r')
        else:

            # Getting the dashboard
            status_msg = f"P:{telemetry["pressure"]:>5.2f}m"
            p = pwms
            dashboard = (
                f"HORI_PWM:[{p['t1']:>4} {p['t2']:>4} {p['t3']:>4} {p['t4']:>4}] | "
                f"VERT_PWM:[{p['t5']:>4} {p['t6']:>4} {p['t7']:>4} {p['t8']:>4}] | {status_msg}"
            )
            print(f"{dashboard:<150}", end='\r', flush=True)
        
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nShutting down script.")
finally:
    is_running = False
    stop_all_thrusters()
    master.close()
