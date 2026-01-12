import pygame
import numpy as np
import time
import json
import socket
import threading
from config import *
from input_handler import JoystickController
from rov_kinematics import compute_thruster_forces, map_force_to_pwm
from pid import PID
from kf import DepthKalmanFilter
import cv2
import imagezmq

shared_data = {
    # Shared from base station to pi
    "pwms": [1500] * 8,
    "running": True,
    # Shared from pi to base station
    "cpu_temp": 0,
    "timestamp": 0, # Heart beat
    "pressure": 0, 
    "depth": 0,
    "water_temp": 0,
    "roll": 0,
    "pitch": 0,   
    "yaw": 0,     
    "last_frames": {}
}

def video_receiver():
    """Listens for video streams and displays them in separate windows."""
    image_hub = imagezmq.ImageHub()
    print("[Thread] Video Receiver started. Waiting for frames...")
    
    while shared_data["running"]:
        try:
            # recv_image returns the name of the stream (e.g., 'auv_realsense') 
            # and the image itself
            cam_id, frame = image_hub.recv_image()
            
            # Display based on which camera sent it
            shared_data['last_frames'][cam_id] = frame
            # cv2.imshow(cam_id, frame) # Can't do this in demon thread
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break
                
            # Acknowledge receipt to the sender (required by imagezmq)
            image_hub.send_reply(b'OK')
            
        except Exception as e:
            print(f"Video Receiver Error: {e}")
            time.sleep(1)

    cv2.destroyAllWindows()

def command_sender():
    """Sends PWM commands at a steady frequency."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Allows the port to be reused immediately after a crash
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    print("[Thread] Command Sender started.")
    while shared_data["running"]:
        try:
            pwms = shared_data["pwms"]
            pwm_commands = {
                "t1": invert_pwm(pwms[0], I1), "t2": invert_pwm(pwms[1], I2), 
                "t3": invert_pwm(pwms[2], I3), "t4": invert_pwm(pwms[3], I4),
                "t5": invert_pwm(pwms[4], I5), "t6": invert_pwm(pwms[5], I6),
                "t7": invert_pwm(pwms[6], I7), "t8": invert_pwm(pwms[7], I8),
            }
            sock.sendto(json.dumps(pwm_commands).encode(), (PI_IP, UDP_PORT_CMD))
            time.sleep(0.05)  # 20Hz
        except Exception as e:
            print(f"Sender Error: {e}")
            time.sleep(1)

def telemetry_listener():
    """Listens for ROV feedback."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(("0.0.0.0", UDP_PORT_DATA))
        sock.settimeout(1.0) # Don't block forever if no data comes
    except OSError as e:
        print(f"Binding Error: {e}")
        return

    print("[Thread] Telemetry Listener started.")
    while shared_data["running"]:
        try:
            data, addr = sock.recvfrom(1024)
            telemetry = json.loads(data.decode())
            shared_data['cpu_temp'] = telemetry['cpu_temp']
            shared_data['timestamp'] = telemetry['timestamp']
            shared_data['pressure'] = telemetry['pressure'] - PRESSURE_OFFSET
            # shared_data['depth'] = telemetry['depth']
            shared_data['water_temp'] = telemetry['water_temp']
            shared_data['roll'] = telemetry['roll'] - ROLL_OFFSET
            shared_data['pitch'] = telemetry['pitch'] - PITCH_OFFSET
            shared_data['yaw'] = telemetry['yaw'] - YAW_OFFSET
            # In a real app, you'd save this to a global for the HUD to draw
        except socket.timeout:
            continue
        except Exception as e:
            print(f"Listener Error: {e}")


def main():
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    clock = pygame.time.Clock()

    try:
        controller = JoystickController(deadzone=0.1)
    except RuntimeError as e:
        print(e)
        pygame.quit()
        return

    kf = DepthKalmanFilter()

    target_depth = 0
    depth_pid = PID(DEPTH_KP, DEPTH_KI, DEPTH_KD, 1, -1)

    target_roll = 0
    roll_pid = PID(ROLL_KP, ROLL_KI, ROLL_KD, 1, -1, is_angle=True)

    target_pitch = 0
    pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, 1, -1, is_angle=True)

    target_yaw = 0
    yaw_pid = PID(YAW_KP, YAW_KI, YAW_KD, 1, -1, is_angle=True)

    thread1 = threading.Thread(target=telemetry_listener, daemon=True)
    thread2 = threading.Thread(target=command_sender, daemon=True)
    thread3 = threading.Thread(target=video_receiver, daemon=True)

    thread1.start()
    thread2.start()
    thread3.start()

    running = True
    while running:
        dt = clock.tick(30)/1000

        # Handle quit event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        p_curr = shared_data["pressure"]
        raw_depth = max(0, (p_curr - 1013.25) * 100 / (1025 * 9.81))

        if DEPTH_KF:
            measured_depth = kf.update(raw_depth, dt)
        else:
            measured_depth = raw_depth

        # Read joystick input
        raw_inputs = controller.get_input_vector()
        raw_surge, raw_sway, raw_heave, raw_roll, raw_pitch, raw_yaw = raw_inputs

        if DEPTH_PID:
            target_depth += raw_heave * 0.5 * dt
            heave_command = depth_pid.compute(measured_depth, target_depth, dt)
        else:
            heave_command = raw_heave

        if PITCH_PID:
            target_pitch += raw_pitch * 20 * dt
            pitch_command = pitch_pid.compute(shared_data['pitch'], target_pitch, dt)
        else:
            pitch_command = raw_pitch

        if ROLL_PID:
            target_roll += raw_roll * 20 * dt
            roll_command = roll_pid.compute(shared_data['roll'], target_roll, dt)
        else:
            roll_command = raw_roll

        if YAW_PID:
            target_yaw += raw_yaw * 20 * dt
            yaw_command = yaw_pid.compute(shared_data['yaw'], target_yaw, dt)
        else:
            yaw_command = raw_yaw

        # Get thruster force distribution
        thruster_forces = compute_thruster_forces(raw_surge, raw_sway, heave_command, roll_command, pitch_command, yaw_command)

        # Convert forces to PWM
        thruster_pwms = [map_force_to_pwm(f) for f in thruster_forces]
        shared_data['pwms'] = thruster_pwms

        p = shared_data["pwms"]
        pi_temp = shared_data["water_temp"]
        f = thruster_forces
        
        dashboard = (
            f"\033[H" +  # Move cursor to top-left (Home)
            f"\n"*20 +
            f"--- ROV_SEA-6.0 DASHBOARD ---\n"
            f"SYSTEM: Pressure: {p_curr:>7.2f} mb | Pi Temp: {pi_temp:>4.1f}°C\n"
            f"{'-'*60}\n"
            f"THRUSTERS (Forces & PWMs):\n"
            f"  Horizontal: T1:{f[0]:>6.2f}({p[0]}) T2:{f[1]:>6.2f}({p[1]}) T3:{f[2]:>6.2f}({p[2]}) T4:{f[3]:>6.2f}({p[3]})\n"
            f"  Vertical:   T5:{f[4]:>6.2f}({p[4]}) T6:{f[5]:>6.2f}({p[5]}) T7:{f[6]:>6.2f}({p[6]}) T8:{f[7]:>6.2f}({p[7]})\n"
            f"{'-'*60}\n"
            f"NAVIGATION:      {'[SETPOINT]':<15} {'[MEASURED]':<15}\n"
            f"  Depth (m):     {target_depth:>15.2f} {measured_depth:>15.2f}\n"
            f"  Roll  (°):     {target_roll:>15.2f} {shared_data['roll']:>15.2f}\n"
            f"  Pitch (°):     {target_pitch:>15.2f} {shared_data['pitch']:>15.2f}\n"
            f"  Yaw   (°):     {target_yaw:>15.2f} {shared_data['yaw']:>15.2f}\n"
            f"{'-'*60}\n"
            f"Status: RUNNING | Frequency: {clock.get_fps():.1f} FPS"
        )

        # Clear screen once at start or just use the Home cursor trick
        print(dashboard, end='', flush=False)

        cam_ids = list(shared_data['last_frames'].keys())
        for cam_id in cam_ids:
            frame = shared_data['last_frames'][cam_id]
            frame = frame[::-1]
            if frame is not None:
                cv2.imshow(cam_id, frame)
            
        # waitKey(1) is required to actually render the window
        if cv2.waitKey(1) & 0xFF == ord('q'):
            running = False

    # On exit: stop thrusters safely
    pygame.quit()
    shared_data["running"] = False
    print("\nSimulation exited.")

if __name__ == "__main__":
    main()
