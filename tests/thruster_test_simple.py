import pigpio
import time
import sys

pi = pigpio.pi()

thruster_pins = [18, 27, 17, 22] # T1-4 # 2, 3, 4 are to be inverted
thruster_pins = [19, 6, 20, 13] # T5-8 # 5, 6 are to be inverted


NEUTRAL = 1500

for i in thruster_pins:

    print(f"Arming...{i}")
    pi.set_servo_pulsewidth(i, NEUTRAL)
    time.sleep(1)

    print(f"FWD....{i}")
    pi.set_servo_pulsewidth(i, 1600)
    time.sleep(5)

    print(f"dis-arming..{i}")
    pi.set_servo_pulsewidth(i, NEUTRAL)
    time.sleep(1)
    

    print(f"RVS....{i}")
    pi.set_servo_pulsewidth(i, 1400)
    time.sleep(1)

    print(f"dis-arming..{i}")
    pi.set_servo_pulsewidth(i, NEUTRAL)
    time.sleep(1)
    
    dummy = input("Enter to run next thruster: ")

pi.stop()
