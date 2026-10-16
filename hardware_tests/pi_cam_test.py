"""Show the Raspberry Pi camera feed.

Setup:
    sudo apt update
    sudo apt install python3-picamera2 python3-opencv

Find cameras:
    rpicam-hello --list-cameras
    v4l2-ctl --list-devices
"""

import cv2
from picamera2 import Picamera2


def main():
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()

    print("Press 'q' to quit")
    try:
        while True:
            frame = picam2.capture_array()  # RGB array
            # Picamera2 gives RGB, OpenCV expects BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.imshow("RPi Camera Feed", frame_bgr)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        picam2.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
