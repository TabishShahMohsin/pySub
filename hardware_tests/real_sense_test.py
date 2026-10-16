"""Show RGB, depth and both IR streams from a RealSense D435i, plus IMU readings.

Needs a USB 3.0 cable for the gyro / accelerometer streams.
Install: pip install pyrealsense2 (building librealsense on the Pi is hard).
"""

import sys

import cv2
import numpy as np
import pyrealsense2 as rs

RES_W, RES_H = 640, 480  # Reduced resolution for better performance


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, RES_W, RES_H, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, RES_W, RES_H, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.infrared, 1, RES_W, RES_H, rs.format.y8, 30)
    config.enable_stream(rs.stream.infrared, 2, RES_W, RES_H, rs.format.y8, 30)
    config.enable_stream(rs.stream.accel)
    config.enable_stream(rs.stream.gyro)

    profile = pipeline.start(config)

    # Turn the IR emitter off
    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 0)

    colorizer = rs.colorizer()
    accel = gyro = None

    try:
        while True:
            # wait_for_frames() already paces the loop at the camera rate; an
            # extra sleep here would only make frames queue up and lag.
            frames = pipeline.wait_for_frames()

            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            ir_left = frames.get_infrared_frame(1)
            ir_right = frames.get_infrared_frame(2)

            # Motion frames run at a different rate and aren't in every frameset
            accel_frame = frames.first_or_default(rs.stream.accel)
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            if accel_frame:
                accel = accel_frame.as_motion_frame().get_motion_data()
            if gyro_frame:
                gyro = gyro_frame.as_motion_frame().get_motion_data()

            if not (depth_frame and color_frame and ir_left and ir_right):
                continue

            depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir_left_image = cv2.cvtColor(np.asanyarray(ir_left.get_data()), cv2.COLOR_GRAY2BGR)
            ir_right_image = cv2.cvtColor(np.asanyarray(ir_right.get_data()), cv2.COLOR_GRAY2BGR)

            top_row = np.hstack((color_image, depth_image))
            bottom_row = np.hstack((ir_left_image, ir_right_image))
            display = np.vstack((top_row, bottom_row))
            cv2.imshow("D435i: RGB | Depth | IR-L | IR-R", cv2.resize(display, (1280, 720)))

            if accel is not None and gyro is not None:
                status = (
                    f"\rTS: {frames.get_timestamp():.2f} | "
                    f"ACCEL: x:{accel.x:>5.2f} y:{accel.y:>5.2f} z:{accel.z:>5.2f} | "
                    f"GYRO: x:{gyro.x:>5.2f} y:{gyro.y:>5.2f} z:{gyro.z:>5.2f}"
                )
                sys.stdout.write(status)
                sys.stdout.flush()

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\nStreams stopped.")


if __name__ == "__main__":
    main()
