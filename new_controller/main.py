# Subsystem controllers
from camera import Camera
from pid import PIDController
from ik import IK
from motor_serial import MotorSerial

import numpy as np
from time import sleep

HERTZ = 60

TARGET_X = 0
TARGET_Y = 0

MAX_MM = 150

MAX_TILT_ANGLE = 20

if __name__ == "__main__":
  # Initializations

  # Camera
  # IK
  # PID(s)
  # I2C interface

  # while True:

  camera = Camera(camera_index=0)
  pid_controller_x = PIDController(1, 0, 0, 1/HERTZ)
  pid_controller_y = PIDController(1, 0, 0, 1/HERTZ)

  bus = MotorSerial('/dev/tty.usbmodem21301')

  running = True

  while running:
    coordinates = camera.compute_x_and_y()
    print(f"Ball Coords: {coordinates}")
    if (coordinates[0] and coordinates[1]):
      x_error_mm = pid_controller_x.compute(coordinates[0])
      y_error_mm = pid_controller_y.compute(coordinates[1])

      # Normalizing error
      normalized_x_error = x_error_mm / MAX_MM
      normalized_y_error = y_error_mm / MAX_MM

      # Scaling angle
      x_tilt = (normalized_x_error * MAX_TILT_ANGLE)
      y_tilt = (normalized_y_error * MAX_TILT_ANGLE)

      print(f"X tilt: {x_tilt}")
      print(f"Y tilt: {y_tilt}")

      stewart_platform_ik = IK(105.83, 99.92, 47.5, 190, np.radians(15), 0.146, 0)
      servo_angles = stewart_platform_ik.compute(np.array([x_tilt, y_tilt, 0]))
      bus.send([int(x) for x in servo_angles.tolist()])

      sleep(0.05)
    else:
      print("Camera can't see anything")

    running = True
