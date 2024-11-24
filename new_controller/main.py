# Subsystem controllers
from camera import Camera
from pid import PIDController
from ik import IK
from motor_serial import MotorSerial

import numpy as np
from time import sleep
import math

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
  # pid_controller_x = PIDController(0.2, 0.01, 0.08, 0.05, dead_zone=10)
  # pid_controller_y = PIDController(0.2, 0.01, 0.08, 0.05, dead_zone=10)
  pid_controller_x = PIDController(0.1, 0.00015, 0.05, 0.05, dead_zone=5, edge_threshold=60)
  pid_controller_y = PIDController(0.1, 0.00015, 0.05, 0.05, dead_zone=5, edge_threshold=60)
  # 0.1, 0.00015, 0.05, 0.05 (integral window of 2)
  bus = MotorSerial('/dev/tty.usbmodem11201')

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


      # roll = (normalized_x_error * MAX_TILT_ANGLE)
      # pitch = (normalized_y_error * MAX_TILT_ANGLE)
                             
      # Scaling angle
      roll = max(-15, min((normalized_x_error * MAX_TILT_ANGLE), 20))
      pitch = max(-15, min(-(normalized_y_error * MAX_TILT_ANGLE),20))


      print(f"Pitch: {pitch}")
      print(f"Roll: {roll}")

      stewart_platform_ik = IK(105.83, 99.92, 47.5, 190, np.radians(15), 0.146, 0)
      servo_angles = stewart_platform_ik.compute(np.array([pitch, roll, 0]))

      bus_angles = []
      for angle in servo_angles.tolist():
        # Make motor tilt more extreme 
        if angle > 70:
          print('***** EXCEPTION ******')
          angle = 70
        elif angle < -15:
          angle = -15

        bus_angles.append(int(angle))

      print(bus_angles)
      bus.send(bus_angles)

      sleep(0.05)
      # stewart_platform_ik.plot()
    else:
      print("Camera can't see anything")

    running = True
